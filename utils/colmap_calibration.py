"""pycolmap orchestration for calibration-plate camera geometry estimation.

Pipeline (see the paper's Section on Camera Properties and Geometry Estimation):
  1. Extract SIFT features for the virtual calibration photographs, telling COLMAP
     their intrinsics exactly (they are synthetic, so we know K exactly).
  2. Extract SIFT features for the real photographs with a physically-motivated
     initial focal length (see `estimate_focal_length_px`) that is kept fixed
     throughout - see the note on `register_real_cameras` for why this isn't
     refined further.
  3. Match everything exhaustively (virtual<->virtual, virtual<->real, real<->real).
  4. Seed a reconstruction containing only the virtual images at their known,
     exact poses, and triangulate the plate's 3D points from it.
  5. Localise each real photograph against that fixed model independently, via
     direct 2D-3D PnP (RANSAC + refinement) - see `register_real_cameras` for why
     this doesn't use COLMAP's incremental-mapping registration loop.
  6. Strip the virtual images out of the final reconstruction.
"""

from pathlib import Path

import cv2
import numpy as np
import pycolmap

from utils import plate_geometry


def estimate_focal_length_px(
    image_path: Path,
    output_width: int,
    pixel_pitch_um: float = 1.6,
) -> float | None:
    """A starting-point focal length (px, at `output_width`) for a real photo, from its
    EXIF FocalLength and an assumed sensor pixel pitch (default ~1.6um, typical of a
    quad-Bayer-binned 12MP smartphone sensor's binned readout).

    Why bother, when bundle adjustment refines focal length anyway: COLMAP's generic
    fallback (`default_focal_length_factor` x image width) can be far enough off that
    real-camera pose estimation converges to a wrong local minimum instead - focal
    length and depth are coupled/ambiguous when the only observed geometry is a single,
    mildly-tilted planar target, and a bad-enough initial guess can fall on the wrong
    side of that ambiguity for an entire image set. A physically-motivated starting
    point (still refined afterwards) is far more robust across different sets/rigs than
    a generic image-size-based heuristic. Returns None if the image has no usable EXIF.
    """
    from PIL import Image, ExifTags

    img = Image.open(image_path)
    exif = img._getexif()
    if not exif:
        return None
    tags = {ExifTags.TAGS.get(k, k): v for k, v in exif.items()}
    focal_mm = tags.get("FocalLength")
    if not focal_mm:
        return None
    focal_px_native = float(focal_mm) / (pixel_pitch_um * 1e-3)
    return focal_px_native * (output_width / img.size[0])


def resize_photographs(src_paths: list[Path], dst_dir: Path, max_long_edge: int | None) -> list[Path]:
    """Copy (optionally downscaled) real photographs into a working directory."""
    dst_dir.mkdir(parents=True, exist_ok=True)
    out_paths = []
    for src in src_paths:
        img = cv2.imread(str(src), cv2.IMREAD_COLOR)
        if img is None:
            raise ValueError(f"Could not read image: {src}")
        if max_long_edge is not None:
            h, w = img.shape[:2]
            scale = max_long_edge / max(h, w)
            if scale < 1.0:
                img = cv2.resize(img, (round(w * scale), round(h * scale)), interpolation=cv2.INTER_AREA)
        dst = dst_dir / src.name
        cv2.imwrite(str(dst), img)
        out_paths.append(dst)
    return out_paths


def build_database(
    db_path: Path,
    work_dir: Path,
    virtual_rel_names: list[str],
    real_rel_names: list[str],
    virtual_K: np.ndarray,
    real_camera_model: str = "SIMPLE_RADIAL",
    real_focal_length_factor: float = 1.2,
    real_focal_length_px: float | None = None,
    real_image_size: tuple[int, int] | None = None,
    max_num_features: int = 4096,
    random_seed: int = 0,
    num_threads: int = 1,
) -> None:
    """Extract SIFT features (virtual images with fixed known intrinsics, real images
    with an initial guess) and match every pair exhaustively.

    `random_seed` pins COLMAP's PRNG before matching's RANSAC-based two-view geometry
    verification runs. That alone is not enough for reproducible results, though:
    COLMAP parallelises extraction/matching across a thread pool by default
    (`num_threads=-1`), and which thread consumes which pair of images - and so which
    calls draw from the PRNG in which order - is not deterministic between runs. Pin
    `num_threads=1` (the default here) for byte-identical results run to run; raise it
    for speed once you don't need that.

    `real_focal_length_px` (with `real_image_size`) sets the real cameras' *starting*
    focal length explicitly (e.g. from `estimate_focal_length_px`) instead of COLMAP's
    generic `real_focal_length_factor x image width` guess. See
    `estimate_focal_length_px` for why this matters - it's not just about speed of
    convergence, a poor starting point can make an entire image set fail to register.
    """
    db_path.parent.mkdir(parents=True, exist_ok=True)
    if db_path.exists():
        db_path.unlink()

    pycolmap.set_random_seed(random_seed)

    extraction_opts = pycolmap.FeatureExtractionOptions()
    extraction_opts.sift.max_num_features = max_num_features
    extraction_opts.num_threads = num_threads

    fx, cx, cy = virtual_K[0, 0], virtual_K[0, 2], virtual_K[1, 2]
    reader_virtual = pycolmap.ImageReaderOptions()
    reader_virtual.camera_model = "SIMPLE_PINHOLE"
    reader_virtual.camera_params = f"{fx},{cx},{cy}"

    pycolmap.extract_features(
        database_path=db_path,
        image_path=work_dir,
        image_names=virtual_rel_names,
        camera_mode=pycolmap.CameraMode.SINGLE,
        reader_options=reader_virtual,
        extraction_options=extraction_opts,
        device=pycolmap.Device.cpu,
    )

    reader_real = pycolmap.ImageReaderOptions()
    reader_real.camera_model = real_camera_model
    if real_focal_length_px is not None:
        rw, rh = real_image_size
        reader_real.camera_params = f"{real_focal_length_px},{rw / 2.0},{rh / 2.0},0.0"
    else:
        reader_real.default_focal_length_factor = real_focal_length_factor

    pycolmap.extract_features(
        database_path=db_path,
        image_path=work_dir,
        image_names=real_rel_names,
        camera_mode=pycolmap.CameraMode.SINGLE,
        reader_options=reader_real,
        extraction_options=extraction_opts,
        device=pycolmap.Device.cpu,
    )

    matching_opts = pycolmap.FeatureMatchingOptions()
    matching_opts.num_threads = num_threads
    pycolmap.match_exhaustive(database_path=db_path, matching_options=matching_opts, device=pycolmap.Device.cpu)


def build_seed_reconstruction(db_path: Path, virtual_views: list, virtual_rel_names: list[str]) -> pycolmap.Reconstruction:
    """A reconstruction containing only the virtual images, registered at their
    known-exact poses. This is the fixed 'calibration model' the real cameras get
    localised against.
    """
    db = pycolmap.Database.open(str(db_path))
    seed = pycolmap.Reconstruction()

    first_db_image = db.read_image_with_name(virtual_rel_names[0])
    virtual_camera = db.read_camera(first_db_image.camera_id)
    seed.add_camera_with_trivial_rig(virtual_camera)

    for view, name in zip(virtual_views, virtual_rel_names):
        db_image = db.read_image_with_name(name)
        image = pycolmap.Image(image_id=db_image.image_id, name=name, camera_id=virtual_camera.camera_id)
        cam_from_world = pycolmap.Rigid3d(pycolmap.Rotation3d(view.R), view.t)
        seed.add_image_with_trivial_frame(image, cam_from_world)

    return seed


def triangulate_seed(
    seed: pycolmap.Reconstruction,
    db_path: Path,
    work_dir: Path,
    output_dir: Path,
    random_seed: int = 0,
    num_threads: int = 1,
) -> pycolmap.Reconstruction:
    if output_dir.exists():
        import shutil
        shutil.rmtree(output_dir)

    pycolmap.set_random_seed(random_seed)
    options = pycolmap.IncrementalPipelineOptions()
    options.num_threads = num_threads
    options.mapper.random_seed = random_seed
    options.mapper.num_threads = num_threads
    options.triangulation.random_seed = random_seed

    return pycolmap.triangulate_points(
        reconstruction=seed,
        database_path=db_path,
        image_path=work_dir,
        output_path=output_dir,
        clear_points=True,
        refine_intrinsics=False,
        options=options,
    )


def _camera_center(cam_from_world: pycolmap.Rigid3d) -> np.ndarray:
    R = cam_from_world.rotation.matrix()
    t = np.array(cam_from_world.translation)
    return -R.T @ t


def _mirror_pose_across_plate(cam_from_world: pycolmap.Rigid3d) -> pycolmap.Rigid3d:
    """Reflect a camera pose through the plate's plane (Z=0).

    A camera localised only against points lying on a single plane (here, the
    calibration plate's triangulated points) has a well-known two-fold pose ambiguity:
    reflecting the camera centre through the plane and rebuilding a fresh
    look-at-the-origin rotation (`plate_geometry.look_at_origin`) gives a second pose
    with *identical* reprojection error to the first - confirmed empirically (refining
    from the mirrored pose leaves it unmoved, i.e. it's already a local optimum, not
    just a plausible guess). `estimate_and_refine_absolute_pose`'s RANSAC has no way to
    prefer one side over the other, so it silently returns whichever it lands on first;
    since the real rig always mounts the camera above the turntable, the positive-Z
    solution is always the physically correct one.
    """
    mirrored_center = _camera_center(cam_from_world) * np.array([1.0, 1.0, -1.0])
    R2, t2 = plate_geometry.look_at_origin(mirrored_center)
    return pycolmap.Rigid3d(pycolmap.Rotation3d(R2), t2)


def register_real_cameras(
    db_path: Path,
    seed_triangulated: pycolmap.Reconstruction,
    virtual_rel_names: list[str],
    real_rel_names: list[str],
    abs_pose_min_num_inliers: int = 15,
    abs_pose_max_error: float = 12.0,
    abs_pose_min_inlier_ratio: float = 0.1,
    random_seed: int = 0,
    num_threads: int = 1,
) -> tuple[pycolmap.Reconstruction, list[str]]:
    """Localise each real photograph independently against the fixed calibration
    model, via direct 2D-3D PnP (RANSAC + refinement) rather than COLMAP's
    incremental-mapping registration loop.

    This deliberately bypasses `pycolmap.incremental_mapping`/`IncrementalMapper`.
    That path (COLMAP's standard incremental-SfM registration) turned out to have a
    reproducible bug in this rig/frame configuration - many virtual-fixed frames in
    one rig, many growing real frames in another - where several real cameras'
    poses would silently collapse to an identical, wrong value instead of being
    independently estimated (visible as repeated Ceres "Unable to perform dense
    Cholesky factorization" warnings immediately before the collapse). It reproduced
    across multiple image sets and parameter combinations, so is treated as a library
    issue in this pycolmap build rather than something to route around with tuning.

    Each real photo's pose comes from its own `estimate_and_refine_absolute_pose`
    call: 2D keypoints in the real photo, matched (via the database's stored,
    geometrically-verified two-view matches) to the 3D points the virtual cameras
    already triangulated. Every real camera is therefore an independent LO-RANSAC +
    non-linear pose refinement - no shared mutable state between images, so the
    collapse bug above has no path to occur.

    Camera intrinsics (focal length, distortion) are never refined, only pose -
    deliberately, not just for lack of a per-image implementation. A joint bundle
    adjustment across all real cameras' shared intrinsics (tried and removed) is a
    second, independent way this rig/frame configuration misbehaves: regardless of
    which parameters are held constant, it reproducibly diverges once more than a
    handful of real cameras are involved (focal length or camera positions running
    off to absurd values, "NO_CONVERGENCE" after hitting the iteration cap). Some of
    that is inherent to the problem, not just this library - all the correspondences
    used to localise the real cameras lie on a single (near-)planar target, and
    jointly fitting focal length from a planar target's observations is a classically
    ill-conditioned problem (focal length and depth/distance trade off against each
    other almost for free). `estimate_focal_length_px`'s EXIF-derived starting guess
    is used as the final focal length instead.

    Returns (reconstruction, registered_real_names) - the reconstruction contains
    the fixed virtual cameras (unchanged) plus every real camera that met
    `abs_pose_min_num_inliers`.
    """
    import copy

    pycolmap.set_random_seed(random_seed)
    db = pycolmap.Database.open(str(db_path))

    real_camera_id = db.read_image_with_name(real_rel_names[0]).camera_id
    real_camera = db.read_camera(real_camera_id)

    result = copy.deepcopy(seed_triangulated)
    result.add_camera_with_trivial_rig(real_camera)

    estimation_opts = pycolmap.AbsolutePoseEstimationOptions()
    estimation_opts.ransac.max_error = abs_pose_max_error
    estimation_opts.ransac.min_inlier_ratio = abs_pose_min_inlier_ratio
    estimation_opts.ransac.random_seed = random_seed
    estimation_opts.ransac.num_threads = num_threads

    refinement_opts = pycolmap.AbsolutePoseRefinementOptions()
    refinement_opts.refine_focal_length = False
    refinement_opts.refine_extra_params = False

    registered_names = []
    for real_name in real_rel_names:
        real_db_image = db.read_image_with_name(real_name)
        real_id = real_db_image.image_id
        real_keypoints = db.read_keypoints(real_id)

        # Candidate 2D-3D correspondences for RANSAC. Deliberately *not* deduplicated
        # by real keypoint index here: the same real keypoint can legitimately get
        # matched (to the same, correct 3D point) from several virtual viewpoints, and
        # that redundancy is useful signal for RANSAC, not noise.
        candidate_real_idxs = []
        candidate_point3D_ids = []
        for virtual_name in virtual_rel_names:
            virtual_id = db.read_image_with_name(virtual_name).image_id
            a, b = min(virtual_id, real_id), max(virtual_id, real_id)
            if not db.exists_two_view_geometry(a, b):
                continue
            inlier_matches = db.read_two_view_geometry(a, b).inlier_matches
            if len(inlier_matches) == 0:
                continue
            virtual_image = seed_triangulated.find_image_with_name(virtual_name)
            for idx_a, idx_b in inlier_matches:
                virtual_idx, real_idx = (int(idx_a), int(idx_b)) if a == virtual_id else (int(idx_b), int(idx_a))
                p2d_virtual = virtual_image.points2D[virtual_idx]
                if not p2d_virtual.has_point3D():
                    continue
                candidate_real_idxs.append(real_idx)
                candidate_point3D_ids.append(p2d_virtual.point3D_id)

        if len(candidate_real_idxs) < abs_pose_min_num_inliers:
            continue

        points2D = [real_keypoints[i][:2] for i in candidate_real_idxs]
        points3D = [seed_triangulated.point3D(pid).xyz for pid in candidate_point3D_ids]

        points2D_arr = np.array(points2D)
        points3D_arr = np.array(points3D)
        pose = pycolmap.estimate_and_refine_absolute_pose(
            points2D_arr, points3D_arr, real_camera,
            estimation_options=estimation_opts, refinement_options=refinement_opts,
        )
        if pose is None or pose["num_inliers"] < abs_pose_min_num_inliers:
            continue

        # The plate's triangulated points are (near-)coplanar, so this PnP solve has a
        # two-fold ambiguity (see `_mirror_pose_across_plate`) and RANSAC has no reason
        # to prefer the physically-correct side. Flip back whenever it lands the camera
        # below the turntable. No re-refinement afterwards: the mirrored pose already has
        # the same reprojection error as the one `estimate_and_refine_absolute_pose` just
        # refined (that's the ambiguity), and re-running `refine_absolute_pose` from it is
        # actively harmful - observed to occasionally diverge to a wildly wrong pose,
        # since it's initialised exactly at a symmetric point where the problem is poorly
        # conditioned.
        if _camera_center(pose["cam_from_world"])[2] < 0:
            pose["cam_from_world"] = _mirror_pose_across_plate(pose["cam_from_world"])

        # Keep every detected keypoint (not just the candidates used for pose
        # estimation) so the image's Point2D indices line up with the database's
        # keypoint indices - needed for `add_observation` below and for any later
        # re-triangulation from this image.
        image = pycolmap.Image(
            name=real_name, keypoints=real_keypoints[:, :2],
            camera_id=real_camera.camera_id, image_id=real_id,
        )
        result.add_image_with_trivial_frame(image, pose["cam_from_world"])

        # One observation per real keypoint index (a Point2D can only reference one
        # Point3D): keep the first inlier correspondence seen for each index.
        added_real_idxs = set()
        for real_idx, point3D_id, is_inlier in zip(candidate_real_idxs, candidate_point3D_ids, pose["inlier_mask"]):
            if is_inlier and real_idx not in added_real_idxs:
                result.add_observation(point3D_id, pycolmap.TrackElement(real_id, real_idx))
                added_real_idxs.add(real_idx)

        registered_names.append(real_name)

    return result, registered_names


def filter_implausible_real_cameras(
    recon: pycolmap.Reconstruction,
    virtual_rel_names: set[str],
    expected_distance_mm: float,
    max_distance_ratio: float = 3.0,
) -> list[str]:
    """Deregister any real camera whose distance from the plate centre is wildly off
    from the virtual cameras' known distance - a sign that photo just didn't have
    enough good correspondences for a reliable pose, even if it nominally met the
    inlier-count threshold.

    (An earlier version of this also rejected cameras whose elevation angle deviated
    from the set's own median, on the theory that a fixed rig photographing a
    rotating plate should keep elevation ~constant across a set and only vary
    azimuth. That assumption doesn't hold robustly here: for some image sets more
    than half the individually-estimated poses land off the "correct" elevation
    (single-planar-target PnP is a poorly-conditioned problem prone to a
    geometrically-plausible-but-wrong solution), which makes a per-set median an
    unreliable reference - it can just as easily vote for the wrong cluster as the
    right one. Distance from a value we actually know in advance doesn't have that
    failure mode.)

    Mutates `recon` in place; returns the discarded names.
    """
    removed_names = []
    for img in list(recon.images.values()):
        if not img.has_pose or img.name in virtual_rel_names:
            continue
        distance = float(np.linalg.norm(img.projection_center()))
        ratio = max(distance / expected_distance_mm, expected_distance_mm / max(distance, 1e-6))
        if ratio > max_distance_ratio:
            recon.deregister_frame(img.frame_id)
            removed_names.append(img.name)
    return removed_names


def _mean_reprojection_error(recon: pycolmap.Reconstruction, img: pycolmap.Image) -> float:
    errors = []
    for p2d in img.points2D:
        if not p2d.has_point3D():
            continue
        projected = img.project_point(recon.point3D(p2d.point3D_id).xyz)
        if projected is not None:
            errors.append(float(np.linalg.norm(np.array(projected) - np.array(p2d.xy))))
    return float(np.mean(errors)) if errors else float("inf")


def filter_degenerate_real_cameras(
    recon: pycolmap.Reconstruction,
    virtual_rel_names: set[str],
    min_separation_mm: float = 20.0,
) -> list[str]:
    """Some real cameras can converge to near-duplicate poses when their independent
    PnP refinement (in `register_real_cameras`) fails to move a degenerate initial
    guess - a poorly-conditioned-optimisation failure mode that shows up when localising
    monocular cameras against a single planar target. Real photos were taken at
    different physical table rotations, so two registered real
    cameras ending up implausibly close together indicates one (or both) never actually
    got refined away from a bad initial guess, not a genuine coincidence.

    Detects clusters of real cameras whose projection centres are closer than
    `min_separation_mm` and keeps only the best-conditioned camera (lowest mean
    reprojection error) in each cluster, deregistering the rest. Mutates `recon` in
    place; returns the names of the images that were deregistered.
    """
    real_images = [img for img in recon.images.values() if img.has_pose and img.name not in virtual_rel_names]
    if len(real_images) < 2:
        return []

    centers = {img.image_id: np.array(img.projection_center()) for img in real_images}
    errors = {img.image_id: _mean_reprojection_error(recon, img) for img in real_images}
    names = {img.image_id: img.name for img in real_images}
    frame_ids = {img.image_id: img.frame_id for img in real_images}

    parent = {i: i for i in centers}

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    ids = list(centers)
    for i in range(len(ids)):
        for j in range(i + 1, len(ids)):
            a, b = ids[i], ids[j]
            if np.linalg.norm(centers[a] - centers[b]) < min_separation_mm:
                union(a, b)

    clusters: dict[int, list[int]] = {}
    for i in ids:
        clusters.setdefault(find(i), []).append(i)

    removed_names = []
    for cluster_ids in clusters.values():
        if len(cluster_ids) <= 1:
            continue
        best = min(cluster_ids, key=lambda i: errors[i])
        for i in cluster_ids:
            if i != best:
                recon.deregister_frame(frame_ids[i])
                removed_names.append(names[i])

    return removed_names


def strip_virtual_images(recon: pycolmap.Reconstruction, virtual_rel_names: set[str]) -> pycolmap.Reconstruction:
    """Return a copy of the reconstruction with the virtual calibration images
    deregistered, leaving only the real cameras and the points they observe.
    """
    import copy

    pruned = copy.deepcopy(recon)
    virtual_frame_ids = [
        img.frame_id for img in pruned.images.values() if img.name in virtual_rel_names
    ]
    for frame_id in virtual_frame_ids:
        pruned.deregister_frame(frame_id)
    return pruned


def registration_summary(recon: pycolmap.Reconstruction, virtual_rel_names: set[str]) -> dict:
    registered = [img for img in recon.images.values() if img.has_pose]
    n_virtual = sum(img.name in virtual_rel_names for img in registered)
    n_real = len(registered) - n_virtual
    return {
        "num_registered_virtual": n_virtual,
        "num_registered_real": n_real,
        "num_points3D": recon.num_points3D(),
        "mean_track_length": recon.compute_mean_track_length(),
        "mean_reprojection_error": recon.compute_mean_reprojection_error(),
    }
