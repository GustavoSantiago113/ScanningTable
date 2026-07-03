"""pycolmap orchestration for calibration-plate camera geometry estimation.

Pipeline (see the paper's Section on Camera Properties and Geometry Estimation):
  1. Extract SIFT features for the virtual calibration photographs, telling COLMAP
     their intrinsics exactly (they are synthetic, so we know K exactly).
  2. Extract SIFT features for the real photographs with a physically-motivated
     initial focal length (see `estimate_focal_length_px`) that bundle adjustment
     refines further.
  3. Match everything exhaustively (virtual<->virtual, virtual<->real, real<->real).
  4. Seed a reconstruction containing only the virtual images at their known,
     exact poses, and triangulate the plate's 3D points from it.
  5. Continue incremental mapping from that seed with the virtual frames fixed
     (`fix_existing_frames`) and their camera kept constant (`constant_cameras`),
     so bundle adjustment only ever refines the real cameras' pose + intrinsics.
  6. Strip the virtual images out of the final reconstruction.
"""

from pathlib import Path

import cv2
import numpy as np
import pycolmap


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


def register_real_cameras(
    db_path: Path,
    work_dir: Path,
    seed_triangulated_dir: Path,
    output_dir: Path,
    virtual_camera_id: int,
    abs_pose_min_num_inliers: int = 15,
    abs_pose_min_inlier_ratio: float = 0.1,
    max_reg_trials: int = 1,
    max_runtime_seconds: int = 300,
    random_seed: int = 0,
    num_threads: int = 1,
) -> dict[int, pycolmap.Reconstruction]:
    """Grow the reconstruction onto the real images. Virtual frames stay fixed and
    their camera intrinsics stay constant; only the real cameras' pose and
    intrinsics (focal length + distortion) are refined by bundle adjustment.

    The absolute-pose RANSAC thresholds are relaxed from COLMAP's defaults
    (min 30 inliers / 0.25 ratio): a real photo only matches a handful of virtual
    renders well (different appearance domains - clean synthetic render vs. a
    printed pattern under real lighting/lens blur), so demanding the same
    match volume as real-vs-real photo pairs is unrealistic.

    `max_reg_trials=1` and `max_runtime_seconds` bound the worst case: some real
    photographs may just not carry enough calibration-plate signal to localise
    (e.g. the plate mostly occluded by the artefact at that rotation), and
    COLMAP's default of retrying each stubborn candidate 3x is very slow once a
    few dozen images are in play. Not every real photograph is guaranteed to
    register - the notebook reports how many did.

    `random_seed` pins COLMAP's PRNG (used by the RANSAC inside absolute-pose
    estimation), and `num_threads=1` makes the order images are attempted in and
    the order the PRNG gets drawn from deterministic too - both are needed for
    reproducible results. Left at COLMAP's defaults (seeded from system time,
    threads auto), registration outcomes vary dramatically run to run for this
    dataset - the borderline-inlier-count regime `abs_pose_min_num_inliers`
    sits right at the boundary of "registers vs. doesn't" for many real photos,
    and how many photos land on which side of that boundary is exactly what
    RANSAC's randomness (and the order it's consumed in) perturbs.
    """
    if output_dir.exists():
        import shutil
        shutil.rmtree(output_dir)

    pycolmap.set_random_seed(random_seed)

    options = pycolmap.IncrementalPipelineOptions()
    options.num_threads = num_threads
    options.fix_existing_frames = True
    options.constant_cameras = {virtual_camera_id}
    options.ba_refine_focal_length = True
    options.ba_refine_principal_point = False
    options.ba_refine_extra_params = True
    options.multiple_models = False
    options.max_runtime_seconds = int(max_runtime_seconds)
    options.mapper.abs_pose_min_num_inliers = abs_pose_min_num_inliers
    options.mapper.abs_pose_min_inlier_ratio = abs_pose_min_inlier_ratio
    options.mapper.max_reg_trials = max_reg_trials
    options.mapper.random_seed = random_seed
    options.mapper.num_threads = num_threads
    options.triangulation.random_seed = random_seed
    # COLMAP's "structure-less" registration fallback (registers an image from raw 2D-2D
    # correspondences when it doesn't yet see enough triangulated 3D points) produced degenerate,
    # near-identical poses for many real cameras in this setup - masked by silent Ceres solver
    # failures ("Unable to perform dense Cholesky factorization") during the bundle adjustment
    # that should have corrected them. Disabling it registers fewer real cameras, but every
    # registered pose is then actually independently estimated from real 2D-3D correspondences.
    options.structure_less_registration_fallback = False

    return pycolmap.incremental_mapping(
        database_path=db_path,
        image_path=work_dir,
        output_path=output_dir,
        options=options,
        input_path=seed_triangulated_dir,
    )


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
    """Some real cameras can converge to near-duplicate poses when bundle adjustment's
    linear solver fails to refine a copied initial guess (visible as "Unable to perform
    dense Cholesky factorization" warnings) - a poorly-conditioned-optimisation failure
    mode that shows up when localising monocular cameras against a single planar target.
    Real photos were taken at different physical table rotations, so two registered real
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
