"""Calibration-plate camera geometry with pycolmap.

  1. Render virtual photos of the plate pattern from known poses around it
     (plate_geometry) and extract SIFT from them with their exact intrinsics.
  2. Extract SIFT from the real photos, with a focal-length prior measured from the plate
     (`estimate_focal_length_from_plate`).
  3. Match everything exhaustively (virtual-virtual, virtual-real, real-real).
  4. Seed a reconstruction with only the virtual images at their exact poses and
     triangulate the plate's 3D points: the world frame is the plate, in millimetres.
  5. Register the real photos incrementally against that seed, virtual frames and their
     camera held fixed, so bundle adjustment only refines the real cameras.
  6. Drop implausible poses, fill photos COLMAP couldn't place from the turntable orbit,
     and strip the virtual images.

`search_calib_max_long_edge` repeats 1-5 at several real-photo resolutions until enough
photos register on a plausible orbit.

Determinism: SIFT extraction runs on the CPU (`_EXTRACTION_NUM_THREADS` threads) and
matching/mapping use `num_threads` with a fixed seed. Pin `_EXTRACTION_NUM_THREADS` and
`num_threads` to 1 for byte-identical runs: multi-threaded extraction/matching can change
which photos register from run to run (thread order changes image ids and PRNG
consumption).

World frame: the plate keeps this repo's rendering orientation (see
`plate_geometry.render_plate_photograph`), so real cameras land below z=0;
`cropping.correct_z_axis_inversion` flips that downstream. Everything here (distance
filter, orbit fit, height spread) is independent of that sign.
"""

import copy
import re
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import pycolmap
from scipy.spatial.transform import Rotation

from utils import plate_geometry

# the app names photos `<set>_stop_<NN>_<timestamp>.jpg` (older captures: `stop_<NN>.jpg`)
_STOP_NAME_RE = re.compile(r"stop_(\d+)(?=[._])")
_EXTRACTION_NUM_THREADS = 1


def estimate_focal_length_px(image_path: Path, output_width: int, pixel_pitch_um: float = 1.6) -> float | None:
    """Fallback focal length (px at `output_width`) from EXIF FocalLength and an assumed
    sensor pixel pitch; None without EXIF."""
    from PIL import Image, ExifTags

    img = Image.open(image_path)
    exif = img._getexif()
    if not exif:
        return None
    tags = {ExifTags.TAGS.get(k, k): v for k, v in exif.items()}
    focal_mm = tags.get("FocalLength")
    if not focal_mm:
        return None
    return float(focal_mm) / (pixel_pitch_um * 1e-3) * (output_width / img.size[0])


def resize_photographs(src_paths: list[Path], dst_dir: Path, max_long_edge: int | None) -> list[Path]:
    """Copy (optionally downscaled, INTER_AREA) real photographs into a working directory."""
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


def estimate_focal_length_from_plate(
    real_paths: list[Path],
    plate_image: np.ndarray,
    n_views: int = 12,
    pattern_long_edge_px: int = 1000,
    min_inliers: int = 40,
) -> float | None:
    """The real camera's focal length (px, at `real_paths`' resolution) measured from the
    plate: SIFT matches between `n_views` evenly spaced photos and the flat pattern, a
    RANSAC homography per photo, and Zhang's plane-calibration constraints solved for the
    focal length alone (principal point at the centre, square pixels, no distortion --
    bundle adjustment refines all of it). Median over the photos; None if fewer than 3
    photos give `min_inliers` homography inliers.

    Registering the first real photo rests on few correct real-virtual correspondences;
    without a focal prior COLMAP must also estimate the focal length there, which needs
    more of them. Photos agree within ~1.5% (~1100px at a 1500px long edge)."""
    pattern = plate_image if plate_image.ndim == 2 else cv2.cvtColor(plate_image, cv2.COLOR_RGB2GRAY)
    scale = pattern_long_edge_px / max(pattern.shape)
    pattern = cv2.resize(pattern, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    sift = cv2.SIFT_create(8000)
    kp_pattern, desc_pattern = sift.detectAndCompute(pattern, None)
    matcher = cv2.BFMatcher()
    focals = []
    for idx in np.linspace(0, len(real_paths) - 1, min(n_views, len(real_paths))).round().astype(int):
        photo = cv2.imread(str(real_paths[idx]), cv2.IMREAD_GRAYSCALE)
        if photo is None:
            continue
        h, w = photo.shape
        kp, desc = sift.detectAndCompute(photo, None)
        if desc is None or len(kp) < 2:
            continue
        good = [a for a, b in matcher.knnMatch(desc_pattern, desc, k=2) if a.distance < 0.8 * b.distance]
        if len(good) < min_inliers:
            continue
        src = np.float32([kp_pattern[m.queryIdx].pt for m in good])
        dst = np.float32([kp[m.trainIdx].pt for m in good])
        H, inliers = cv2.findHomography(src, dst, cv2.RANSAC, 3.0)
        if H is None or inliers is None or int(inliers.sum()) < min_inliers:
            continue
        # image coords centred on the principal point and scaled by the width, so
        # K = diag(f/w, f/w, 1) and omega = K^-T K^-1 = diag(a, a, 1) with a = (w/f)^2
        T = np.array([[1.0 / w, 0.0, -0.5], [0.0, 1.0 / w, -0.5 * h / w], [0.0, 0.0, 1.0]])
        Hn = T @ H
        Hn /= np.linalg.norm(Hn)
        h1, h2 = Hn[:, 0], Hn[:, 1]
        # h1' omega h2 = 0 and h1' omega h1 = h2' omega h2, linear in a
        A = np.array([h1[0] * h2[0] + h1[1] * h2[1], h1[0] ** 2 + h1[1] ** 2 - h2[0] ** 2 - h2[1] ** 2])
        b = -np.array([h1[2] * h2[2], h1[2] ** 2 - h2[2] ** 2])
        a = float(A @ b / (A @ A))
        if a > 0:
            focals.append(w / np.sqrt(a))
    return float(np.median(focals)) if len(focals) >= 3 else None


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
    cpu_brute_force_matcher: bool = False,
) -> None:
    """Extract SIFT (virtual images with their exact intrinsics, real images with
    `real_focal_length_px` as a prior or COLMAP's `real_focal_length_factor` x width guess)
    and match every pair exhaustively, all on the CPU.

    `cpu_brute_force_matcher`: exact nearest-neighbour matching instead of COLMAP's
    approximate CPU default -- recovers registrations on marginal captures, at ~30-40x the
    matching time (`search_calib_max_long_edge` uses it only as a last resort)."""
    db_path.parent.mkdir(parents=True, exist_ok=True)
    if db_path.exists():
        db_path.unlink()

    pycolmap.set_random_seed(random_seed)

    extraction_opts = pycolmap.FeatureExtractionOptions()
    extraction_opts.sift.max_num_features = max_num_features
    extraction_opts.num_threads = _EXTRACTION_NUM_THREADS

    fx, cx, cy = virtual_K[0, 0], virtual_K[0, 2], virtual_K[1, 2]
    reader_virtual = pycolmap.ImageReaderOptions()
    reader_virtual.camera_model = "SIMPLE_PINHOLE"
    reader_virtual.camera_params = f"{fx},{cx},{cy}"
    pycolmap.extract_features(
        database_path=db_path, image_path=work_dir, image_names=virtual_rel_names,
        camera_mode=pycolmap.CameraMode.SINGLE, reader_options=reader_virtual,
        extraction_options=extraction_opts, device=pycolmap.Device.cpu,
    )

    reader_real = pycolmap.ImageReaderOptions()
    reader_real.camera_model = real_camera_model
    if real_focal_length_px is not None:
        rw, rh = real_image_size
        reader_real.camera_params = f"{real_focal_length_px},{rw / 2.0},{rh / 2.0},0.0"
    else:
        reader_real.default_focal_length_factor = real_focal_length_factor
    pycolmap.extract_features(
        database_path=db_path, image_path=work_dir, image_names=real_rel_names,
        camera_mode=pycolmap.CameraMode.SINGLE, reader_options=reader_real,
        extraction_options=extraction_opts, device=pycolmap.Device.cpu,
    )

    matching_opts = pycolmap.FeatureMatchingOptions()
    matching_opts.num_threads = num_threads
    matching_opts.sift.cpu_brute_force_matcher = cpu_brute_force_matcher
    pycolmap.match_exhaustive(database_path=db_path, matching_options=matching_opts, device=pycolmap.Device.cuda)


def build_seed_reconstruction(db_path: Path, virtual_views: list, virtual_rel_names: list[str]) -> pycolmap.Reconstruction:
    """A reconstruction with only the virtual images, at their exact poses: the fixed
    calibration model the real cameras are registered against."""
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
    """Triangulate points for the registered images' matches (intrinsics kept fixed)."""
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
        reconstruction=seed, database_path=db_path, image_path=work_dir, output_path=output_dir,
        clear_points=True, refine_intrinsics=False, options=options,
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
    """Grow the seed reconstruction onto the real images. Virtual frames and their camera
    stay fixed; only the real cameras' poses and intrinsics (focal + k1) are refined.

    The absolute-pose thresholds are relaxed from COLMAP's defaults (30 inliers / 0.25):
    a real photo matches only a few virtual renders, and those weakly. Structure-less
    registration is disabled -- it produced degenerate, near-identical poses here."""
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
    options.structure_less_registration_fallback = False

    return pycolmap.incremental_mapping(
        database_path=db_path, image_path=work_dir, output_path=output_dir,
        options=options, input_path=seed_triangulated_dir,
    )


def filter_implausible_real_cameras(
    recon: pycolmap.Reconstruction,
    virtual_rel_names: set[str],
    expected_distance_mm: float,
    max_distance_ratio: float = 3.0,
) -> list[str]:
    """Deregister real cameras whose distance from the plate centre is off from the
    expected rig distance by more than `max_distance_ratio` (either way). Mutates `recon`;
    returns the discarded names."""
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
    min_separation_mm: float | None = None,
    min_separation_fraction: float = 0.25,
) -> list[str]:
    """Collapse clusters of near-duplicate real camera poses (a bundle-adjustment failure
    mode: cameras stuck at a copied initial guess) to their lowest-reprojection-error
    member. Two cameras are duplicates if closer than `min_separation_mm`, by default
    `min_separation_fraction` x the median nearest-neighbour spacing, so it adapts to the
    rig's radius. Mutates `recon`; returns the deregistered names."""
    real_images = [img for img in recon.images.values() if img.has_pose and img.name not in virtual_rel_names]
    if len(real_images) < 2:
        return []

    centers = {img.image_id: np.array(img.projection_center()) for img in real_images}
    errors = {img.image_id: _mean_reprojection_error(recon, img) for img in real_images}
    names = {img.image_id: img.name for img in real_images}
    frame_ids = {img.image_id: img.frame_id for img in real_images}

    if min_separation_mm is None:
        nearest = [min(np.linalg.norm(centers[i] - centers[j]) for j in centers if j != i) for i in centers]
        min_separation_mm = min_separation_fraction * float(np.median(nearest))

    parent = {i: i for i in centers}

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    ids = list(centers)
    for i in range(len(ids)):
        for j in range(i + 1, len(ids)):
            a, b = ids[i], ids[j]
            if np.linalg.norm(centers[a] - centers[b]) < min_separation_mm:
                ra, rb = find(a), find(b)
                if ra != rb:
                    parent[ra] = rb

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


# --- filling photos COLMAP couldn't register, from the turntable orbit ------------------


def _stop_index_from_name(name: str) -> int:
    """`"set_1/01_stop_07_20260718_122912.jpg"` -> 7: photos are named in capture order,
    one turntable step apart."""
    match = _STOP_NAME_RE.search(name)
    if match is None:
        raise ValueError(f"Could not parse a stop index out of image name: {name}")
    return int(match.group(1))


@dataclass
class OrbitFit:
    """Camera azimuth as a linear function of stop index: one full turn across all stops."""

    n_total_stops: int
    phase_deg: float
    direction: float

    def azimuth_deg(self, index: int) -> float:
        return self.phase_deg + self.direction * index * 360.0 / self.n_total_stops


def fit_orbit_from_registered(
    recon: pycolmap.Reconstruction, real_rel_names: list[str], n_total_stops: int,
) -> OrbitFit | None:
    """Azimuth phase and turning direction from the registered real cameras: for each
    direction, the circular mean of azimuth - direction x stop x step; the direction with
    the tighter residuals wins. None with fewer than 2 registered cameras."""
    registered_centers = {
        img.name: np.array(img.projection_center())
        for img in recon.images.values()
        if img.has_pose and img.name in real_rel_names
    }
    if len(registered_centers) < 2:
        return None

    step_deg = 360.0 / n_total_stops
    indices = {name: _stop_index_from_name(name) for name in registered_centers}
    azimuths = {name: np.degrees(np.arctan2(c[1], c[0])) for name, c in registered_centers.items()}

    best = None
    for direction in (1.0, -1.0):
        residuals_deg = np.array([azimuths[n] - direction * indices[n] * step_deg for n in registered_centers])
        residuals_rad = np.radians(residuals_deg)
        phase_deg = float(np.degrees(np.arctan2(np.mean(np.sin(residuals_rad)), np.mean(np.cos(residuals_rad)))))
        spread = float(np.mean(np.abs((residuals_deg - phase_deg + 180.0) % 360.0 - 180.0)))
        if best is None or spread < best[0]:
            best = (spread, direction, phase_deg)

    _spread, direction, phase_deg = best
    return OrbitFit(n_total_stops=n_total_stops, phase_deg=phase_deg, direction=direction)


def _circular_distance(a: int, b: int, n: int) -> int:
    d = abs(a - b) % n
    return min(d, n - d)


def estimate_turntable_axis(world_from_cam_rotations: list[np.ndarray]) -> np.ndarray:
    """Unit turntable axis (world) from the registered cameras' camera-to-world rotations:
    every photo is the same camera turned about that axis, so each pair's relative
    rotation is about it. Angle-weighted average of the pairs' axes (signs aligned to +z);
    +z with fewer than 2 usable rotations."""
    z = np.array([0.0, 0.0, 1.0])
    if len(world_from_cam_rotations) < 2:
        return z
    acc = np.zeros(3)
    for i in range(len(world_from_cam_rotations)):
        for j in range(i + 1, len(world_from_cam_rotations)):
            rotvec = Rotation.from_matrix(world_from_cam_rotations[j] @ world_from_cam_rotations[i].T).as_rotvec()
            angle = np.linalg.norm(rotvec)
            if angle < np.radians(5.0):
                continue
            axis = rotvec / angle
            acc += (axis if axis @ z >= 0 else -axis) * min(angle, np.pi - angle)
    norm = np.linalg.norm(acc)
    return acc / norm if norm > 1e-9 else z


def estimate_turntable_axis_point(camera_centers: np.ndarray, axis: np.ndarray) -> np.ndarray:
    """A point on the turntable axis: the least-squares circle centre of the camera centres
    projected onto the plane perpendicular to `axis` (origin with fewer than 3 centres)."""
    if len(camera_centers) < 3:
        return np.zeros(3)
    u = np.cross(axis, [1.0, 0.0, 0.0] if abs(axis[0]) < 0.9 else [0.0, 1.0, 0.0])
    u /= np.linalg.norm(u)
    v = np.cross(axis, u)
    x, y = camera_centers @ u, camera_centers @ v
    design = np.column_stack([x, y, np.ones_like(x)])
    (a, b, _c), *_ = np.linalg.lstsq(design, x * x + y * y, rcond=None)
    return (a / 2.0) * u + (b / 2.0) * v


def register_missing_real_cameras_from_orbit(
    recon: pycolmap.Reconstruction, db_path: Path, real_rel_names: list[str],
    n_total_stops: int | None = None,
) -> list[str]:
    """Pose every photo COLMAP couldn't register: its nearest registered neighbour on each
    side (by stop index) rotated rigidly about the turntable axis by the orbit's azimuth
    difference, averaged by closeness (`_pose_from_neighbours`); intrinsics from the
    nearest neighbour. Leave-one-out error ~0.3-0.4deg / 2-3mm. Mutates `recon`; returns
    the added names (none if fewer than 2 photos registered).

    `n_total_stops`: stops in one full turn; by default the larger of the photo count and
    the span of stop indices (so a missing photo doesn't change the step angle). Raises
    ValueError if a name carries no `stop_<NN>` index."""
    registered_real = {
        img.name: img for img in recon.images.values() if img.has_pose and img.name in real_rel_names
    }
    missing_names = [name for name in real_rel_names if name not in registered_real]
    if not missing_names:
        return []

    if n_total_stops is None:
        all_indices = [_stop_index_from_name(name) for name in real_rel_names]
        n_total_stops = max(len(real_rel_names), max(all_indices) - min(all_indices) + 1)
    orbit = fit_orbit_from_registered(recon, real_rel_names, n_total_stops)
    if orbit is None:
        return []

    registered_indices = {name: _stop_index_from_name(name) for name in registered_real}
    world_from_cam = {name: img.cam_from_world().rotation.matrix().T for name, img in registered_real.items()}
    centers = {name: np.array(img.projection_center()) for name, img in registered_real.items()}
    axis = estimate_turntable_axis(list(world_from_cam.values()))
    axis_point = estimate_turntable_axis_point(np.array(list(centers.values())), axis)

    db = pycolmap.Database.open(str(db_path))
    added = []
    for name in missing_names:
        index = _stop_index_from_name(name)
        nearest_name = min(registered_indices,
                           key=lambda other: _circular_distance(index, registered_indices[other], n_total_stops))
        camera_id = registered_real[nearest_name].camera_id

        R_world_from_cam, center = _pose_from_neighbours(
            index, orbit, axis, axis_point, registered_indices, world_from_cam, centers, n_total_stops,
        )
        R = R_world_from_cam.T
        t = -R @ center

        db_image = db.read_image_with_name(name)
        image = pycolmap.Image(image_id=db_image.image_id, name=name, camera_id=camera_id)
        recon.add_image_with_trivial_frame(image, pycolmap.Rigid3d(pycolmap.Rotation3d(R), t))
        added.append(name)
    return added


def _pose_from_neighbours(
    index: int, orbit: OrbitFit, axis: np.ndarray, axis_point: np.ndarray,
    registered_indices: dict[str, int], world_from_cam: dict[str, np.ndarray],
    centers: dict[str, np.ndarray], n_total_stops: int,
) -> tuple[np.ndarray, np.ndarray]:
    """(world_from_cam rotation, camera centre) for stop `index`; see
    `register_missing_real_cameras_from_orbit`. Weights are 1 / stop distance."""
    def signed_offset(other_index):
        d = (index - other_index) % n_total_stops
        return d if d <= n_total_stops // 2 else d - n_total_stops

    offsets = {name: signed_offset(i) for name, i in registered_indices.items()}
    picks = []
    for side in (1, -1):
        same_side = [n for n, d in offsets.items() if d * side > 0]
        if same_side:
            picks.append(min(same_side, key=lambda n: abs(offsets[n])))
    if not picks:
        picks = [min(offsets, key=lambda n: abs(offsets[n]))]

    rotations, positions, weights = [], [], []
    target_az = orbit.azimuth_deg(index)
    for name in picks:
        delta = np.radians(target_az - orbit.azimuth_deg(registered_indices[name]))
        turn = Rotation.from_rotvec(axis * delta)
        rotations.append((turn * Rotation.from_matrix(world_from_cam[name])).as_quat())
        positions.append(turn.apply(centers[name] - axis_point) + axis_point)
        weights.append(1.0 / max(1, abs(offsets[name])))
    weights = np.array(weights)
    R = Rotation.from_quat(np.array(rotations)).mean(weights=weights).as_matrix()
    return R, np.average(np.array(positions), axis=0, weights=weights)


# --- results --------------------------------------------------------------------------


def strip_virtual_images(recon: pycolmap.Reconstruction, virtual_rel_names: set[str]) -> pycolmap.Reconstruction:
    """Copy of `recon` with the virtual images deregistered and their camera/rig removed."""
    pruned = copy.deepcopy(recon)
    virtual_frame_ids = [img.frame_id for img in pruned.images.values() if img.name in virtual_rel_names]
    virtual_camera_ids = {img.camera_id for img in pruned.images.values() if img.name in virtual_rel_names}
    for frame_id in virtual_frame_ids:
        pruned.deregister_frame(frame_id)
    for camera_id in virtual_camera_ids:
        del pruned.rigs[camera_id]
        del pruned.cameras[camera_id]
    return pruned


def rescale_cameras(recon: pycolmap.Reconstruction, new_width: int, new_height: int) -> None:
    """Rescale every camera (focal length, principal point, size) and its images' 2D points
    to `new_width` x `new_height` -- e.g. to run dense stereo on photos resized to another
    long edge than calibration chose. Poses are unchanged. Mutates `recon`."""
    for camera_id in list(recon.cameras):
        camera = recon.camera(camera_id)
        scale = np.array([new_width / camera.width, new_height / camera.height])
        camera.rescale(new_width, new_height)
        for img in recon.images.values():
            if img.camera_id == camera_id:
                for p2d in img.points2D:
                    p2d.xy = p2d.xy * scale


def registration_summary(recon: pycolmap.Reconstruction, virtual_rel_names: set[str]) -> dict:
    registered = [img for img in recon.images.values() if img.has_pose]
    n_virtual = sum(img.name in virtual_rel_names for img in registered)
    return {
        "num_registered_virtual": n_virtual,
        "num_registered_real": len(registered) - n_virtual,
        "num_points3D": recon.num_points3D(),
        "mean_track_length": recon.compute_mean_track_length(),
        "mean_reprojection_error": recon.compute_mean_reprojection_error(),
    }


def orbit_height_spread_mm(recon: pycolmap.Reconstruction, real_rel_names: list[str]) -> float:
    """10th-90th percentile range of the registered real cameras' heights above the plate
    (world z), mm. One camera on a turntable gives a horizontal circle (6-17mm on good
    captures); a wrong solution can register most photos at low reprojection error yet
    spread them over hundreds of mm. 0.0 with fewer than 3 cameras."""
    names = set(real_rel_names)
    z = [float(np.asarray(img.projection_center())[2]) for img in recon.images.values()
         if img.has_pose and img.name in names]
    if len(z) < 3:
        return 0.0
    p10, p90 = np.percentile(z, [10, 90])
    return float(p90 - p10)


# --- one attempt, and the search over resolutions ---------------------------------------


@dataclass
class CalibrationAttempt:
    """What later steps need from one `run_calibration_attempt` pass."""

    max_long_edge: int | None
    real_paths: list[Path]
    real_rel_names: list[str]
    virtual_rel_names: list[str]
    virtual_views: list
    virtual_K: np.ndarray
    out_w: int
    out_h: int
    recon: pycolmap.Reconstruction
    plate_points: np.ndarray
    removed_far: list[str]
    removed_dup: list[str]
    num_registered_real: int
    real_focal_prior_px: float | None = None
    real_focal_prior_source: str = "none"


def run_calibration_attempt(
    real_srcs: list[Path],
    work_dir: Path,
    set_name: str,
    plate_image: np.ndarray,
    plate_size_mm: float,
    n_virtual_views: int,
    virtual_elevation_deg: float,
    virtual_distance_mm: float,
    plate_fill_fraction: float,
    max_long_edge: int | None,
    db_path: Path,
    seed_triangulated_dir: Path,
    reconstruction_dir: Path,
    max_num_features: int = 4096,
    abs_pose_min_num_inliers: int = 15,
    abs_pose_min_inlier_ratio: float = 0.1,
    max_reg_trials: int = 1,
    max_runtime_seconds: int = 300,
    max_distance_ratio: float = 3.0,
    min_camera_separation_mm: float | None = None,
    random_seed: int = 0,
    num_threads: int = 1,
    cpu_brute_force_matcher: bool = False,
) -> CalibrationAttempt:
    """Steps 1-5 of the module docstring at one real-photo resolution, then the
    implausible/duplicate pose filters."""
    real_paths = resize_photographs(real_srcs, work_dir / set_name, max_long_edge)
    real_rel_names = [f"{set_name}/{p.name}" for p in real_paths]
    out_h, out_w = cv2.imread(str(real_paths[0])).shape[:2]

    virtual_views = plate_geometry.generate_virtual_views(
        n_views=n_virtual_views, elevation_deg=virtual_elevation_deg, distance_mm=virtual_distance_mm,
    )
    focal_px = plate_geometry.focal_length_for_fill(
        plate_size_mm, virtual_distance_mm, out_w, out_h, fill_fraction=plate_fill_fraction,
    )
    virtual_K = plate_geometry.build_intrinsics(focal_px, out_w, out_h)

    (work_dir / "calibration_pattern").mkdir(parents=True, exist_ok=True)
    virtual_rel_names = []
    for view in virtual_views:
        rendered, _corners = plate_geometry.render_plate_photograph(
            plate_image, plate_size_mm, virtual_K, view.R, view.t, (out_w, out_h),
        )
        rel_name = f"calibration_pattern/{view.name}.png"
        cv2.imwrite(str(work_dir / rel_name), rendered)
        virtual_rel_names.append(rel_name)
    virtual_names_set = set(virtual_rel_names)

    real_focal_length_px, focal_source = estimate_focal_length_from_plate(real_paths, plate_image), "plate"
    if real_focal_length_px is None:
        real_focal_length_px, focal_source = estimate_focal_length_px(real_srcs[0], out_w), "exif"
    if real_focal_length_px is None:
        focal_source = "none"

    build_database(
        db_path=db_path, work_dir=work_dir,
        virtual_rel_names=virtual_rel_names, real_rel_names=real_rel_names, virtual_K=virtual_K,
        real_focal_length_px=real_focal_length_px, real_image_size=(out_w, out_h),
        max_num_features=max_num_features, random_seed=random_seed, num_threads=num_threads,
        cpu_brute_force_matcher=cpu_brute_force_matcher,
    )

    seed = build_seed_reconstruction(db_path, virtual_views, virtual_rel_names)
    virtual_camera_id = seed.image(seed.reg_image_ids()[0]).camera_id
    seed = triangulate_seed(
        seed, db_path, work_dir, seed_triangulated_dir, random_seed=random_seed, num_threads=num_threads,
    )
    # the plate's own points, triangulated from virtual views only -- untouched by real-photo
    # pose error; used for leveling (pipeline._level_point_cloud)
    plate_points = np.array([p.xyz for p in seed.points3D.values()])

    reconstructions = register_real_cameras(
        db_path, work_dir, seed_triangulated_dir, reconstruction_dir, virtual_camera_id,
        abs_pose_min_num_inliers=abs_pose_min_num_inliers, abs_pose_min_inlier_ratio=abs_pose_min_inlier_ratio,
        max_reg_trials=max_reg_trials, max_runtime_seconds=max_runtime_seconds,
        random_seed=random_seed, num_threads=num_threads,
    )
    if not reconstructions:
        recon = seed
        removed_far, removed_dup = [], []
    else:
        recon = max(reconstructions.values(), key=lambda r: r.num_reg_images())
        removed_far = filter_implausible_real_cameras(recon, virtual_names_set, virtual_distance_mm, max_distance_ratio)
        removed_dup = filter_degenerate_real_cameras(recon, virtual_names_set, min_camera_separation_mm)

    num_registered_real = sum(1 for img in recon.images.values() if img.has_pose and img.name not in virtual_names_set)

    return CalibrationAttempt(
        max_long_edge=max_long_edge, real_paths=real_paths, real_rel_names=real_rel_names,
        virtual_rel_names=virtual_rel_names, virtual_views=virtual_views, virtual_K=virtual_K,
        out_w=out_w, out_h=out_h, recon=recon, plate_points=plate_points,
        removed_far=removed_far, removed_dup=removed_dup, num_registered_real=num_registered_real,
        real_focal_prior_px=real_focal_length_px, real_focal_prior_source=focal_source,
    )


def search_calib_max_long_edge(
    real_srcs: list[Path],
    work_dir: Path,
    set_name: str,
    plate_image: np.ndarray,
    plate_size_mm: float,
    n_virtual_views: int,
    virtual_elevation_deg: float,
    virtual_distance_mm: float,
    plate_fill_fraction: float,
    long_edge_candidates: list[int],
    min_registered_real: int,
    db_path: Path,
    seed_triangulated_dir: Path,
    reconstruction_dir: Path,
    max_num_features: int = 4096,
    abs_pose_min_num_inliers: int = 15,
    abs_pose_min_inlier_ratio: float = 0.1,
    max_reg_trials: int = 1,
    max_runtime_seconds: int = 300,
    max_distance_ratio: float = 3.0,
    min_camera_separation_mm: float | None = None,
    random_seed: int = 0,
    num_threads: int = 1,
    enable_brute_force_fallback: bool = True,
    max_orbit_height_spread_mm: float | None = 40.0,
    progress_cb=None,
) -> CalibrationAttempt:
    """`run_calibration_attempt` at each long edge (smallest first), returning the first
    that registers more than `min_registered_real` photos.

    * An attempt whose cameras don't lie on a turntable orbit (`orbit_height_spread_mm` >
      `max_orbit_height_spread_mm`; None disables) is rejected however many it registered.
    * If none clears the bar, the best one is kept (re-run if needed, so the on-disk
      database matches it).
    * If every attempt registers nothing and `enable_brute_force_fallback`, the sweep is
      repeated with exact matching (~30-40x slower matching).
    * If every attempt is rejected, raises RuntimeError.

    `progress_cb(message)` is called once per attempt; defaults to `print`."""
    notify = progress_cb or print
    best = None
    best_brute_force = False
    last_candidate = None
    last_brute_force = False

    def attempt_at(candidate, brute_force):
        return run_calibration_attempt(
            real_srcs, work_dir, set_name, plate_image, plate_size_mm,
            n_virtual_views, virtual_elevation_deg, virtual_distance_mm, plate_fill_fraction,
            candidate, db_path, seed_triangulated_dir, reconstruction_dir,
            max_num_features=max_num_features,
            abs_pose_min_num_inliers=abs_pose_min_num_inliers, abs_pose_min_inlier_ratio=abs_pose_min_inlier_ratio,
            max_reg_trials=max_reg_trials, max_runtime_seconds=max_runtime_seconds,
            max_distance_ratio=max_distance_ratio, min_camera_separation_mm=min_camera_separation_mm,
            random_seed=random_seed, num_threads=num_threads, cpu_brute_force_matcher=brute_force,
        )

    for cpu_brute_force_matcher in ((False, True) if enable_brute_force_fallback else (False,)):
        if cpu_brute_force_matcher:
            if best is not None and best.num_registered_real > 0:
                break
            notify("No usable registration with approximate matching -- retrying every long edge with exact "
                   "(brute-force) matching; much slower")

        for candidate in long_edge_candidates:
            attempt = attempt_at(candidate, cpu_brute_force_matcher)
            last_candidate, last_brute_force = candidate, cpu_brute_force_matcher
            matcher_note = " (exact CPU matching)" if cpu_brute_force_matcher else ""
            spread = orbit_height_spread_mm(attempt.recon, attempt.real_rel_names)
            notify(
                f"max_long_edge={candidate}{matcher_note}: {attempt.num_registered_real} real camera(s) "
                f"registered ({len(attempt.removed_far)} discarded far, {len(attempt.removed_dup)} discarded duplicate), "
                f"camera height spread {spread:.0f}mm, initial focal length "
                + (f"{attempt.real_focal_prior_px:.0f}px ({attempt.real_focal_prior_source})"
                   if attempt.real_focal_prior_px else "COLMAP default")
            )
            if max_orbit_height_spread_mm is not None and spread > max_orbit_height_spread_mm:
                notify(f"max_long_edge={candidate}{matcher_note} rejected: the cameras' heights above the plate "
                       f"spread {spread:.0f}mm (> {max_orbit_height_spread_mm:g}mm) -- not a turntable orbit")
                continue
            if best is None or attempt.num_registered_real > best.num_registered_real:
                best, best_brute_force = attempt, cpu_brute_force_matcher
            if attempt.num_registered_real > min_registered_real:
                notify(f"max_long_edge={candidate}{matcher_note} cleared the {min_registered_real}-camera bar -- stopping search")
                return attempt

    if best is None:
        raise RuntimeError(
            f"Every candidate max_long_edge gave camera poses that don't lie on a turntable orbit "
            f"(height spread > {max_orbit_height_spread_mm:g}mm) -- check the plate is flat, fully visible "
            f"and evenly lit in the photos, or raise calib_max_orbit_height_spread_mm."
        )
    if (best.max_long_edge, best_brute_force) != (last_candidate, last_brute_force):
        notify(f"No candidate exceeded {min_registered_real} registered real cameras -- re-running the best "
               f"(max_long_edge={best.max_long_edge}, {best.num_registered_real} registered)")
        best = attempt_at(best.max_long_edge, best_brute_force)
    else:
        notify(f"No candidate exceeded {min_registered_real} registered real cameras -- keeping the best "
               f"(max_long_edge={best.max_long_edge}, {best.num_registered_real} registered)")
    return best
