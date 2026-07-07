"""Dense point-cloud reconstruction from the calibrated sparse model (Step 4's output).

Pipeline:
  1. Load the calibrated real-camera reconstruction (poses + intrinsics, no virtual
     cameras) written by camera_geometry_estimation.ipynb to outputs/<set>/final, and
     reuse the same downscaled real photographs Step 4 already calibrated against
     (outputs/<set>/work) - no separate copy/downscale/rescale needed, since that
     resolution choice (tractable PatchMatchStereo runtime vs. achievable point
     density - full 4000x3000 photos measured ~60-90s/image on this GPU) already
     lives in one place, Step 4's own MAX_LONG_EDGE.
  2. Undistort those working-resolution images against the calibrated model (COLMAP's
     dense stereo pipeline works on an undistorted/pinhole workspace, not the
     original possibly-distorted photos).
  3. Run PatchMatchStereo (COLMAP's MVS algorithm; needs a CUDA GPU) for per-image
     depth and normal maps.
  4. Run StereoFusion to merge depth maps into a single dense, coloured point cloud.
  5. Restrict to a region of interest around the artefact (a rough spatial filter -
     not the paper's dedicated cropping stage, which comes later) and downsample to
     hit target point densities (points per mm^2 of the artefact's bounding-box
     surface area, used as a simple, mesh-free proxy for surface area).

`rescale_reconstruction` is kept as a utility for anyone who wants dense stereo at a
*different* resolution than Step 4's calibration one, but the default pipeline above
doesn't need it - reusing Step 4's own working images is resolution-consistent by
construction.
"""

import copy
import shutil
from pathlib import Path

import numpy as np
import pycolmap


def load_calibrated_reconstruction(final_dir: Path) -> pycolmap.Reconstruction:
    recon = pycolmap.Reconstruction()
    recon.read(str(final_dir))
    return recon


def rescale_reconstruction(
    recon: pycolmap.Reconstruction, target_width: int, target_height: int
) -> pycolmap.Reconstruction:
    """A copy of `recon` with every camera's intrinsics rescaled to
    (target_width, target_height). Poses carry over unchanged - only focal length and
    principal point (and the camera's own width/height) depend on resolution.
    """
    rescaled = copy.deepcopy(recon)
    for camera in rescaled.cameras.values():
        camera.rescale(target_width, target_height)
    return rescaled


def undistort_for_stereo(
    recon: pycolmap.Reconstruction,
    sparse_dir: Path,
    image_path: Path,
    workspace_dir: Path,
    max_image_size: int = -1,
) -> list[str]:
    """Write `recon` to `sparse_dir`, then undistort its images into a COLMAP dense
    workspace at `workspace_dir` (images/ + sparse/ + stereo/), ready for
    PatchMatchStereo. Returns the image names undistorted (in the same order COLMAP
    will process them).
    """
    if sparse_dir.exists():
        shutil.rmtree(sparse_dir)
    sparse_dir.mkdir(parents=True)
    recon.write(sparse_dir)

    if workspace_dir.exists():
        shutil.rmtree(workspace_dir)

    image_names = [img.name for img in recon.images.values() if img.has_pose]

    undistort_opts = pycolmap.UndistortCameraOptions()
    undistort_opts.max_image_size = max_image_size

    pycolmap.undistort_images(
        output_path=workspace_dir,
        input_path=sparse_dir,
        image_path=image_path,
        image_names=image_names,
        output_type="COLMAP",
        undistort_options=undistort_opts,
    )
    return image_names


def run_patch_match_stereo(
    workspace_dir: Path,
    max_image_size: int = -1,
    window_radius: int = 5,
    num_iterations: int = 5,
    geom_consistency: bool = True,
    num_samples: int = 15,
) -> None:
    """Per-image depth/normal maps via COLMAP's PatchMatchStereo. Requires a CUDA GPU -
    there is no CPU fallback.
    """
    options = pycolmap.PatchMatchOptions()
    options.max_image_size = max_image_size
    options.window_radius = window_radius
    options.num_iterations = num_iterations
    options.geom_consistency = geom_consistency
    options.num_samples = num_samples
    pycolmap.patch_match_stereo(workspace_path=workspace_dir, options=options)


def run_stereo_fusion(
    workspace_dir: Path,
    output_path: Path,
    min_num_pixels: int = 5,
    max_reproj_error: float = 2.0,
    max_depth_error: float = 0.01,
) -> pycolmap.Reconstruction:
    """Fuse PatchMatchStereo's depth maps into one dense, coloured point cloud.

    `output_path` must be (and is created as) a directory - pycolmap writes a
    small COLMAP-format reconstruction there (cameras/images/points3D.bin) holding
    the fused points, which is also what's returned.
    """
    output_path.mkdir(parents=True, exist_ok=True)
    options = pycolmap.StereoFusionOptions()
    options.min_num_pixels = min_num_pixels
    options.max_reproj_error = max_reproj_error
    options.max_depth_error = max_depth_error
    return pycolmap.stereo_fusion(
        output_path=str(output_path),
        workspace_path=str(workspace_dir),
        input_type="geometric",
        options=options,
    )


def read_depth_map(path: Path) -> np.ndarray:
    """Read a COLMAP PatchMatchStereo depth map (.geometric.bin/.photometric.bin):
    an ASCII "width&height&channels&" header followed by raw little-endian float32
    data, row-major, single channel.
    """
    with open(path, "rb") as f:
        header = b""
        while header.count(b"&") < 3:
            header += f.read(1)
        width, height, channels = map(int, header.decode("ascii").rstrip("&").split("&"))
        data = np.fromfile(f, dtype="<f4", count=width * height * channels)
    return data.reshape(height, width, channels).squeeze(axis=-1)


def points_and_colors(recon: pycolmap.Reconstruction) -> tuple[np.ndarray, np.ndarray]:
    points = np.array([p.xyz for p in recon.points3D.values()])
    colors = np.array([p.color for p in recon.points3D.values()])
    return points, colors


def filter_by_radius(
    points: np.ndarray,
    colors: np.ndarray,
    radius_mm: float,
    center: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> tuple[np.ndarray, np.ndarray]:
    """Rough region-of-interest filter: keep only points within `radius_mm` of `center`
    (the world origin, where the calibration plate is centred, by default). This is a
    coarse spatial cut to exclude background/table clutter picked up by dense
    matching - not the paper's dedicated cropping stage (a later step), which would
    do this more carefully.
    """
    center_arr = np.array(center)
    distances = np.linalg.norm(points - center_arr, axis=1)
    mask = distances < radius_mm
    return points[mask], colors[mask]


def bounding_box_surface_area_mm2(points: np.ndarray) -> float:
    """Surface area (mm^2) of the points' axis-aligned bounding box, as a simple,
    mesh-free proxy for the artefact's surface area when normalising point density.
    """
    mins, maxs = points.min(axis=0), points.max(axis=0)
    w, h, d = maxs - mins
    return float(2 * (w * h + w * d + h * d))


def voxel_downsample(
    points: np.ndarray, colors: np.ndarray, voxel_size: float
) -> tuple[np.ndarray, np.ndarray]:
    """Grid-based downsampling: average the points (and colors) falling in each
    occupied voxel of the given size into one representative point.
    """
    if voxel_size <= 0 or len(points) == 0:
        return points, colors

    keys = np.floor(points / voxel_size).astype(np.int64)
    order = np.lexsort(keys.T[::-1])
    keys_sorted = keys[order]
    points_sorted = points[order]
    colors_sorted = colors[order]

    is_new_group = np.empty(len(keys_sorted), dtype=bool)
    is_new_group[0] = True
    is_new_group[1:] = np.any(keys_sorted[1:] != keys_sorted[:-1], axis=1)
    group_ids = np.cumsum(is_new_group) - 1
    n_groups = group_ids[-1] + 1

    sums_points = np.zeros((n_groups, 3))
    sums_colors = np.zeros((n_groups, 3))
    counts = np.zeros(n_groups)
    np.add.at(sums_points, group_ids, points_sorted)
    np.add.at(sums_colors, group_ids, colors_sorted)
    np.add.at(counts, group_ids, 1)

    return sums_points / counts[:, None], sums_colors / counts[:, None]


def downsample_to_density(
    points: np.ndarray,
    colors: np.ndarray,
    target_density_per_mm2: float,
    surface_area_mm2: float,
    num_search_steps: int = 40,
) -> tuple[np.ndarray, np.ndarray, float, bool]:
    """Voxel-downsample (points, colors) to approximately `target_density_per_mm2`
    points per mm^2 of `surface_area_mm2`, via binary search over the voxel size.

    Returns (points, colors, achieved_density_per_mm2, reached_target). If the raw
    point cloud is already sparser than the target, it's returned unchanged with
    reached_target=False - no amount of downsampling can *increase* density.
    """
    target_count = max(int(round(target_density_per_mm2 * surface_area_mm2)), 1)
    if len(points) <= target_count:
        return points, colors, len(points) / surface_area_mm2, False

    lo, hi = 1e-5, float(np.max(points.max(axis=0) - points.min(axis=0))) or 1.0
    for _ in range(num_search_steps):
        mid = (lo + hi) / 2
        candidate_points, _ = voxel_downsample(points, colors, mid)
        if len(candidate_points) > target_count:
            lo = mid
        else:
            hi = mid

    out_points, out_colors = voxel_downsample(points, colors, hi)
    return out_points, out_colors, len(out_points) / surface_area_mm2, True


def write_ply(path: Path, points: np.ndarray, colors: np.ndarray | None = None) -> None:
    """Minimal binary-little-endian PLY writer (vertices only, optional RGB)."""
    n = len(points)
    if colors is not None:
        dtype = [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("red", "u1"), ("green", "u1"), ("blue", "u1")]
    else:
        dtype = [("x", "<f4"), ("y", "<f4"), ("z", "<f4")]

    vertices = np.zeros(n, dtype=dtype)
    vertices["x"], vertices["y"], vertices["z"] = points[:, 0], points[:, 1], points[:, 2]
    if colors is not None:
        clipped = np.clip(colors, 0, 255).astype(np.uint8)
        vertices["red"], vertices["green"], vertices["blue"] = clipped[:, 0], clipped[:, 1], clipped[:, 2]

    header_lines = [
        "ply", "format binary_little_endian 1.0", f"element vertex {n}",
        "property float x", "property float y", "property float z",
    ]
    if colors is not None:
        header_lines += ["property uchar red", "property uchar green", "property uchar blue"]
    header_lines.append("end_header")
    header = ("\n".join(header_lines) + "\n").encode("ascii")

    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "wb") as f:
        f.write(header)
        f.write(vertices.tobytes())
