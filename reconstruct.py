"""End-to-end reconstruction pipeline: every stage from the paper's Section 2.4 in one script,
iterating automatically over every acquisition-set folder under `images/`.

Consolidates what `camera_geometry_estimation.ipynb`, `dense_reconstruction.ipynb`,
`cropping.ipynb`, `point_cloud_registration.ipynb`, `meshing.ipynb`, and `texturing.ipynb`
each do into a single, non-interactive run:

  1. **Camera geometry estimation** (Section 2.4.1) - per set: render the virtual
     calibration sequence, extract/match SIFT features, triangulate the calibration model from
     the virtual cameras alone, then register the real photographs against it.
  2. **Dense point-cloud reconstruction** (Section 2.4.2) - per set: PatchMatchStereo +
     StereoFusion (COLMAP's dense pipeline, substituting for the paper's CMVS/PMVS - see
     `utils/dense_reconstruction.py`). **Requires a CUDA GPU.**
  3. **Cropping** (Section 2.4.3) - per set: correct Step 1's known z-axis mirroring, then crop
     to the calibration pattern's footprint and a luminosity-derived z range.
  4. **Point-cloud registration** (Section 2.4.4) - coarse-align + Weighted-ICP + confidence-
     weighted merge every set's cropped cloud into one common frame.
  5. **Meshing** (Section 2.4.5) - Screened Poisson surface reconstruction on the merged cloud.
  6. **Texturing** (Section 2.4.6) - reposition every set's real cameras into the mesh's common
     frame and paint each vertex with the occlusion-aware, multi-view-blended photo colour (see
     `utils/texturing.py` for why this is vertex colouring, not a MeshLab UV atlas).

Differences from the notebooks, by design:

- **Sets are discovered automatically** (`discover_sets`) from `images/`'s subfolders, rather
  than a hardcoded `SET_NAME` edited per run.
- **No plots.** The notebooks' `matplotlib`/`open3d` visualisations exist for inspecting a
  single run interactively; this script only logs text - including the same "possible problem"
  diagnostics the notebooks show visually (e.g. a low camera-registration rate, a low coarse-
  alignment score, low vertex-photo coverage), as `logging.warning` calls instead of a chart to
  eyeball.
- **Almost everything stays in memory between stages** instead of round-tripping through disk
  the way separate notebooks necessarily do - cropped points, the merged cloud, the mesh, and
  the per-set registration transforms are all just passed as Python objects from one stage
  function to the next. Only what COLMAP's own file-based API requires (its database, and
  several intermediate reconstruction/dense-workspace directories) still touches disk.
- **Downscaling is optional** (`--no-downscale`, or `--max-long-edge` to change the notebooks'
  default of 2000px).
- **Cleanup always runs once texturing finishes successfully** - by default it leaves only
  `outputs/mesh/mesh.ply` and `outputs/textured/textured.ply`. `--keep-intermediates` widens
  what survives cleanup to one plain `.ply` per major stage instead - `<set>/sparse.ply` (Step
  4), `<set>/dense.ply` (Step 5), `<set>/cropped/cropped.ply` (Step 6), `merged/merged.ply`
  (Step 7), plus the same mesh/textured outputs - written specifically for this flag, since the
  pipeline itself never puts these on disk otherwise. Either way, COLMAP's own databases and
  dense workspaces are never kept - `--keep-intermediates` is for inspecting the point-cloud
  pipeline stage by stage, not for keeping every byte a run touched.
- **One set failing doesn't abort the run.** Camera geometry, dense reconstruction, and cropping
  run per set inside a `try`/`except`; a set that raises is logged and skipped, and the run
  continues with whatever sets remain (registration/meshing/texturing need at least one).

Usage: `python reconstruct.py` (from the repo root, with the same `.venv` the notebooks use).
See `python reconstruct.py --help` for the available overrides.
"""

from __future__ import annotations

import argparse
import logging
import shutil
import subprocess
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d
import pycolmap
from scipy.spatial import cKDTree

from utils import colmap_calibration
from utils import cropping as cr
from utils import dense_reconstruction as dr
from utils import generate_pattern
from utils import meshing as mesh
from utils import plate_geometry
from utils import registration as reg
from utils import texturing as tex

log = logging.getLogger("reconstruct")


@dataclass
class Config:
    images_dir: Path
    output_dir: Path

    random_seed: int = 0
    num_threads: int = 1

    # Calibration plate (utils/generate_pattern.py).
    plate_size_mm: float = 130.0
    plate_dpi: int = 300
    plate_grid_cells: int = 52
    plate_seed: int = 42
    plate_border_mm: float = 5.0

    # Real photographs: downscaled for tractable SIFT/dense-stereo runtime. None = native
    # resolution.
    max_long_edge: int | None = 3000

    # Virtual calibration sequence.
    n_virtual_views: int = 24
    virtual_elevation_deg: float = 45.0
    virtual_distance_mm: float = 250.0
    plate_fill_fraction: float = 1.4

    # SIFT / matching.
    max_num_features: int = 4096

    # Absolute-pose RANSAC (real cameras vs. the virtual-only calibration model).
    abs_pose_min_num_inliers: int = 15
    abs_pose_min_inlier_ratio: float = 0.1
    max_reg_trials: int = 1
    max_runtime_seconds: int = 300

    # Post-registration sanity filters.
    max_distance_ratio: float = 3.0
    # None = adaptive (a fraction of the median inter-camera spacing actually observed for this
    # set, rather than a fixed mm value that implicitly assumes a specific rig radius - see
    # colmap_calibration.filter_degenerate_real_cameras's own docstring for the real diagnosis
    # this default replaced: a flat 20mm threshold discarded 32/36 entirely genuine, correctly-
    # ordered camera poses on a rig whose actual working radius put consecutive stops ~18mm apart).
    min_camera_separation_mm: float | None = None
    min_registered_fraction_warn: float = 0.5

    # Dense reconstruction (PatchMatchStereo / StereoFusion).
    patch_match_max_image_size: int = -1
    patch_match_window_radius: int = 5
    patch_match_num_iterations: int = 5
    fusion_min_num_pixels: int = 5
    fusion_max_reproj_error: float = 2.0
    fusion_max_depth_error: float = 0.01
    roi_radius_mm: float = 65.0
    density_targets_per_mm2: tuple[float, ...] = (100.0, 300.0)
    min_fused_points_warn: int = 500

    # Cropping.
    pattern_size_mm: float = 130.0
    turntable_z_mm: float = 0.0
    z_start_offset_mm: float = 2.0
    z_slice_thickness_mm: float = 1.0
    luminosity_threshold: float = 100.0
    # Require the luminosity threshold to hold for this many consecutive slices before accepting
    # it as the real dark-to-light transition, not a single statistically-lucky slice - see
    # cropping.find_lower_z_limit's own docstring for the real case this fixed (a saturated-red
    # object with no dark support material almost got entirely cropped away by a one-slice fluke).
    min_sustained_slices: int = 3
    gap_thickness_mm: float = 5.0
    min_points_per_slice: int = 5
    min_cropped_points_warn: int = 200

    # Turntable-tilt plane fit (cropping.fit_dominant_plane/level_plane) - corrects for the real,
    # physically captured rig not being perfectly level, which the z-slice crop above otherwise
    # silently assumes. See fit_dominant_plane's own docstring for a real case this fixed.
    plane_fit_distance_threshold_mm: float = 1.0
    plane_fit_n_trials: int = 2000
    min_plane_inlier_fraction_warn: float = 0.5

    # Registration.
    reference_set: str | None = None  # None = first discovered set (sorted)
    normal_k: int = 16
    lambda_mm: float = 1.0
    coarse_n_search_points: int = 500
    coarse_n_trials: int = 500
    coarse_max_total_candidates: int = 6000
    coarse_distance_tolerance_mm: float = 1.0
    coarse_overlap_threshold_mm: float = 1.5
    coarse_min_base_spread_mm: float = 8.0
    coarse_n_score_points: int = 1500
    icp_max_iterations: int = 60
    icp_max_correspondence_distance_mm: float = 4.0
    merge_radius_mm: float = 1.0
    min_coarse_score_warn: float = 0.2

    # Isolated/floating-point removal (cropping.remove_isolated_points) - used both per-set
    # right after cropping (below) and again on the merged cloud during meshing, since
    # registration can introduce its own new isolated debris a per-set pass can't catch.
    isolation_radius_mm: float = 1.5
    min_component_size: int = 10

    # Meshing.
    poisson_depth: int = 14
    poisson_scale: float = 1.1
    poisson_linear_fit: bool = False
    isolated_fraction_warn: float = 0.10
    # Density-based trimming of Poisson's own low-confidence vertices (mesh.trim_low_density_vertices)
    # - removes the "blob" Poisson balloons into wherever the input cloud goes sparse (e.g. thin
    # structures), and the small disconnected shell fragments (mesh.remove_small_mesh_components)
    # that trimming leaves behind along the cut. See both functions' docstrings.
    poisson_density_trim_quantile: float = 0.02
    mesh_min_component_triangles: int = 100

    # Texturing.
    occlusion_eps_mm: float = 1.5
    min_facing: float = 0.1
    min_vertex_coverage_warn: float = 0.3
    camera_angle_warn_deg: float = 30.0


def discover_sets(images_dir: Path) -> list[str]:
    """Every subfolder of `images_dir` that contains at least one `.jpg`, sorted by name."""
    if not images_dir.is_dir():
        raise FileNotFoundError(f"images directory not found: {images_dir}")
    return sorted(p.name for p in images_dir.iterdir() if p.is_dir() and any(p.glob("*.jpg")))


def check_cuda_available() -> bool:
    """A best-effort check for an NVIDIA GPU (`nvidia-smi`) - dense reconstruction
    (PatchMatchStereo) needs CUDA and has no CPU fallback (see `utils/dense_reconstruction.py`),
    so this is worth flagging up front rather than failing deep into the first set.
    """
    try:
        return subprocess.run(["nvidia-smi"], capture_output=True, timeout=10).returncode == 0
    except (FileNotFoundError, subprocess.TimeoutExpired, OSError):
        return False


# --- Stage 1: camera geometry estimation (paper Section 2.4.1-2.4.3) -----------------------


def stage_camera_geometry(set_name: str, cfg: Config) -> dict:
    real_dir = cfg.images_dir / set_name
    out_dir = cfg.output_dir / set_name
    work_dir = out_dir / "work"
    db_path = out_dir / "colmap.db"
    seed_triangulated_dir = out_dir / "seed_triangulated"
    reconstruction_dir = out_dir / "reconstruction"
    full_triangulated_dir = out_dir / "full_triangulated"
    out_dir.mkdir(parents=True, exist_ok=True)

    plate_image, _ = generate_pattern.generate_pattern(
        cfg.plate_size_mm, cfg.plate_dpi, cfg.plate_grid_cells, cfg.plate_seed, cfg.plate_border_mm
    )
    plate_image = np.array(plate_image, dtype=np.uint8)

    real_srcs = sorted(real_dir.glob("*.jpg"))
    if not real_srcs:
        raise RuntimeError(f"no .jpg photographs found in {real_dir}")
    log.info("%s: %d real photographs", set_name, len(real_srcs))

    real_paths = colmap_calibration.resize_photographs(real_srcs, work_dir / set_name, cfg.max_long_edge)
    real_rel_names = [f"{set_name}/{p.name}" for p in real_paths]

    sample = cv2.imread(str(real_paths[0]))
    if sample is None:
        raise RuntimeError(f"could not read downscaled working photo {real_paths[0]}")
    out_h, out_w = sample.shape[:2]
    log.info(
        "%s: working resolution %dx%d (%s)", set_name, out_w, out_h,
        f"downscaled to max_long_edge={cfg.max_long_edge}" if cfg.max_long_edge else "native resolution",
    )

    virtual_views = plate_geometry.generate_virtual_views(
        n_views=cfg.n_virtual_views, elevation_deg=cfg.virtual_elevation_deg, distance_mm=cfg.virtual_distance_mm,
    )
    focal_px = plate_geometry.focal_length_for_fill(
        cfg.plate_size_mm, cfg.virtual_distance_mm, out_w, out_h, fill_fraction=cfg.plate_fill_fraction
    )
    virtual_K = plate_geometry.build_intrinsics(focal_px, out_w, out_h)

    virtual_dir = work_dir / "calibration_pattern"
    virtual_dir.mkdir(parents=True, exist_ok=True)
    virtual_rel_names = []
    for view in virtual_views:
        rendered, _ = plate_geometry.render_plate_photograph(
            plate_image, cfg.plate_size_mm, virtual_K, view.R, view.t, (out_w, out_h)
        )
        rel_name = f"calibration_pattern/{view.name}.png"
        cv2.imwrite(str(work_dir / rel_name), rendered)
        virtual_rel_names.append(rel_name)
    log.info("%s: rendered %d virtual calibration photographs", set_name, len(virtual_rel_names))

    real_focal_length_px = colmap_calibration.estimate_focal_length_px(real_srcs[0], out_w)
    if real_focal_length_px is None:
        log.warning(
            "%s: no usable EXIF focal length on %s - falling back to COLMAP's generic focal-length "
            "guess, which is more likely to make real-camera registration fail for this set",
            set_name, real_srcs[0].name,
        )

    colmap_calibration.build_database(
        db_path=db_path, work_dir=work_dir,
        virtual_rel_names=virtual_rel_names, real_rel_names=real_rel_names, virtual_K=virtual_K,
        real_focal_length_px=real_focal_length_px, real_image_size=(out_w, out_h),
        max_num_features=cfg.max_num_features, random_seed=cfg.random_seed, num_threads=cfg.num_threads,
    )

    seed = colmap_calibration.build_seed_reconstruction(db_path, virtual_views, virtual_rel_names)
    virtual_camera_id = seed.image(seed.reg_image_ids()[0]).camera_id
    triangulated = colmap_calibration.triangulate_seed(
        seed, db_path, work_dir, seed_triangulated_dir, random_seed=cfg.random_seed, num_threads=cfg.num_threads,
    )
    log.info("%s: triangulated %d calibration-plate points from the virtual cameras", set_name, triangulated.num_points3D())

    reconstructions = colmap_calibration.register_real_cameras(
        db_path, work_dir, seed_triangulated_dir, reconstruction_dir, virtual_camera_id,
        abs_pose_min_num_inliers=cfg.abs_pose_min_num_inliers,
        abs_pose_min_inlier_ratio=cfg.abs_pose_min_inlier_ratio,
        max_reg_trials=cfg.max_reg_trials, max_runtime_seconds=cfg.max_runtime_seconds,
        random_seed=cfg.random_seed, num_threads=cfg.num_threads,
    )
    if not reconstructions:
        raise RuntimeError("incremental mapping produced no reconstruction at all")
    recon = max(reconstructions.values(), key=lambda r: r.num_reg_images())

    removed_far = colmap_calibration.filter_implausible_real_cameras(
        recon, set(virtual_rel_names), expected_distance_mm=cfg.virtual_distance_mm,
        max_distance_ratio=cfg.max_distance_ratio,
    )
    removed_dup = colmap_calibration.filter_degenerate_real_cameras(
        recon, set(virtual_rel_names), min_separation_mm=cfg.min_camera_separation_mm,
    )
    if removed_far:
        log.info("%s: discarded %d implausibly-far real camera(s): %s", set_name, len(removed_far), removed_far)
    if removed_dup:
        log.info("%s: discarded %d degenerate/duplicate real camera(s): %s", set_name, len(removed_dup), removed_dup)

    recon = colmap_calibration.triangulate_seed(
        recon, db_path, work_dir, full_triangulated_dir, random_seed=cfg.random_seed, num_threads=cfg.num_threads,
    )

    stats = colmap_calibration.registration_summary(recon, set(virtual_rel_names))
    n_real = stats["num_registered_real"]
    registered_fraction = n_real / len(real_rel_names)
    log.info(
        "%s: registered %d/%d real cameras (%.0f%%), mean reprojection error %.2fpx",
        set_name, n_real, len(real_rel_names), 100 * registered_fraction, stats["mean_reprojection_error"],
    )
    if n_real == 0:
        raise RuntimeError("zero real cameras registered")
    if registered_fraction < cfg.min_registered_fraction_warn:
        log.warning(
            "%s: only %.0f%% of real photographs registered (%d/%d) - camera geometry may be "
            "unreliable for this set (consider raising N_VIRTUAL_VIEWS or loosening the "
            "ABS_POSE_* thresholds)",
            set_name, 100 * registered_fraction, n_real, len(real_rel_names),
        )

    final_recon = colmap_calibration.strip_virtual_images(recon, set(virtual_rel_names))
    return dict(final_recon=final_recon, work_dir=work_dir)


# --- Stage 2: dense point-cloud reconstruction (paper Section 2.4.3) -----------------------


def stage_dense_reconstruction(
    set_name: str, cfg: Config, final_recon: pycolmap.Reconstruction, work_dir: Path
) -> tuple[np.ndarray, np.ndarray]:
    out_dir = cfg.output_dir / set_name / "dense"
    sparse_dir = out_dir / "sparse"
    workspace_dir = out_dir / "workspace"
    fused_dir = out_dir / "fused"
    out_dir.mkdir(parents=True, exist_ok=True)

    t0 = time.time()
    undistorted_names = dr.undistort_for_stereo(final_recon, sparse_dir, work_dir, workspace_dir, max_image_size=-1)
    log.info("%s: undistorted %d images for dense stereo (%.1fs)", set_name, len(undistorted_names), time.time() - t0)

    t0 = time.time()
    dr.run_patch_match_stereo(
        workspace_dir, max_image_size=cfg.patch_match_max_image_size,
        window_radius=cfg.patch_match_window_radius, num_iterations=cfg.patch_match_num_iterations,
    )
    log.info("%s: PatchMatchStereo done (%.1fs)", set_name, time.time() - t0)

    t0 = time.time()
    fused = dr.run_stereo_fusion(
        workspace_dir, fused_dir, min_num_pixels=cfg.fusion_min_num_pixels,
        max_reproj_error=cfg.fusion_max_reproj_error, max_depth_error=cfg.fusion_max_depth_error,
    )
    log.info("%s: StereoFusion done (%.1fs), %d fused points", set_name, time.time() - t0, fused.num_points3D())

    points, colors = dr.points_and_colors(fused)
    if len(points) < cfg.min_fused_points_warn:
        log.warning(
            "%s: only %d dense points fused - cropping/registration may fail or produce a poor "
            "result for this set", set_name, len(points),
        )

    roi_points, _ = dr.filter_by_radius(points, colors, radius_mm=cfg.roi_radius_mm)
    if len(roi_points) > 0:
        roi_area = dr.bounding_box_surface_area_mm2(roi_points)
        achieved = len(roi_points) / roi_area
        targets = ", ".join(f"{t:.0f}" for t in cfg.density_targets_per_mm2)
        log.info("%s: ROI density ~%.2f points/mm^2 (targets: %s pts/mm^2)", set_name, achieved, targets)

    return points, colors


# --- Stage 3: cropping (paper Section 2.4.4) ------------------------------------------------


def stage_cropping(set_name: str, cfg: Config, points: np.ndarray, colors: np.ndarray) -> tuple[np.ndarray, np.ndarray, float, float]:
    points = cr.correct_z_axis_inversion(points)

    xy_points_for_plane, _, _ = cr.crop_xy(points, colors, cfg.pattern_size_mm)
    plane = cr.fit_dominant_plane(
        xy_points_for_plane, distance_threshold_mm=cfg.plane_fit_distance_threshold_mm,
        n_trials=cfg.plane_fit_n_trials,
    )
    log.info(
        "%s: turntable tilt %.1f deg from level (%.0f%% of the x/y-cropped cloud is this one plane)",
        set_name, plane.tilt_deg, 100 * plane.inlier_fraction,
    )
    if plane.inlier_fraction < cfg.min_plane_inlier_fraction_warn:
        log.warning(
            "%s: only %.0f%% of the x/y-cropped cloud fit one dominant plane - the turntable-tilt "
            "correction may have locked onto the wrong surface (e.g. a large/flat artefact competing "
            "with the real turntable) rather than the genuine pattern/turntable plane",
            set_name, 100 * plane.inlier_fraction,
        )
    points = cr.level_plane(points, plane)

    xy_points, xy_colors, _ = cr.crop_xy(points, colors, cfg.pattern_size_mm)

    z_limit, profile, reached_threshold = cr.find_lower_z_limit(
        xy_points, xy_colors, turntable_z=cfg.turntable_z_mm, start_offset_mm=cfg.z_start_offset_mm,
        slice_thickness_mm=cfg.z_slice_thickness_mm, luminosity_threshold=cfg.luminosity_threshold,
        min_sustained_slices=cfg.min_sustained_slices,
    )
    if not reached_threshold:
        log.info(
            "%s: luminosity threshold never reached - no dark support material detected, z lower "
            "limit fell back to the %.1fmm start offset", set_name, z_limit,
        )

    z_max = float(xy_points[:, 2].max()) if len(xy_points) else 0.0
    z_upper_limit = cr.find_upper_z_limit(
        profile, fallback_z_max=z_max, gap_thickness_mm=cfg.gap_thickness_mm, min_points_per_slice=cfg.min_points_per_slice,
    )
    if z_upper_limit < z_max:
        log.info("%s: trimmed floating debris above z=%.1fmm (cloud's own max was %.1fmm)", set_name, z_upper_limit, z_max)

    cropped_points, cropped_colors, _ = cr.crop_to_pattern_and_z(points, colors, cfg.pattern_size_mm, z_limit, z_upper_limit)
    log.info("%s: cropped %d -> %d points", set_name, len(points), len(cropped_points))
    if len(cropped_points) < cfg.min_cropped_points_warn:
        log.warning(
            "%s: only %d points survived cropping - the crop box or z-limits may be wrong for "
            "this set", set_name, len(cropped_points),
        )

    before_isolated = len(cropped_points)
    cropped_points, cropped_colors = cr.remove_isolated_points(
        cropped_points, cropped_colors, radius=cfg.isolation_radius_mm, min_component_size=cfg.min_component_size,
    )
    removed_isolated = before_isolated - len(cropped_points)
    if removed_isolated:
        log.info("%s: removed %d floating/isolated point(s) (%.2f%%)", set_name, removed_isolated, 100 * removed_isolated / before_isolated)

    return cropped_points, cropped_colors, z_limit, z_upper_limit


# --- Stage 4: point-cloud registration (paper Section 2.4.4) -------------------------------


def run_registration(
    cropped: dict[str, tuple[np.ndarray, np.ndarray, float, float]], cfg: Config
) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict[str, tuple[np.ndarray, np.ndarray]]]:
    set_names = list(cropped.keys())
    reference_set = cfg.reference_set if cfg.reference_set in cropped else set_names[0]
    if cfg.reference_set and cfg.reference_set not in cropped:
        log.warning(
            "requested reference set %r was not successfully cropped - falling back to %r",
            cfg.reference_set, reference_set,
        )

    confidences = {}
    for name, (points, _colors, z_lower, _z_upper) in cropped.items():
        normals = reg.estimate_normals(points, k=cfg.normal_k)
        confidence = reg.compute_confidence(points, normals, z_lower, lam=cfg.lambda_mm)
        confidences[name] = confidence
        log.info(
            "%s: confidence mean=%.2f (min=%.2f, max=%.2f)", name, confidence.mean(), confidence.min(), confidence.max()
        )

    ref_points, ref_colors, _, _ = cropped[reference_set]
    merged_points = ref_points.copy()
    merged_colors = ref_colors.copy()
    merged_confidence = confidences[reference_set].copy()
    transforms: dict[str, tuple[np.ndarray, np.ndarray]] = {reference_set: (np.eye(3), np.zeros(3))}

    for name in set_names:
        if name == reference_set:
            continue
        src_points, src_colors, _, _ = cropped[name]
        src_confidence = confidences[name]
        t0 = time.time()

        coarse = reg.coarse_align(
            src_points, merged_points, n_search_points=cfg.coarse_n_search_points, n_trials=cfg.coarse_n_trials,
            max_total_candidates=cfg.coarse_max_total_candidates, distance_tolerance=cfg.coarse_distance_tolerance_mm,
            overlap_threshold=cfg.coarse_overlap_threshold_mm, min_base_spread=cfg.coarse_min_base_spread_mm,
            n_score_points=cfg.coarse_n_score_points,
        )
        if coarse.score < cfg.min_coarse_score_warn:
            log.warning(
                "%s: coarse alignment score is low (%.3f) - registration onto the merged cloud "
                "may have failed to find the correct alignment", name, coarse.score,
            )

        R, t, icp_history = reg.weighted_icp(
            src_points, src_confidence, merged_points, merged_confidence, init_R=coarse.R, init_t=coarse.t,
            max_iterations=cfg.icp_max_iterations, max_correspondence_distance=cfg.icp_max_correspondence_distance_mm,
        )

        transformed_points = reg.apply_transform(src_points, R, t)
        tree = cKDTree(merged_points)
        nn_dist, _ = tree.query(transformed_points, k=1, workers=-1)
        median_nn = float(np.median(nn_dist)) if len(nn_dist) else float("inf")
        log.info(
            "%s: coarse score=%.3f, %d ICP iterations (%.1fs), median nn dist=%.2fmm",
            name, coarse.score, len(icp_history), time.time() - t0, median_nn,
        )
        if median_nn > cfg.icp_max_correspondence_distance_mm:
            log.warning(
                "%s: median nearest-neighbour distance after ICP (%.2fmm) exceeds the ICP "
                "correspondence threshold (%.2fmm) - this set's alignment may not have converged",
                name, median_nn, cfg.icp_max_correspondence_distance_mm,
            )

        transforms[name] = (R, t)
        before_n = len(merged_points)
        merged_points, merged_colors, merged_confidence = reg.merge_point_clouds(
            merged_points, merged_colors, merged_confidence,
            transformed_points, src_colors, src_confidence, merge_radius=cfg.merge_radius_mm,
        )
        log.info("%s: merged %d + %d -> %d points", name, before_n, len(src_points), len(merged_points))

    return merged_points, merged_colors, merged_confidence, transforms


# --- Stage 5: meshing (paper Section 2.4.5) -------------------------------------------------


def run_meshing(
    merged_points: np.ndarray, merged_colors: np.ndarray, merged_confidence: np.ndarray, cfg: Config
) -> tuple[o3d.geometry.TriangleMesh, Path]:
    clean_points, clean_colors, _clean_confidence = mesh.remove_isolated_points(
        merged_points, merged_colors, merged_confidence,
        radius=cfg.isolation_radius_mm, min_component_size=cfg.min_component_size,
    )
    removed = len(merged_points) - len(clean_points)
    removed_fraction = removed / max(len(merged_points), 1)
    log.info("meshing: removed %d/%d isolated points (%.1f%%)", removed, len(merged_points), 100 * removed_fraction)
    if removed_fraction > cfg.isolated_fraction_warn:
        log.warning(
            "meshing: an unusually high fraction of points (%.1f%%) were isolated/removed before "
            "Poisson reconstruction - check registration quality", 100 * removed_fraction,
        )

    pcd = mesh.build_point_cloud(clean_points, clean_colors)
    mesh.estimate_normals(pcd, k=cfg.normal_k)

    t0 = time.time()
    tri_mesh, densities = mesh.poisson_reconstruct(
        pcd, depth=cfg.poisson_depth, scale=cfg.poisson_scale, linear_fit=cfg.poisson_linear_fit
    )
    vertices, faces, _colors = mesh.mesh_arrays(tri_mesh)
    log.info(
        "meshing: Poisson reconstruction done (%.1fs) -> %d vertices, %d faces",
        time.time() - t0, len(vertices), len(faces),
    )
    if len(faces) < 1000:
        log.warning(
            "meshing: mesh has very few faces (%d) - reconstruction may have failed, or the "
            "merged cloud may be too sparse/noisy", len(faces),
        )

    tri_mesh = mesh.trim_low_density_vertices(tri_mesh, densities, quantile=cfg.poisson_density_trim_quantile)
    tri_mesh = mesh.remove_small_mesh_components(tri_mesh, min_triangles=cfg.mesh_min_component_triangles)
    trimmed_vertices, trimmed_faces, _colors = mesh.mesh_arrays(tri_mesh)
    log.info(
        "meshing: trimmed low-density/disconnected debris -> %d vertices, %d faces (was %d, %d)",
        len(trimmed_vertices), len(trimmed_faces), len(vertices), len(faces),
    )

    mesh_path = cfg.output_dir / "mesh" / "mesh.ply"
    mesh.write_mesh(mesh_path, tri_mesh)
    log.info("meshing: wrote %s", mesh_path)
    return tri_mesh, mesh_path


# --- Stage 6: texturing (paper Section 2.4.6) -----------------------------------------------


def run_texturing(
    tri_mesh: o3d.geometry.TriangleMesh,
    set_data: dict[str, dict],
    transforms: dict[str, tuple[np.ndarray, np.ndarray]],
    cfg: Config,
) -> Path:
    raster_dir = cfg.output_dir / "textured" / "rasters"

    cameras = []
    for set_name, data in set_data.items():
        if set_name not in transforms:
            continue
        R_reg, t_reg = transforms[set_name]
        for pose in tex.load_set_cameras(data["final_recon"], data["work_dir"]):
            R_common, t_common = tex.camera_pose_in_common_frame(pose, R_reg, t_reg)
            cameras.append(dict(set=set_name, pose=pose, R=R_common, t=t_common))
    log.info("texturing: %d registered real cameras across %d set(s)", len(cameras), len(set_data))

    verts = np.asarray(tri_mesh.vertices)
    centroid = verts.mean(axis=0) if len(verts) else np.zeros(3)
    bad_angle_count = 0
    for c in cameras:
        center, forward = tex.camera_center_and_forward(c["R"], c["t"])
        to_centroid = centroid - center
        norm = np.linalg.norm(to_centroid)
        if norm <= 0:
            continue
        angle = np.degrees(np.arccos(np.clip(forward @ (to_centroid / norm), -1, 1)))
        if angle > cfg.camera_angle_warn_deg:
            bad_angle_count += 1
    if bad_angle_count:
        log.warning(
            "texturing: %d/%d repositioned cameras point more than %.0f degrees away from the "
            "mesh centroid - the camera-pose chain (z-flip/registration) may be wrong for some set",
            bad_angle_count, len(cameras), cfg.camera_angle_warn_deg,
        )

    for c in cameras:
        png = tex.undistort_photo(c["pose"], raster_dir)
        c["image"] = tex.load_rgb(png)
        c["focal_px"] = c["pose"].focal_px
        c["principal_point"] = c["pose"].principal_point

    tri_mesh.compute_vertex_normals()
    photo_colors, has_photo_color = tex.texture_mesh_vertex_colors(
        tri_mesh, cameras, occlusion_eps=cfg.occlusion_eps_mm, min_facing=cfg.min_facing,
    )
    coverage = float(has_photo_color.mean()) if len(has_photo_color) else 0.0
    log.info("texturing: %.1f%% of vertices coloured directly from a registered photo", 100 * coverage)
    if coverage < cfg.min_vertex_coverage_warn:
        log.warning(
            "texturing: only %.1f%% vertex coverage from photos - the textured mesh relies "
            "heavily on the Step 5 point-cloud fallback colour", 100 * coverage,
        )

    original_colors = np.asarray(tri_mesh.vertex_colors) * 255.0
    final_colors = np.where(has_photo_color[:, None], photo_colors, original_colors)
    tri_mesh.vertex_colors = o3d.utility.Vector3dVector(np.clip(final_colors, 0, 255) / 255.0)

    textured_path = cfg.output_dir / "textured" / "textured.ply"
    textured_path.parent.mkdir(parents=True, exist_ok=True)
    o3d.io.write_triangle_mesh(str(textured_path), tri_mesh)
    log.info("texturing: wrote %s", textured_path)
    return textured_path


# --- Optional intermediate point-cloud persistence (--keep-intermediates only) -------------
#
# None of these are needed by the pipeline itself - every stage below passes its output to the
# next in memory. They exist purely so `--keep-intermediates` has plain, inspectable .ply files
# to keep instead of COLMAP's own (much larger, much less portable) database/workspace
# directories - see `cleanup_intermediates`.


def write_sparse_ply(cfg: Config, set_name: str, recon: pycolmap.Reconstruction) -> Path:
    """`outputs/<set>/sparse.ply` - Step 4's calibrated real-camera sparse point cloud."""
    path = cfg.output_dir / set_name / "sparse.ply"
    path.parent.mkdir(parents=True, exist_ok=True)
    recon.export_PLY(str(path))
    return path


def write_dense_ply(cfg: Config, set_name: str, points: np.ndarray, colors: np.ndarray) -> Path:
    """`outputs/<set>/dense.ply` - Step 5's raw, unfiltered dense fused point cloud."""
    path = cfg.output_dir / set_name / "dense.ply"
    dr.write_ply(path, points, colors)
    return path


def write_cropped_ply(cfg: Config, set_name: str, points: np.ndarray, colors: np.ndarray) -> Path:
    """`outputs/<set>/cropped/cropped.ply` - Step 6's cropped, isolated-point-cleaned cloud."""
    path = cfg.output_dir / set_name / "cropped" / "cropped.ply"
    cr.write_ply(path, points, colors)
    return path


def write_merged_ply(cfg: Config, points: np.ndarray, colors: np.ndarray) -> Path:
    """`outputs/merged/merged.ply` - Step 7's registered, merged cloud."""
    path = cfg.output_dir / "merged" / "merged.ply"
    reg.write_ply(path, points, colors)
    return path


# --- Cleanup ---------------------------------------------------------------------------------


def cleanup_intermediates(cfg: Config, keep_files: dict[str, Path]) -> None:
    """Delete everything under `cfg.output_dir` except `keep_files` (label -> path), each
    preserved at the same path, relative to `cfg.output_dir`, it already lives at.
    """
    for label, path in keep_files.items():
        if not path.exists() or path.stat().st_size == 0:
            raise RuntimeError(f"refusing to clean up: {label} ({path}) is missing or empty")

    with tempfile.TemporaryDirectory() as tmp:
        staged = []
        for path in keep_files.values():
            rel = path.relative_to(cfg.output_dir)
            tmp_path = Path(tmp) / rel
            tmp_path.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, tmp_path)
            staged.append((tmp_path, rel))

        shutil.rmtree(cfg.output_dir)
        cfg.output_dir.mkdir(parents=True)
        for tmp_path, rel in staged:
            dst = cfg.output_dir / rel
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.move(str(tmp_path), str(dst))

    log.info(
        "cleaned up intermediates - kept: %s",
        ", ".join(str(cfg.output_dir / rel) for _, rel in sorted(staged, key=lambda x: str(x[1]))),
    )


# --- CLI / orchestration ---------------------------------------------------------------------


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "End-to-end photogrammetric reconstruction (paper Sections 2.4.1-2.4.6): camera "
            "geometry estimation, dense reconstruction, cropping, registration, meshing, and "
            "texturing, iterating automatically over every set folder under images/."
        ),
    )
    parser.add_argument("--images-dir", type=Path, default=Path("images"),
                         help="directory containing one subfolder of .jpg photographs per acquisition set (default: images/)")
    parser.add_argument("--output-dir", type=Path, default=Path("outputs"),
                         help="working/output directory - cleared at the start of a run and reduced to mesh.ply + "
                              "textured.ply at the end (default: outputs/)")
    parser.add_argument("--sets", nargs="+", default=None,
                         help="restrict the run to these set names instead of auto-discovering every folder under --images-dir")
    parser.add_argument("--reference-set", default=None,
                         help="set used as the registration reference frame (default: the first discovered set)")
    parser.add_argument("--max-long-edge", type=int, default=3000,
                         help="downscale real photographs so their long edge is at most this many pixels "
                              "before SIFT/dense stereo (default: 3000)")
    parser.add_argument("--no-downscale", action="store_true",
                         help="process real photographs at native resolution - overrides --max-long-edge")
    parser.add_argument("--num-threads", type=int, default=1,
                         help="COLMAP thread count (default: 1, for reproducible results - see "
                              "camera_geometry_estimation.ipynb's own notes on this)")
    parser.add_argument("--random-seed", type=int, default=0)
    parser.add_argument("--keep-intermediates", action="store_true",
                         help="keep one .ply per major stage instead of just the final mesh/textured output: "
                              "outputs/<set>/sparse.ply, outputs/<set>/dense.ply, "
                              "outputs/<set>/cropped/cropped.ply, and outputs/merged/merged.ply - all written "
                              "specifically for this flag, since the pipeline itself never puts them on disk. "
                              "COLMAP's own databases/workspaces are still cleaned up either way")
    parser.add_argument("--log-level", default="INFO", choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    return parser.parse_args(argv)


def build_config(args: argparse.Namespace) -> Config:
    return Config(
        images_dir=args.images_dir,
        output_dir=args.output_dir,
        max_long_edge=None if args.no_downscale else args.max_long_edge,
        num_threads=args.num_threads,
        random_seed=args.random_seed,
        reference_set=args.reference_set,
    )


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    logging.basicConfig(level=args.log_level, format="%(asctime)s %(levelname)-8s %(message)s", datefmt="%H:%M:%S")

    cfg = build_config(args)

    if not check_cuda_available():
        log.warning(
            "no NVIDIA GPU detected (nvidia-smi not found or failed) - dense reconstruction "
            "(PatchMatchStereo) requires CUDA and has no CPU fallback; this run will likely fail there"
        )

    set_names = args.sets if args.sets else discover_sets(cfg.images_dir)
    if not set_names:
        log.error("no set folders with .jpg photographs found under %s", cfg.images_dir)
        return 1
    log.info("discovered %d set(s): %s", len(set_names), set_names)

    if cfg.output_dir.exists():
        log.info("clearing existing %s before a fresh run", cfg.output_dir)
        shutil.rmtree(cfg.output_dir)
    cfg.output_dir.mkdir(parents=True)

    set_data: dict[str, dict] = {}
    cropped: dict[str, tuple[np.ndarray, np.ndarray, float, float]] = {}
    keep_files: dict[str, Path] = {}
    for set_name in set_names:
        try:
            log.info("=== %s: camera geometry estimation ===", set_name)
            geom = stage_camera_geometry(set_name, cfg)
            if args.keep_intermediates:
                sparse_path = write_sparse_ply(cfg, set_name, geom["final_recon"])
                keep_files[f"{set_name}/sparse"] = sparse_path
                log.info("%s: wrote %s (--keep-intermediates)", set_name, sparse_path)

            log.info("=== %s: dense reconstruction ===", set_name)
            points, colors = stage_dense_reconstruction(set_name, cfg, geom["final_recon"], geom["work_dir"])
            if args.keep_intermediates:
                dense_path = write_dense_ply(cfg, set_name, points, colors)
                keep_files[f"{set_name}/dense"] = dense_path
                log.info("%s: wrote %s (--keep-intermediates)", set_name, dense_path)

            log.info("=== %s: cropping ===", set_name)
            cropped_points, cropped_colors, z_lower, z_upper = stage_cropping(set_name, cfg, points, colors)
            if args.keep_intermediates:
                cropped_ply_path = write_cropped_ply(cfg, set_name, cropped_points, cropped_colors)
                keep_files[f"{set_name}/cropped"] = cropped_ply_path
                log.info("%s: wrote %s (--keep-intermediates)", set_name, cropped_ply_path)
        except Exception:
            log.exception("%s: failed - skipping this set", set_name)
            continue

        set_data[set_name] = dict(final_recon=geom["final_recon"], work_dir=geom["work_dir"])
        cropped[set_name] = (cropped_points, cropped_colors, z_lower, z_upper)

    if not cropped:
        log.error("every set failed before registration - nothing to reconstruct")
        return 1
    if len(cropped) < len(set_names):
        log.warning("only %d/%d sets made it to registration: %s", len(cropped), len(set_names), sorted(cropped))
    if len(cropped) == 1:
        log.warning(
            "only one set succeeded (%s) - skipping multi-view registration/merge; the mesh will "
            "be built from this single partial acquisition alone", next(iter(cropped)),
        )

    log.info("=== point cloud registration ===")
    merged_points, merged_colors, merged_confidence, transforms = run_registration(cropped, cfg)
    if args.keep_intermediates:
        merged_ply_path = write_merged_ply(cfg, merged_points, merged_colors)
        keep_files["merged"] = merged_ply_path
        log.info("wrote %s (--keep-intermediates)", merged_ply_path)

    log.info("=== meshing ===")
    tri_mesh, mesh_path = run_meshing(merged_points, merged_colors, merged_confidence, cfg)
    keep_files["mesh"] = mesh_path

    log.info("=== texturing ===")
    textured_path = run_texturing(tri_mesh, set_data, transforms, cfg)
    keep_files["textured"] = textured_path

    log.info("=== cleanup ===")
    cleanup_intermediates(cfg, keep_files)

    log.info("done: %s, %s", mesh_path, textured_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
