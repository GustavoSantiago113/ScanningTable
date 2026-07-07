"""Cropping: remove the turntable, calibration pattern, and any support material from the
dense point cloud produced by Step 5 (dense_reconstruction.ipynb), leaving just the artefact.

Follows the paper's method directly:
  1. The calibration model is defined at z = 0 during bundle adjustment (Step 4), so an
     axis-aligned x/y box the size of the calibration pattern, centred at the world origin,
     already excludes almost everything that isn't the pattern or the object sitting on it.
  2. The z lower limit is *not* a fixed number - it's found by starting 2 mm above the
     turntable (z = 0) and sliding a 1 mm-thick horizontal slab upward, computing each
     slab's average point luminosity, until that average exceeds a threshold. Below that
     z, points belong to the dark supporting material (or the turntable itself); above it,
     points belong to the (much lighter) artefact. The z upper limit is just the cloud's own
     maximum z - there's nothing above the artefact to cut away.

On captures with no dark support material underneath the object (nothing beneath it needs
hiding), the very first slab at z = 2 mm already exceeds the threshold, so the z-crop is a
no-op and only the x/y box does any work - that's an honest reflection of the input, not a
bug in the search.
"""

from dataclasses import dataclass
from pathlib import Path

import numpy as np


def read_ply(path: Path) -> tuple[np.ndarray, np.ndarray | None]:
    """Read a binary-little-endian PLY of the shape written by `write_ply` below (and by
    dense_reconstruction.write_ply): float32 x/y/z, optional uchar red/green/blue.
    """
    with open(path, "rb") as f:
        header = b""
        while not header.endswith(b"end_header\n"):
            header += f.readline()
        header_text = header.decode("ascii")
        n = int(next(l for l in header_text.splitlines() if l.startswith("element vertex")).split()[-1])
        has_color = "red" in header_text

        dtype = [("x", "<f4"), ("y", "<f4"), ("z", "<f4")]
        if has_color:
            dtype += [("red", "u1"), ("green", "u1"), ("blue", "u1")]
        data = np.fromfile(f, dtype=dtype, count=n)

    points = np.stack([data["x"], data["y"], data["z"]], axis=1).astype(np.float64)
    colors = None
    if has_color:
        colors = np.stack([data["red"], data["green"], data["blue"]], axis=1).astype(np.float64)
    return points, colors


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


def correct_z_axis_inversion(points: np.ndarray) -> np.ndarray:
    """Negate z to correct a known Step 4 issue in this replica: the real-camera pose
    disambiguation for a coplanar calibration target (see `plate_geometry.look_at_origin`'s
    docstring on the reflection-through-z=0 ambiguity) resolves to the mirrored solution for
    this capture rig, so the whole real reconstruction - turntable, pattern, and artefact
    alike - comes out on the wrong side of the z = 0 plane, with the artefact reconstructed
    *below* the turntable instead of above it.

    This is a workaround at the data-consumption end so the rest of this module (written
    straight from the paper, which assumes the artefact sits above the turntable) applies
    correctly - the proper fix belongs in `colmap_calibration.py`'s pose disambiguation.
    """
    corrected = points.copy()
    corrected[:, 2] *= -1
    return corrected


def luminosity(colors: np.ndarray) -> np.ndarray:
    """Rec. 601 luma - the paper's "average luminosity" of a point's RGB colour."""
    return colors[:, 0] * 0.299 + colors[:, 1] * 0.587 + colors[:, 2] * 0.114


@dataclass
class ZSliceStat:
    z_lo: float
    z_hi: float
    num_points: int
    mean_luminosity: float  # nan if the slab is empty


def compute_luminosity_profile(
    points: np.ndarray,
    colors: np.ndarray,
    slice_thickness_mm: float = 1.0,
    z_min: float | None = None,
    z_max: float | None = None,
) -> list[ZSliceStat]:
    """Average point luminosity in consecutive `slice_thickness_mm`-thick horizontal slabs
    from `z_min` to `z_max` (defaulting to the cloud's own z-range) - the diagnostic behind
    `find_lower_z_limit`, also useful on its own for plotting the luminosity-vs-height
    profile that the threshold is chosen against.
    """
    lum = luminosity(colors)
    if z_min is None:
        z_min = float(points[:, 2].min())
    if z_max is None:
        z_max = float(points[:, 2].max())

    edges = np.arange(z_min, z_max + slice_thickness_mm, slice_thickness_mm)
    stats = []
    for z in edges[:-1]:
        mask = (points[:, 2] >= z) & (points[:, 2] < z + slice_thickness_mm)
        n = int(mask.sum())
        mean_lum = float(lum[mask].mean()) if n > 0 else float("nan")
        stats.append(ZSliceStat(float(z), float(z + slice_thickness_mm), n, mean_lum))
    return stats


def find_lower_z_limit(
    points: np.ndarray,
    colors: np.ndarray,
    turntable_z: float = 0.0,
    start_offset_mm: float = 2.0,
    slice_thickness_mm: float = 1.0,
    luminosity_threshold: float = 100.0,
) -> tuple[float, list[ZSliceStat], bool]:
    """The paper's z lower-limit search: starting `start_offset_mm` above `turntable_z`,
    slide a `slice_thickness_mm` slab upward until its average luminosity exceeds
    `luminosity_threshold`. Returns (z_limit, profile, threshold_reached).

    If no slab up to the cloud's own max z exceeds the threshold (e.g. this capture used no
    dark support material, so there's nothing dark to cut away), z_limit falls back to the
    starting z and `threshold_reached` is False - that's a correct outcome, not a failure.
    """
    z_start = turntable_z + start_offset_mm
    z_max = float(points[:, 2].max())

    if z_start >= z_max:
        return z_start, [], False

    profile = compute_luminosity_profile(points, colors, slice_thickness_mm, z_min=z_start, z_max=z_max)
    for stat in profile:
        if stat.num_points > 0 and stat.mean_luminosity > luminosity_threshold:
            return stat.z_lo, profile, True
    return z_start, profile, False


def find_upper_z_limit(
    profile: list[ZSliceStat],
    fallback_z_max: float,
    gap_thickness_mm: float = 5.0,
    min_points_per_slice: int = 5,
) -> float:
    """Find where the artefact's own point mass ends, using the same per-slice `profile`
    `find_lower_z_limit` already computed: the first z at which `gap_thickness_mm` worth of
    consecutive slices all fall below `min_points_per_slice` - a density gap separating the
    solid artefact from any disconnected reconstruction debris floating further up (e.g. a
    specular-highlight fusion ghost) that isn't turntable/pattern/support material, but isn't
    part of the artefact either. Returns `fallback_z_max` unchanged if no such gap is found -
    not every capture has debris to cut.
    """
    if not profile:
        return fallback_z_max

    slice_thickness_mm = profile[0].z_hi - profile[0].z_lo
    gap_slices = max(1, round(gap_thickness_mm / slice_thickness_mm))
    counts = [s.num_points for s in profile]

    for i in range(len(counts) - gap_slices + 1):
        if all(c < min_points_per_slice for c in counts[i:i + gap_slices]):
            return profile[i].z_lo
    return fallback_z_max


def crop_xy(
    points: np.ndarray,
    colors: np.ndarray | None,
    pattern_size_mm: float,
) -> tuple[np.ndarray, np.ndarray | None, np.ndarray]:
    """Keep only points whose x and y fall within the calibration pattern's footprint
    (an axis-aligned square of side `pattern_size_mm`, centred at the world origin where
    the pattern is defined). Returns (points, colors, keep_mask). This alone already
    discards the turntable rim and any background clutter outside the pattern - most of
    what a real capture's dense reconstruction picks up besides the pattern and artefact.
    """
    half = pattern_size_mm / 2.0
    mask = (
        (points[:, 0] >= -half) & (points[:, 0] <= half)
        & (points[:, 1] >= -half) & (points[:, 1] <= half)
    )
    cropped_colors = colors[mask] if colors is not None else None
    return points[mask], cropped_colors, mask


def crop_to_pattern_and_z(
    points: np.ndarray,
    colors: np.ndarray | None,
    pattern_size_mm: float,
    lower_z_limit: float,
    upper_z_limit: float | None = None,
) -> tuple[np.ndarray, np.ndarray | None, np.ndarray]:
    """The full crop: `crop_xy` followed by a z range from `lower_z_limit` up to
    `upper_z_limit` (defaults to the cloud's own max z - nothing above the artefact needs
    cutting). Returns (cropped_points, cropped_colors, keep_mask), the last relative to the
    original, uncropped `points`.
    """
    if upper_z_limit is None:
        upper_z_limit = float(points[:, 2].max())

    xy_points, xy_colors, xy_mask = crop_xy(points, colors, pattern_size_mm)
    z_mask = (xy_points[:, 2] >= lower_z_limit) & (xy_points[:, 2] <= upper_z_limit)

    cropped_points = xy_points[z_mask]
    cropped_colors = xy_colors[z_mask] if xy_colors is not None else None

    full_mask = xy_mask.copy()
    full_mask[xy_mask] = z_mask
    return cropped_points, cropped_colors, full_mask
