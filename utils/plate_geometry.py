"""Virtual calibration-plate viewpoints: camera poses and synthetic photograph rendering.

The calibration plate is a flat, printed pattern lying in the world plane Z=0,
centred at the world origin. Virtual cameras are placed on a sphere around it and
always look at the origin. Camera axes follow the OpenCV/COLMAP convention
(+X right, +Y down, +Z forward into the scene), so a world point maps to the
camera frame as  x_cam = R @ x_world + t.
"""

from dataclasses import dataclass

import cv2
import numpy as np

@dataclass
class VirtualView:
    name: str
    azimuth_deg: float
    elevation_deg: float
    distance_mm: float
    R: np.ndarray  # (3, 3) world -> camera rotation
    t: np.ndarray  # (3,)   world -> camera translation
    center: np.ndarray  # (3,) camera centre in world coordinates


def look_at_origin(center: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """World->camera (R, t) for a camera at `center`, looking at the world origin with
    world +Z as the up reference.

    This is factored out of `spherical_pose` because it is also the exact transform
    needed to resolve the planar pose ambiguity in `colmap_calibration.py`: two camera
    centres related by reflection through the Z=0 plane (negate just the Z component)
    produce, via this same construction, the two poses that a coplanar point set cannot
    distinguish by reprojection error alone.
    """
    forward = -center / np.linalg.norm(center)
    world_up = np.array([0.0, 0.0, 1.0])
    right = np.cross(forward, world_up)
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)

    R = np.stack([right, down, forward])
    t = -R @ center
    return R, t


def spherical_pose(azimuth_deg: float, elevation_deg: float, distance_mm: float):
    """World->camera (R, t) for a camera on a sphere of given radius, looking at the origin."""
    az = np.radians(azimuth_deg)
    el = np.radians(elevation_deg)

    center = distance_mm * np.array([
        np.cos(el) * np.cos(az),
        np.cos(el) * np.sin(az),
        np.sin(el),
    ])
    R, t = look_at_origin(center)
    return R, t, center


def generate_virtual_views(
    n_views: int = 12,
    elevation_deg: float = 45.0,
    distance_mm: float = 250.0,
    azimuth_start_deg: float = 0.0,
) -> list[VirtualView]:
    """The paper's calibration sequence: n_views around the plate at a fixed elevation."""
    step_deg = 360.0 / n_views
    views = []
    for i in range(n_views):
        az = azimuth_start_deg + i * step_deg
        R, t, center = spherical_pose(az, elevation_deg, distance_mm)
        views.append(VirtualView(
            name=f"virtual_{i:02d}",
            azimuth_deg=az, elevation_deg=elevation_deg, distance_mm=distance_mm,
            R=R, t=t, center=center,
        ))
    return views


def focal_length_for_fill(
    plate_size_mm: float,
    distance_mm: float,
    image_width: int,
    image_height: int,
    fill_fraction: float = 0.6,
) -> float:
    """Pick a focal length (px) so the plate occupies roughly `fill_fraction` of the
    shorter image dimension at the given distance, using the plate's half-diagonal
    as a generous bounding radius. Keeps the virtual views framed like plausible
    real photographs without depending on any specific camera's hardware specs.
    """
    radius_mm = plate_size_mm * np.sqrt(2) / 2
    half_fov = np.arcsin(min(radius_mm / distance_mm, 0.99)) / fill_fraction
    return (min(image_width, image_height) / 2.0) / np.tan(half_fov)


def build_intrinsics(focal_px: float, image_width: int, image_height: int) -> np.ndarray:
    cx, cy = image_width / 2.0, image_height / 2.0
    return np.array([
        [focal_px, 0.0, cx],
        [0.0, focal_px, cy],
        [0.0, 0.0, 1.0],
    ])


def render_plate_photograph(
    plate_image: np.ndarray,
    plate_size_mm: float,
    K: np.ndarray,
    R: np.ndarray,
    t: np.ndarray,
    out_size: tuple[int, int],
) -> tuple[np.ndarray, np.ndarray]:
    """Warp the flat, fronto-parallel plate image into a synthetic photograph.

    The plate is treated as a planar target lying in Z=0, centred at the world
    origin, so a single homography (plate pixels -> rendered pixels) exactly
    reproduces what a pinhole camera at (R, t) would see of it.

    Returns (rendered_image, projected_corners) where corners are ordered
    top-left, top-right, bottom-right, bottom-left, matching plate_image's own
    corners.
    """
    half = plate_size_mm / 2.0
    corners_world = np.array([
        [-half, -half, 0.0],
        [half, -half, 0.0],
        [half, half, 0.0],
        [-half, half, 0.0],
    ])
    rvec, _ = cv2.Rodrigues(R)
    corners_img, _ = cv2.projectPoints(corners_world, rvec, t, K, None)
    corners_img = corners_img.reshape(-1, 2).astype(np.float32)

    h, w = plate_image.shape[:2]
    corners_src = np.array([[0, 0], [w, 0], [w, h], [0, h]], dtype=np.float32)

    H, _ = cv2.findHomography(corners_src, corners_img)
    rendered = cv2.warpPerspective(
        plate_image, H, out_size,
        borderMode=cv2.BORDER_CONSTANT, borderValue=255,
    )
    return rendered, corners_img
