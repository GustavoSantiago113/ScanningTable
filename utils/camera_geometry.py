import numpy as np
import cv2
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

def camera_pose(azimuth_deg: float, elevation_deg: float, distance: float):
    """
    Return (R, t, cam_pos_world) for a camera at the given spherical position
    looking at the world origin.

    Convention (OpenCV): X = right, Y = down, Z = forward.
    A world point X_w maps to camera frame as: X_c = R @ X_w + t
    """
    az = np.radians(azimuth_deg)
    el = np.radians(elevation_deg)

    cam_pos = np.array([
        distance * np.cos(el) * np.cos(az),
        distance * np.cos(el) * np.sin(az),
        distance * np.sin(el),
    ])

    # Camera Z-axis: pointing from camera toward origin
    z_cam = -cam_pos / np.linalg.norm(cam_pos)

    # Camera X-axis: right = forward × world_up  (world_up = Z)
    world_up = np.array([0.0, 0.0, 1.0])
    x_cam = np.cross(z_cam, world_up)
    x_cam /= np.linalg.norm(x_cam)

    # Camera Y-axis: down = forward × right
    y_cam = np.cross(z_cam, x_cam)
    y_cam /= np.linalg.norm(y_cam)

    R = np.array([x_cam, y_cam, z_cam])   # rows = camera axes in world
    t = -R @ cam_pos                       # translation in camera frame
    return R, t, cam_pos