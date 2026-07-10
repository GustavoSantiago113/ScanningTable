"""Texturing: paper's Section 2.4.6, following on from Step 8's mesh (`meshing.py`):

> The texturing process refers back to the original sets of photographs and the camera
> position information calculated during the sparse point-cloud reconstruction to determine
> the detailed appearance (i.e., texture) of each face of the mesh. ... This is a more complex
> task than in the conventional single-scan photogrammetry workflow because the merging
> process will have reoriented the component parts of the mesh, requiring the camera positions
> to be moved accordingly. The starting point of the process is the set of camera locations
> estimated by the bundle adjustment processing of VisualSFM ... As a result of the point cloud
> registration stage, each of the N partial meshes has been transformed from the location
> assumed by the bundle adjustment process. As a result, before texturing, the mesh must also
> be transformed by a model-matrix, Mn, which is the inverse of the optimal transform
> calculated during point-cloud registration ...

This replica repositions Step 4's calibrated real cameras (`outputs/<set>/final`) into Step 8's
mesh's common frame - this replica's "VmnMn" - by carrying each one through the same chain the
mesh itself went through:

  1. **Step 6's z-axis-inversion correction.** Step 6 corrects the *points* by negating z
     outright (`cropping.correct_z_axis_inversion`) - a reflection, not a rotation, so there's
     no way to move a camera the same way and still call the result a physically normal
     camera. `flip_z_camera_pose` derives the (improper, but correctly-reprojecting) camera
     analogue instead - see its docstring.
  2. **Step 7's registration transform** (`outputs/merged/transforms.json`) - the same
     per-set rigid transform `registration.apply_transform` carries the points through,
     carried through the camera pose the same way (`compose_with_registration`).

With every camera repositioned, `texture_mesh_vertex_colors` projects each registered,
undistorted photo onto the mesh and paints each *vertex* with the occlusion-aware, multi-view-
blended result - not a UV-mapped texture atlas, but real photographic colour, correctly
occlusion-tested per camera via `open3d`'s own `RaycastingScene` (Embree-backed, CPU-only, no
GL/display dependency).

Two things needed correcting empirically before that result was trustworthy, not just
plausible-looking - see that function's own docstring for the full account:

- **Ray direction.** Casting the occlusion ray *from* each vertex *towards* the camera
  routinely self-intersects an adjacent triangle at t &asymp; 0 ("shadow acne") - this mesh is
  not watertight or edge-manifold. Casting *from the camera towards* the vertex instead avoids
  the problem entirely and doubles as a proper z-buffer-style visibility test.
- **Occlusion tolerance.** An overly strict tolerance was rejecting vertices whose normals
  point almost exactly at the camera; tracing a few of those hits showed real geometry several
  mm to multiple cm away, not numerical noise - genuine self-occlusion on a small, geometrically
  complex hand-painted miniature (limbs, weapon, folds). A wider, calibrated tolerance fixed the
  false rejections without papering over real ones.
"""

import json
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d
import pycolmap

# Step 6's point correction (`cropping.correct_z_axis_inversion`), as a matrix: Fz.
FLIP_Z = np.diag([1.0, 1.0, -1.0])


@dataclass
class CameraPose:
    """A single registered real camera, straight out of Step 4's `outputs/<set>/final`
    reconstruction - pose and intrinsics still in that set's own local frame.
    """
    name: str  # e.g. "set_1/stop_01_20260703_163611.jpg"
    image_path: Path  # the exact working-resolution photo COLMAP calibrated against
    width: int
    height: int
    focal_px: float
    principal_point: tuple[float, float]
    radial_k: float
    R: np.ndarray  # (3, 3) world (this set's own frame) -> camera
    t: np.ndarray  # (3,)


def load_set_cameras(recon: pycolmap.Reconstruction, work_dir: Path) -> list[CameraPose]:
    """Registered real cameras from Step 4's final reconstruction (`recon` - e.g.
    `pycolmap.Reconstruction(str(outputs/<set>/final))`, or an already-in-memory
    reconstruction straight from Step 4, no disk round-trip needed), paired with the
    working-resolution photograph (`outputs/<set>/work/...`) COLMAP actually calibrated them
    against - texture sampling needs the same pixel grid the intrinsics were fit to, not the
    original full-resolution photo (same reasoning `dense_reconstruction.ipynb` already relies
    on for Step 5).
    """
    poses = []
    for img in recon.images.values():
        if not img.has_pose:
            continue
        cam = recon.camera(img.camera_id)
        if cam.model != pycolmap.CameraModelId.SIMPLE_RADIAL:
            raise ValueError(f"{img.name}: unsupported camera model {cam.model}")
        f, cx, cy, k = cam.params
        cfw = img.cam_from_world()
        poses.append(CameraPose(
            name=img.name,
            image_path=work_dir / img.name,
            width=cam.width,
            height=cam.height,
            focal_px=float(f),
            principal_point=(float(cx), float(cy)),
            radial_k=float(k),
            R=cfw.rotation.matrix(),
            t=np.asarray(cfw.translation, dtype=np.float64),
        ))
    return poses


def load_registration_transforms(path: Path) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """Step 7's per-set rigid transform (`outputs/merged/transforms.json`) as (R, t) arrays,
    the same convention `registration.apply_transform` uses: p_merged = R @ p + t.
    """
    data = json.loads(path.read_text())
    return {name: (np.array(v["R"], dtype=np.float64), np.array(v["t"], dtype=np.float64)) for name, v in data.items()}


def flip_z_camera_pose(R: np.ndarray, t: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """The camera-side analogue of `cropping.correct_z_axis_inversion`'s point correction.

    The original (R, t) correctly reprojects every point P onto its real photographed pixel:
    x_cam = R @ P + t. Step 6 replaces P with P' = Fz @ P (Fz = diag(1, 1, -1), an involution),
    so P = Fz @ P', and x_cam = R @ Fz @ P' + t = (R @ Fz) @ P' + t. `(R @ Fz, t)` therefore
    reprojects corrected-frame points onto exactly the same, still-correct pixels - the linear
    projection formula doesn't care that, as an *extrinsic matrix*, `R @ Fz` is improper
    (det = -1: it's a rotoreflection, not a rotation). It isn't meant to describe a second,
    physically real camera - only to make the projection arithmetic land on the right pixel.
    The camera centre implied by this pair works out to exactly the reflection-through-z=0 that
    `plate_geometry.look_at_origin`'s docstring describes as this rig's other twisted-pair
    solution, matching Step 6's own point correction.
    """
    return R @ FLIP_Z, t.copy()


def compose_with_registration(
    R: np.ndarray, t: np.ndarray, R_reg: np.ndarray, t_reg: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Carry a camera pose through Step 7's registration transform, the same way
    `registration.apply_transform` carries points: p_merged = R_reg @ p + t_reg for a point p
    in this set's own (already z-corrected) frame. Substituting p = R_reg.T @ (p_merged -
    t_reg) into x_cam = R @ p + t gives the camera's pose directly in the merged frame.
    """
    R_final = R @ R_reg.T
    t_final = t - R_final @ t_reg
    return R_final, t_final


def camera_pose_in_common_frame(
    pose: CameraPose, R_reg: np.ndarray, t_reg: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """`pose`, carried from Step 4's own per-set frame into Step 8's mesh's common frame -
    this replica's "VmnMn".
    """
    R1, t1 = flip_z_camera_pose(pose.R, pose.t)
    return compose_with_registration(R1, t1, R_reg, t_reg)


def camera_center_and_forward(R: np.ndarray, t: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """A camera's centre and forward-look direction in world coordinates, from a world->camera
    (R, t) pair - for plotting the paper's Figure 9-style "camera positions and orientations"
    diagram. `R`'s rows are the camera's local axes expressed in world coordinates, so row 2
    (forward) doubles as the look direction without any extra inversion.
    """
    center = -R.T @ t
    forward = R[2, :]
    return center, forward


def undistort_photo(pose: CameraPose, dst_dir: Path) -> Path:
    """A PNG copy of `pose.image_path`, with Step 4's calibrated `SIMPLE_RADIAL` distortion
    (`pose.radial_k`) removed - `texture_mesh_vertex_colors` projects vertices with a plain
    pinhole model, so the photo it samples needs to already be undistorted to match. Cached to
    `dst_dir` (keyed on source mtime) since undistorting ~100 photos is the slow part of a
    re-run otherwise.
    """
    dst_dir.mkdir(parents=True, exist_ok=True)
    dst = dst_dir / (Path(pose.name).stem + ".png")
    if dst.exists() and dst.stat().st_mtime >= pose.image_path.stat().st_mtime:
        return dst

    img = cv2.imread(str(pose.image_path), cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError(f"could not read {pose.image_path}")
    cx, cy = pose.principal_point
    K = np.array([[pose.focal_px, 0, cx], [0, pose.focal_px, cy], [0, 0, 1]], dtype=np.float64)
    dist_coeffs = np.array([pose.radial_k, 0.0, 0.0, 0.0], dtype=np.float64)
    undistorted = cv2.undistort(img, K, dist_coeffs)
    cv2.imwrite(str(dst), undistorted)
    return dst


def load_rgb(path: Path) -> np.ndarray:
    """A photo as an (h, w, 3) RGB uint8 array (`cv2.imread` reads BGR;
    `texture_mesh_vertex_colors` expects RGB to match the mesh's own RGB vertex colors).
    """
    img = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError(f"could not read {path}")
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)


def sample_bilinear(image: np.ndarray, uv: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Bilinear-sample an (h, w, 3) image at floating-point pixel coordinates `uv` (n, 2).
    Returns (colors (n, 3) float, valid (n,) bool - False wherever `uv` falls outside the
    image, in which case that row of `colors` is meaningless).
    """
    h, w = image.shape[:2]
    u, v = uv[:, 0], uv[:, 1]
    valid = (u >= 0) & (u <= w - 1) & (v >= 0) & (v <= h - 1)

    uc = np.clip(u, 0, w - 1 - 1e-6)
    vc = np.clip(v, 0, h - 1 - 1e-6)
    u0, v0 = np.floor(uc).astype(int), np.floor(vc).astype(int)
    u1, v1 = u0 + 1, v0 + 1
    fu, fv = (uc - u0)[:, None], (vc - v0)[:, None]

    c00, c01 = image[v0, u0].astype(np.float64), image[v0, u1].astype(np.float64)
    c10, c11 = image[v1, u0].astype(np.float64), image[v1, u1].astype(np.float64)
    colors = (c00 * (1 - fu) + c01 * fu) * (1 - fv) + (c10 * (1 - fu) + c11 * fu) * fv
    return colors, valid


def texture_mesh_vertex_colors(
    mesh: o3d.geometry.TriangleMesh,
    cameras: list[dict],
    occlusion_eps: float = 1.5,
    min_facing: float = 0.1,
) -> tuple[np.ndarray, np.ndarray]:
    """Paint every mesh vertex with the confidence-weighted blend of every registered photo
    that can actually see it. `cameras` is a list of dicts, one per registered real photo,
    each with:
      - `R`, `t`: pose in the mesh's own common frame (`camera_pose_in_common_frame`)
      - `focal_px`, `principal_point`: intrinsics
      - `image`: the matching undistorted photo as an (h, w, 3) RGB array (`undistort_photo` +
        `load_rgb`)

    For each camera: project every vertex with the pinhole model; keep the ones that land
    in-frame, in front of the camera, and reasonably front-facing (`min_facing` on the vertex
    normal's dot product with the direction to the camera - grazing views are unreliable and
    excluded rather than down-weighted to near-zero). Of those, an Embree ray is cast *from the
    camera towards the vertex* (not the other way around - a ray starting exactly on the mesh
    surface routinely self-hits an adjacent triangle at t~0 from numerical noise alone,
    "shadow acne"; a ray starting far away at the camera has no such problem, and doubles as
    the correct z-buffer-style occlusion test: whatever it hits *first* is what the camera
    actually sees). A vertex is occluded only if the ray hits something *closer* than that
    vertex by more than `occlusion_eps` (loose enough to absorb the Poisson mesh's own sub-mm
    surface noise - `outputs/mesh/mesh.ply` is not perfectly watertight/edge-manifold - without
    papering over genuine occlusion, which was observed running many mm to multiple cm closer,
    not sub-mm, checked against this replica's own mesh); no hit at all (the ray reaches empty
    space beyond the vertex) also counts as visible. Surviving observations are averaged,
    weighted by the same front-facing dot product, so closer-to-head-on views dominate.

    Verified against this replica's own real data: a self-occlusion audit (this docstring's own
    development) found that vertices whose normals point almost exactly at a given camera, yet
    still test occluded, hit real geometry many mm to multiple cm away along that ray - not
    numerical noise near the vertex itself. On a small, geometrically complex hand-painted
    miniature (limbs, weapon, folds) photographed at one fixed camera elevation per set, most
    surface points genuinely are only visible from a narrow slice of the ~100 photos across all
    three sets - low per-vertex coverage here reflects that, not a broken occlusion test.

    Returns (colors (n, 3) 0-255 float, has_color (n,) bool) - `has_color` is False for any
    vertex no registered camera could see; `mesh`'s own existing vertex colours (from Step 7/8's
    point cloud) are a reasonable fallback for those, left to the caller to blend in.
    """
    vertices = np.asarray(mesh.vertices)
    if not mesh.has_vertex_normals():
        mesh.compute_vertex_normals()
    normals = np.asarray(mesh.vertex_normals)
    n = len(vertices)

    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))

    color_sum = np.zeros((n, 3), dtype=np.float64)
    weight_sum = np.zeros(n, dtype=np.float64)

    for cam in cameras:
        R, t, f = cam["R"], cam["t"], cam["focal_px"]
        cx, cy = cam["principal_point"]
        image = cam["image"]
        h, w = image.shape[:2]

        center = -R.T @ t
        to_cam = center[None, :] - vertices
        dist = np.linalg.norm(to_cam, axis=1)
        dirs_to_cam = to_cam / dist[:, None]
        facing = np.einsum("ij,ij->i", normals, dirs_to_cam)

        cam_space = vertices @ R.T + t
        in_front = cam_space[:, 2] > 1e-3
        u = f * cam_space[:, 0] / cam_space[:, 2] + cx
        v = f * cam_space[:, 1] / cam_space[:, 2] + cy
        in_bounds = (u >= 0) & (u <= w - 1) & (v >= 0) & (v <= h - 1)

        candidate = np.nonzero(in_front & in_bounds & (facing > min_facing))[0]
        if len(candidate) == 0:
            continue

        directions = (-dirs_to_cam[candidate]).astype(np.float32)  # camera -> vertex
        origins = np.broadcast_to(center.astype(np.float32), directions.shape)
        rays = o3d.core.Tensor(np.concatenate([origins, directions], axis=1))
        t_hit = scene.cast_rays(rays)["t_hit"].numpy()

        occluded = t_hit < (dist[candidate] - occlusion_eps)
        vis_idx = candidate[~occluded]
        if len(vis_idx) == 0:
            continue

        colors, _ = sample_bilinear(image, np.stack([u[vis_idx], v[vis_idx]], axis=1))
        w_ = facing[vis_idx]
        color_sum[vis_idx] += colors * w_[:, None]
        weight_sum[vis_idx] += w_

    has_color = weight_sum > 0
    colors_out = np.zeros((n, 3), dtype=np.float64)
    colors_out[has_color] = color_sum[has_color] / weight_sum[has_color, None]
    return colors_out, has_color
