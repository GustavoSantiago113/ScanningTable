"""360-degree orbit-view video rendering for a finished, textured mesh (any triangle-mesh PLY
with per-vertex colours - e.g. Step 9's `textured.ply`, or anything saved under `results/`) - a
quick way to visually inspect a result without opening a GUI.

Uses open3d's offscreen renderer, which runs headless via EGL (verified working in this project's
WSL environment - no X server/display needed) rather than the interactive `Visualizer` window
this project's other notebooks use for on-screen previews.

Orbits the camera in the x/y plane around the mesh's own bounding-box centre at a fixed elevation,
matching this project's z-up convention (the turntable sits at world z = 0 throughout
`utils/cropping.py`).
"""

import argparse
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d


def orbit_camera_positions(
    center: np.ndarray, distance: float, num_frames: int, elevation_deg: float = 20.0
) -> list[np.ndarray]:
    """`num_frames` camera eye positions orbiting `center` at `distance`, evenly spaced around a
    full 360-degree turn in the x/y plane at a fixed `elevation_deg` above horizontal.
    """
    elevation = np.radians(elevation_deg)
    horizontal = distance * np.cos(elevation)
    height = distance * np.sin(elevation)
    angles = np.linspace(0, 2 * np.pi, num_frames, endpoint=False)
    return [center + np.array([horizontal * np.cos(a), horizontal * np.sin(a), height]) for a in angles]


def render_orbit_frames(
    mesh: o3d.geometry.TriangleMesh,
    num_frames: int = 120,
    width: int = 1280,
    height: int = 720,
    elevation_deg: float = 20.0,
    fov_deg: float = 30.0,
) -> list[np.ndarray]:
    """Render `num_frames` frames of `mesh` orbiting a full 360 degrees, as a list of (h, w, 3)
    uint8 RGB arrays. The camera distance is derived from `fov_deg` and the mesh's own bounding
    box so the object fills most of the frame at every orbit angle without clipping.
    """
    if not mesh.has_vertex_normals():
        mesh.compute_vertex_normals()

    bbox = mesh.get_axis_aligned_bounding_box()
    center = bbox.get_center()
    radius = float(np.linalg.norm(bbox.get_extent())) / 2  # bounding-sphere radius - orientation-
    # independent, so the object doesn't change apparent size as the camera orbits around it
    distance = radius / np.sin(np.radians(fov_deg / 2)) * 1.05  # exact sphere-fit distance + 5% margin

    renderer = o3d.visualization.rendering.OffscreenRenderer(width, height)
    renderer.scene.set_background([1.0, 1.0, 1.0, 1.0])
    material = o3d.visualization.rendering.MaterialRecord()
    material.shader = "defaultLit"
    renderer.scene.add_geometry("mesh", mesh, material)
    renderer.scene.scene.set_sun_light([-0.3, -0.3, -0.9], [1.0, 1.0, 1.0], 75000)
    renderer.scene.scene.enable_sun_light(True)

    up = np.array([0.0, 0.0, 1.0], dtype=np.float32)
    frames = []
    for eye in orbit_camera_positions(center, distance, num_frames, elevation_deg):
        renderer.setup_camera(fov_deg, center.astype(np.float32), eye.astype(np.float32), up)
        img = renderer.render_to_image()
        frames.append(np.asarray(img))
    return frames


def write_video(frames: list[np.ndarray], path: Path, fps: int = 30) -> None:
    """Write `frames` (RGB uint8 arrays, all the same size) to an mp4 at `path` via OpenCV."""
    if not frames:
        raise ValueError("no frames to write")
    height, width = frames[0].shape[:2]
    path.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height))
    for frame in frames:
        writer.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
    writer.release()


def render_360_video(
    ply_path: Path,
    output_path: Path,
    num_frames: int = 120,
    fps: int = 30,
    width: int = 1280,
    height: int = 720,
    elevation_deg: float = 20.0,
    fov_deg: float = 30.0,
) -> Path:
    """Load the triangle mesh at `ply_path` and write a `num_frames`-frame, `fps`-fps 360-degree
    orbit video to `output_path`.
    """
    mesh = o3d.io.read_triangle_mesh(str(ply_path))
    if len(mesh.triangles) == 0:
        raise ValueError(f"{ply_path} has no triangles - is this a point cloud, not a mesh?")
    frames = render_orbit_frames(
        mesh, num_frames=num_frames, width=width, height=height, elevation_deg=elevation_deg, fov_deg=fov_deg,
    )
    write_video(frames, output_path, fps=fps)
    return output_path


def render_stacked_360_video(
    ply_paths: list[Path],
    output_path: Path,
    num_frames: int = 120,
    fps: int = 30,
    width: int = 1080,
    row_height: int = 640,
    elevation_deg: float = 20.0,
    fov_deg: float = 30.0,
) -> Path:
    """Render each mesh in `ply_paths` as its own independent 360-degree orbit (same
    `num_frames`, so every mesh completes its turn in lockstep), then stack the frames
    vertically - one row per mesh, in the given order - into a single portrait ("Reels-style")
    video. Each mesh is framed independently within its own row (via `render_orbit_frames`'s own
    bounding-box-based framing), so objects of very different real-world sizes still each fill
    their row similarly, rather than being scaled relative to one another.

    Total frame size is `width` x (`row_height` * len(ply_paths)) - e.g. the default
    1080 x (640*3) = 1080x1920 matches the standard 9:16 vertical video aspect ratio.
    """
    if not ply_paths:
        raise ValueError("no meshes given to stack")

    rows = []
    for ply_path in ply_paths:
        mesh = o3d.io.read_triangle_mesh(str(ply_path))
        if len(mesh.triangles) == 0:
            raise ValueError(f"{ply_path} has no triangles - is this a point cloud, not a mesh?")
        rows.append(render_orbit_frames(
            mesh, num_frames=num_frames, width=width, height=row_height,
            elevation_deg=elevation_deg, fov_deg=fov_deg,
        ))

    stacked_frames = [np.vstack(frame_group) for frame_group in zip(*rows)]
    write_video(stacked_frames, output_path, fps=fps)
    return output_path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Render 360-degree orbit videos for textured mesh PLYs (e.g. results/*_txt.ply).",
    )
    parser.add_argument("--results-dir", type=Path, default=Path("results"),
                         help="folder to search for input meshes (default: results/)")
    parser.add_argument("--pattern", default="*_txt.ply", help="glob pattern for input meshes")
    parser.add_argument("--out-dir", type=Path, default=None,
                         help="where to write videos (default: alongside each input file)")
    parser.add_argument("--num-frames", type=int, default=120)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--elevation-deg", type=float, default=20.0)
    parser.add_argument("--fov-deg", type=float, default=30.0)
    parser.add_argument("--stack", action="store_true",
                         help="combine every matched mesh into a single vertically-stacked "
                              "portrait video instead of one video per mesh")
    parser.add_argument("--stack-out", type=Path, default=None,
                         help="output path for the stacked video (default: <results-dir>/combined_360.mp4)")
    parser.add_argument("--row-height", type=int, default=640,
                         help="per-mesh row height for --stack (default: 640, so 3 meshes at the "
                              "default --width=1080 gives a standard 1080x1920 9:16 video)")
    parser.add_argument("--log-level", default="INFO", choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    args = parser.parse_args(argv)

    import logging
    logging.basicConfig(level=args.log_level, format="%(asctime)s %(levelname)-8s %(message)s", datefmt="%H:%M:%S")
    log = logging.getLogger("video_360")

    paths = sorted(args.results_dir.glob(args.pattern))
    if not paths:
        log.error("no files matching %r found under %s", args.pattern, args.results_dir)
        return 1

    if args.stack:
        out_path = args.stack_out or args.results_dir / "combined_360.mp4"
        log.info("rendering %d meshes stacked -> %s: %s", len(paths), out_path, [p.name for p in paths])
        render_stacked_360_video(
            paths, out_path, num_frames=args.num_frames, fps=args.fps,
            width=args.width, row_height=args.row_height,
            elevation_deg=args.elevation_deg, fov_deg=args.fov_deg,
        )
        log.info("wrote %s", out_path)
        return 0

    for ply_path in paths:
        out_dir = args.out_dir or ply_path.parent
        out_path = out_dir / f"{ply_path.stem}_360.mp4"
        log.info("rendering %s -> %s", ply_path, out_path)
        render_360_video(
            ply_path, out_path, num_frames=args.num_frames, fps=args.fps,
            width=args.width, height=args.height, elevation_deg=args.elevation_deg, fov_deg=args.fov_deg,
        )
        log.info("wrote %s", out_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
