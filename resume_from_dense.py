"""Resume `reconstruct.py` from Step 5's (dense reconstruction) already-computed binary output,
re-running only cropping, registration, meshing, and texturing - for when a `reconstruct.py`
run got all the way through dense reconstruction for every set (by far the most expensive
stage - PatchMatchStereo/StereoFusion, GPU-bound, tens of minutes per set) and then crashed, was
killed, or the machine went down somewhere after that (e.g. during meshing).

`reconstruct.py` deliberately keeps most of a run's intermediate state in memory only, passed
directly from one stage function to the next, rather than round-tripping through disk files (see
its own module docstring). The cost of that choice: if the *process* dies before it reaches its
own final cleanup, that in-memory state is gone - even though the actual expensive computation it
was holding the *results* of already ran to completion and is still sitting on disk, because
COLMAP's own file-based API wrote it there as a side effect of running:

  - `outputs/<set>/dense/fused` - StereoFusion's fused, coloured dense point cloud.
  - `outputs/<set>/full_triangulated` - the calibrated reconstruction (real *and* virtual
    cameras, fully triangulated) camera geometry estimation produced right before deriving
    (in-memory only) the object dense reconstruction and texturing actually use, Step 4's
    `final_recon` (virtual images stripped out).
  - `outputs/<set>/work` - the downscaled real photographs texturing samples from.

This script rebuilds `final_recon` from `full_triangulated` (virtual images are identified by
their fixed `calibration_pattern/` name prefix - `full_triangulated` itself doesn't persist the
original virtual-image list separately), reloads the fused points, and then calls
`reconstruct.py`'s own stage functions directly (`stage_cropping`, `run_registration`,
`run_meshing`, `run_texturing`, `cleanup_intermediates`) - the exact same code a fresh
`reconstruct.py` run would use from that point on, just skipping camera geometry estimation and
dense reconstruction entirely.

Usage: `python resume_from_dense.py` (from the repo root, same `--output-dir` the crashed run
used). See `--help` for the rest - it's a strict subset of `reconstruct.py`'s own options, since
only later-stage parameters are relevant here.
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path

import pycolmap

import reconstruct
from utils import colmap_calibration
from utils import dense_reconstruction as dr

log = logging.getLogger("resume_from_dense")


def discover_resumable_sets(output_dir: Path) -> list[str]:
    """Set names under `output_dir` that have both a completed dense reconstruction
    (`dense/fused`) and the calibrated camera reconstruction (`full_triangulated`) needed to
    resume from - exactly what `reconstruct.py`'s camera-geometry and dense-reconstruction
    stages leave on disk once they've actually finished for a given set.
    """
    if not output_dir.is_dir():
        raise FileNotFoundError(f"output directory not found: {output_dir}")

    names = []
    for p in sorted(output_dir.iterdir()):
        if not p.is_dir() or p.name in ("mesh", "textured"):
            continue
        fused_ok = (p / "dense" / "fused" / "points3D.bin").exists()
        triangulated_ok = (p / "full_triangulated" / "points3D.bin").exists()
        work_ok = (p / "work" / p.name).is_dir()
        if fused_ok and triangulated_ok and work_ok:
            names.append(p.name)
        elif fused_ok or triangulated_ok or work_ok:
            log.warning(
                "%s: looks like a set directory from a previous run but is incomplete "
                "(dense/fused=%s, full_triangulated=%s, work=%s) - skipping",
                p.name, fused_ok, triangulated_ok, work_ok,
            )
    return names


def load_final_recon(output_dir: Path, set_name: str) -> pycolmap.Reconstruction:
    """Rebuild Step 4's `final_recon` (calibrated real cameras, virtual images stripped) from
    `outputs/<set>/full_triangulated` - see the module docstring.
    """
    recon = pycolmap.Reconstruction(str(output_dir / set_name / "full_triangulated"))
    virtual_names = {img.name for img in recon.images.values() if img.name.startswith("calibration_pattern/")}
    if not virtual_names:
        log.warning(
            "%s: no images named 'calibration_pattern/*' found in full_triangulated - either "
            "this reconstruction predates that naming convention, or every virtual image was "
            "already stripped; proceeding without stripping anything", set_name,
        )
    return colmap_calibration.strip_virtual_images(recon, virtual_names)


def load_dense_points(output_dir: Path, set_name: str) -> tuple:
    fused = pycolmap.Reconstruction(str(output_dir / set_name / "dense" / "fused"))
    return dr.points_and_colors(fused)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Resume reconstruct.py from Step 5's already-computed dense reconstruction "
                    "(cropping onward) - skips camera geometry estimation and dense reconstruction.",
    )
    parser.add_argument("--output-dir", type=Path, default=Path("outputs"),
                         help="the SAME --output-dir the crashed/interrupted reconstruct.py run used (default: outputs/)")
    parser.add_argument("--sets", nargs="+", default=None,
                         help="restrict to these set names instead of auto-discovering every resumable one")
    parser.add_argument("--reference-set", default=None,
                         help="set used as the registration reference frame (default: the first resumable set)")
    parser.add_argument("--keep-intermediates", action="store_true",
                         help="skip the final cleanup step and leave dense/fused, full_triangulated, work, etc. in "
                              "place - also writes each set's cropped point cloud and the merged point cloud as "
                              ".ply files (see reconstruct.py's own --keep-intermediates help)")
    parser.add_argument("--log-level", default="INFO", choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    logging.basicConfig(level=args.log_level, format="%(asctime)s %(levelname)-8s %(message)s", datefmt="%H:%M:%S")

    cfg = reconstruct.Config(images_dir=Path("images"), output_dir=args.output_dir, reference_set=args.reference_set)

    set_names = args.sets if args.sets else discover_resumable_sets(cfg.output_dir)
    if not set_names:
        log.error(
            "no resumable sets found under %s (need dense/fused + full_triangulated + work per set)",
            cfg.output_dir,
        )
        return 1
    log.info("resuming %d set(s) from their dense reconstruction: %s", len(set_names), set_names)

    set_data: dict[str, dict] = {}
    cropped: dict[str, tuple] = {}
    for set_name in set_names:
        try:
            log.info("=== %s: loading dense reconstruction from disk ===", set_name)
            points, colors = load_dense_points(cfg.output_dir, set_name)
            log.info("%s: %d fused points loaded", set_name, len(points))

            log.info("=== %s: cropping ===", set_name)
            cropped_points, cropped_colors, z_lower, z_upper = reconstruct.stage_cropping(set_name, cfg, points, colors)
            if args.keep_intermediates:
                cropped_ply_path = reconstruct.write_cropped_ply(cfg, set_name, cropped_points, cropped_colors)
                log.info("%s: wrote %s (--keep-intermediates)", set_name, cropped_ply_path)

            log.info("=== %s: loading calibrated cameras from disk ===", set_name)
            final_recon = load_final_recon(cfg.output_dir, set_name)
            work_dir = cfg.output_dir / set_name / "work"
        except Exception:
            log.exception("%s: failed to resume - skipping this set", set_name)
            continue

        set_data[set_name] = dict(final_recon=final_recon, work_dir=work_dir)
        cropped[set_name] = (cropped_points, cropped_colors, z_lower, z_upper)

    if not cropped:
        log.error("every set failed to resume - nothing to reconstruct")
        return 1
    if len(cropped) < len(set_names):
        log.warning("only %d/%d sets resumed successfully: %s", len(cropped), len(set_names), sorted(cropped))

    log.info("=== point cloud registration ===")
    merged_points, merged_colors, merged_confidence, transforms = reconstruct.run_registration(cropped, cfg)
    if args.keep_intermediates:
        merged_ply_path = reconstruct.write_merged_ply(cfg, merged_points, merged_colors)
        log.info("wrote %s (--keep-intermediates)", merged_ply_path)

    log.info("=== meshing ===")
    tri_mesh, mesh_path = reconstruct.run_meshing(merged_points, merged_colors, merged_confidence, cfg)

    log.info("=== texturing ===")
    textured_path = reconstruct.run_texturing(tri_mesh, set_data, transforms, cfg)

    if args.keep_intermediates:
        log.info("--keep-intermediates set - leaving all intermediate files in place")
    else:
        log.info("=== cleanup ===")
        reconstruct.cleanup_intermediates(cfg, mesh_path, textured_path)

    log.info("done: %s, %s", cfg.output_dir / "mesh" / "mesh.ply", cfg.output_dir / "textured" / "textured.ply")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
