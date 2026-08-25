"""Surface Mesh Reconstruction: Poisson surface reconstruction on Step 7's merged, registered
point-cloud, following the paper's Section 2.4.5 directly.

> Following the point-cloud registration and merging operations described above, the points
> require connecting to form a surface mesh before texturing can be applied. Poisson surface
> reconstruction was used for this stage and was chosen for its relative simplicity and
> reliability. Some care is needed to avoid loss of detail on inscribed surfaces. We found that
> an octree depth of 14 gave a good compromise, retaining the detail of the inscriptions with a
> tractable computational complexity.

Implemented via `pycolmap.poisson_meshing`, COLMAP's own binding around Kazhdan's reference
PoissonRecon implementation - the code the paper's cited Screened Poisson paper is built on.
Two other options were tried and rejected first, both on reliability grounds specific to this
machine (a WSL instance), not algorithmic ones - all three ultimately wrap the same PoissonRecon
family of code:

- `pymeshlab` (MeshLab's bindings): repeated runs of the *same* inputs/parameters varied from
  well under a minute to over ten minutes of wall-clock time, and the sustained multi-threaded
  load repeatedly correlated with this machine's WSL instance disconnecting outright.
- `open3d`'s `create_from_point_cloud_poisson`: reliable, but ~1.8x slower than pycolmap's own
  mesher at this project's own settings (128s vs. 70.6s, measured on a 250,000-point cloud at
  depth=14) - and see the module-level note below on why it was dropped in favour of pycolmap's.

**`num_threads` must stay pinned at 1.** `pycolmap.poisson_meshing` reproduces the exact same
WSL-multithreading instability documented above for pymeshlab, just via a different (also
multithreaded) C++ code path: a trivial depth-6 smoke test went from 0.9s at `num_threads=1` to
43s at `num_threads=-1` (all cores), and a depth-14/250k-point run at `num_threads=-1` was still
running after 10+ minutes (and pegging ~18 of this machine's 20 cores) before being killed.
Pinned to `num_threads=1`, the same depth-14/250k-point run finishes in ~70.6s - this is not a
minor tuning knob, it's the difference between "works" and "hangs" on this machine.

**`PoissonMeshingOptions.trim` must stay at exactly `0.0`.** COLMAP's own trim step (its
"soft-orientable" surface-of-interest filter, driven by this parameter) produced provably
corrupted output - NaN and huge-magnitude (>1000, against a true bounding box of order 100mm)
vertex positions - for *any* nonzero value tested, including deliberately tiny ones
(1e-6). This isn't a case of picking a bad absolute threshold: on a realistic mm-scale test cloud,
`trim=10.0` (the library's own default) corrupted 86% of output vertices, while `trim=1.0` on the
*same* cloud was completely clean - the "right" value is scale/density-dependent in a way this
build gives no safe way to discover in advance, and a wrong one corrupts silently rather than
erroring. `trim=0.0` was the only value verified clean across every scale tested; it also has a
side effect worth knowing about - see `trim_unsupported_vertices` below.

**Losing Poisson's own per-vertex density estimate.** `trim=0.0` disables COLMAP's trim step
entirely, and its per-vertex density/confidence field (open3d's `create_from_point_cloud_poisson`
returns this as `densities`) is *only* written to the output file when the trim step actually
runs - so getting it back would mean re-enabling the exact code path just shown to corrupt
output. `trim_unsupported_vertices` (below) replaces the density-based trim this project
previously did with open3d's own `densities` array (see git history for that version) with a
differently-sourced but conceptually equivalent signal: distance from each mesh vertex to the
nearest real input point, via a plain KD-tree over the point cloud Poisson actually saw. This
targets the same failure mode (Poisson's single global implicit function extrapolating a smooth
"blob" past the last real point wherever the input goes sparse - a thin structure, a sparsely-
sampled base) with a metric that's arguably more directly interpretable than Poisson's own
internal octree density, but is new code with its own threshold (`quantile`) that hasn't been
validated against this project's own real captures the way the previous open3d/density version
was - see that function's own docstring for what to check if trimming looks wrong on real data.

`pycolmap.poisson_meshing`'s `point_weight` doesn't give per-point confidence weighting either
(Step 7's own confidence metric still isn't fed into reconstruction here) - it's a single global
Kazhdan `--pointWeight`-style scalar, same limitation open3d's binding had.

**System requirements**: import from a *native* Linux filesystem. `open3d`'s compiled extension
(still used here for point-cloud/mesh I/O and cleanup) is large (~1GB); loading it from a WSL
`/mnt/c`-mounted (Windows) path measured 2+ minutes, against a few seconds from ext4. If this
project lives under `/mnt/...`, move it to somewhere like `~/ScanningTable` first.
"""

import tempfile
from pathlib import Path

import numpy as np
import open3d as o3d
import pycolmap
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree


def remove_isolated_points(
    points: np.ndarray,
    *arrays: np.ndarray,
    radius: float = 1.5,
    min_component_size: int = 10,
) -> tuple[np.ndarray, ...]:
    """Connected-components outlier removal: connect points within `radius` of each other, then
    keep only points belonging to a component of at least `min_component_size` members.

    Poisson fits a single *global* implicit function, so even a handful of points far from
    everything else - each still gets a plausible local normal - can pull that function's
    zero-level-set out into a sizeable, completely disconnected "bubble" surface far from any
    real geometry. Diagnosed on this replica's own merged cloud (`outputs/merged/merged.ply`):
    67,805 of 67,820 points form one connected component at `radius=1.5mm`, and the rest are
    fragments of 1-4 points each, scattered elsewhere - not a real second surface, registration
    debris. Filtering the *point cloud* for these before Poisson ever sees them avoids the bubble
    at the source, rather than trying to identify and trim the mesh it produces from them
    afterwards (which needs Poisson to have already run once, expensively, to find out).

    `points` plus any additional same-length arrays (colors, confidence, ...) are all filtered
    with the same mask; returned in the same order.
    """
    n = len(points)
    tree = cKDTree(points)
    pairs = tree.query_pairs(r=radius, output_type="ndarray")

    if len(pairs) == 0:
        rows, cols = np.array([], dtype=int), np.array([], dtype=int)
    else:
        rows = np.concatenate([pairs[:, 0], pairs[:, 1]])
        cols = np.concatenate([pairs[:, 1], pairs[:, 0]])
    graph = coo_matrix((np.ones(len(rows)), (rows, cols)), shape=(n, n))

    _, labels = connected_components(graph, directed=False)
    sizes = np.bincount(labels)
    mask = sizes[labels] >= min_component_size

    return (points[mask],) + tuple(a[mask] for a in arrays)


def build_point_cloud(points: np.ndarray, colors: np.ndarray | None = None) -> o3d.geometry.PointCloud:
    """An open3d PointCloud from `points` (n,3) and optional `colors` (n,3, 0-255 RGB)."""
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
    if colors is not None:
        pcd.colors = o3d.utility.Vector3dVector(np.clip(colors, 0, 255).astype(np.float64) / 255.0)
    return pcd


def estimate_normals(pcd: o3d.geometry.PointCloud, k: int = 16) -> None:
    """Estimate per-point normals (PCA over `k` nearest neighbours) and orient them consistently
    via Riemannian-graph propagation (Hoppe et al.) - the same algorithm MeshLab's own point-cloud
    normal filter uses, and, as there, needed because the merged cloud combines several original
    viewpoints with no single "outward" direction to assume. Poisson reconstruction needs these
    oriented normals; it cannot work from raw positions alone.
    """
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(knn=k))
    pcd.orient_normals_consistent_tangent_plane(k)


def poisson_reconstruct(
    pcd: o3d.geometry.PointCloud, depth: int = 14, point_weight: float = 1.0, num_threads: int = 1
) -> o3d.geometry.TriangleMesh:
    """Screened Poisson surface reconstruction via `pycolmap.poisson_meshing`. `depth=14` is the
    paper's own reported "good compromise" octree depth - deep enough to retain fine inscribed
    detail without intractable runtime. `point_weight` is Kazhdan's global screening/data-term
    weight (`PoissonMeshingOptions.point_weight`); it is *not* per-point confidence weighting -
    see this module's own docstring for why that capability still isn't available here.

    `num_threads` must stay at its default of 1 - see the module docstring for the measured WSL
    hang this avoids. `trim` is hardcoded to `0.0` for the same reason (any nonzero value tested
    corrupted output on this machine); this also means the returned mesh carries no density field,
    which is why cleanup here uses `trim_unsupported_vertices` (below) instead of a density
    threshold. `remove_isolated_points` on the input cloud (called beforehand) catches disconnected
    debris; `trim_unsupported_vertices` catches a different failure mode Poisson causes even on a
    clean cloud - see its own docstring.

    `pycolmap.poisson_meshing` only reads/writes PLY files, not in-memory point clouds/meshes, so
    this round-trips `pcd` through a temporary directory.
    """
    options = pycolmap.PoissonMeshingOptions()
    options.depth = depth
    options.point_weight = point_weight
    options.num_threads = num_threads
    options.color = True
    options.trim = 0.0

    with tempfile.TemporaryDirectory() as tmp:
        input_path = Path(tmp) / "input.ply"
        output_path = Path(tmp) / "output.ply"
        o3d.io.write_point_cloud(str(input_path), pcd, write_ascii=False)
        pycolmap.poisson_meshing(input_path=input_path, output_path=output_path, options=options)
        mesh = o3d.io.read_triangle_mesh(str(output_path))

    return mesh


def trim_unsupported_vertices(
    mesh: o3d.geometry.TriangleMesh, reference_points: np.ndarray, quantile: float = 0.02
) -> o3d.geometry.TriangleMesh:
    """Drop the mesh's own worst-supported vertices (the `quantile` fraction farthest from any
    real point in `reference_points` - the same cloud Poisson reconstructed from) and clean up
    what that leaves behind.

    Poisson fits one *global* smooth implicit function over the whole cloud, so wherever the
    input point density drops (a thin structure sampled by relatively few points), the
    reconstructed surface doesn't get thin along with it - it extrapolates past the last real
    point and balloons into a rounded "blob" with no points to support it. A prior version of this
    function used Poisson's own per-vertex density estimate to detect that (open3d's binding
    returns it directly); diagnosed on this replica's own merged cloud (`outputs/merged/merged.ply`,
    241,860 points, one object with a sparsely-sampled base), that approach's raw mesh z-range
    extended ~16mm past the point cloud's own lowest point, and every one of those extrapolated
    vertices fell in the bottom ~3.5% of density - trimming that tail at `quantile=0.02` removed
    the blob almost entirely there, while a genuinely thin-but-real region elsewhere in the same
    cloud kept over 98% of its vertices.

    This version targets the same failure mode with a differently-sourced signal - nearest-
    neighbour distance to `reference_points` via a plain KD-tree, since `pycolmap.poisson_meshing`
    doesn't expose a density estimate the way open3d's binding did (see this module's own
    docstring). The two metrics should behave similarly (both are low exactly where the input
    cloud's own support is thin, not just where the surface itself is genuinely thin - distance
    to the nearest real point is arguably an even more direct proxy for "unsupported" than
    Poisson's own internal octree density), but this specific metric/threshold pairing hasn't
    been run against this project's own real captures yet the way the density version was. If a
    real mesh's base looks under- or over-trimmed, check the actual distance distribution
    (`quantile` assumes the same "worst few percent" shape the density version found) before
    assuming the default `quantile=0.02` still applies.

    Trimming vertices out of the middle of a mesh leaves small disconnected shell fragments along
    the cut boundary; `remove_small_mesh_components` (below) should be run afterward to clear
    those.
    """
    tree = cKDTree(reference_points)
    distances, _ = tree.query(np.asarray(mesh.vertices), k=1, workers=-1)
    to_remove = distances > np.quantile(distances, 1.0 - quantile)
    trimmed = o3d.geometry.TriangleMesh(mesh)
    trimmed.remove_vertices_by_mask(to_remove)
    trimmed.remove_unreferenced_vertices()
    trimmed.remove_degenerate_triangles()
    trimmed.remove_duplicated_triangles()
    trimmed.remove_duplicated_vertices()
    trimmed.remove_non_manifold_edges()
    return trimmed


def remove_small_mesh_components(
    mesh: o3d.geometry.TriangleMesh, min_triangles: int = 100
) -> o3d.geometry.TriangleMesh:
    """Drop connected triangle components smaller than `min_triangles`.

    `trim_unsupported_vertices` cuts through the mesh wherever its distance-to-input-cloud metric
    exceeds its threshold, which can strand small shell fragments along the cut - debris left over
    from the trim, not real geometry. Diagnosed on this replica's own merged cloud after trimming
    at the default quantile (with the prior density-based version of that function): 1,522
    connected components, but all but ~130 of them under 10 triangles each and together making up
    under 1% of the mesh's total triangle count - the object itself is the one dominant component.
    `min_triangles=100` clears essentially all of that debris while keeping 99.2%+ of the mesh
    untouched.
    """
    triangle_clusters, cluster_n_triangles, _cluster_area = mesh.cluster_connected_triangles()
    cluster_n_triangles = np.asarray(cluster_n_triangles)
    triangle_clusters = np.asarray(triangle_clusters)
    small_clusters = cluster_n_triangles[triangle_clusters] < min_triangles

    cleaned = o3d.geometry.TriangleMesh(mesh)
    cleaned.remove_triangles_by_mask(small_clusters)
    cleaned.remove_unreferenced_vertices()
    return cleaned


def mesh_arrays(mesh: o3d.geometry.TriangleMesh) -> tuple[np.ndarray, np.ndarray, np.ndarray | None]:
    """(vertices, faces, colors) as plain numpy arrays - vertices (n,3) float, faces (m,3) int,
    colors (n,3) uint8 (0-255) or None if the mesh carries no per-vertex color.
    """
    vertices = np.asarray(mesh.vertices)
    faces = np.asarray(mesh.triangles)
    colors = np.asarray(mesh.vertex_colors) * 255.0 if mesh.has_vertex_colors() else None
    return vertices, faces, colors


def write_mesh(path: Path, mesh: o3d.geometry.TriangleMesh) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    o3d.io.write_triangle_mesh(str(path), mesh)
