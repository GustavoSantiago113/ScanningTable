"""Surface Mesh Reconstruction: Poisson surface reconstruction on Step 7's merged, registered
point-cloud, following the paper's Section 2.4.5 directly.

> Following the point-cloud registration and merging operations described above, the points
> require connecting to form a surface mesh before texturing can be applied. Poisson surface
> reconstruction was used for this stage and was chosen for its relative simplicity and
> reliability. Some care is needed to avoid loss of detail on inscribed surfaces. We found that
> an octree depth of 14 gave a good compromise, retaining the detail of the inscriptions with a
> tractable computational complexity.

Implemented via `open3d`, whose `create_from_point_cloud_poisson` wraps Kazhdan's own reference
PoissonRecon implementation - the code the paper's cited Screened Poisson paper is built on.
`pymeshlab` (MeshLab's bindings) was tried first, since the paper itself uses MeshLab again for
Step 9's texturing, but its Poisson reconstruction proved unreliable in this environment: repeated
runs of the *same* inputs and parameters varied from well under a minute to over ten minutes of
wall-clock time, and the sustained multi-threaded load repeatedly correlated with this machine's
WSL instance disconnecting outright. open3d's implementation has been consistent. The mesh this
module writes is plain PLY with vertex colours, which MeshLab reads natively - Step 9's texturing
can still use MeshLab even though this step no longer does.

One thing open3d's binding doesn't expose that pymeshlab's did: per-point confidence-weighted
Poisson (MeshLab's `pointweight`/`confidence` filter params, driven by Step 7's own confidence
metric). Step 7's confidence values are still computed and available; they just aren't fed into
reconstruction here.

**System requirements**:
- `libgomp.so.1` (GCC's OpenMP runtime) - install once: `sudo apt-get install libgomp1`.
- Import from a *native* Linux filesystem. open3d's compiled extension is large (~1GB); loading it
  from a WSL `/mnt/c`-mounted (Windows) path measured 2+ minutes, against a few seconds from ext4.
  If this project lives under `/mnt/...`, move it to somewhere like `~/ScanningTable` first.
"""

from pathlib import Path

import numpy as np
import open3d as o3d
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
    pcd: o3d.geometry.PointCloud, depth: int = 14, scale: float = 1.1, linear_fit: bool = False
) -> tuple[o3d.geometry.TriangleMesh, np.ndarray]:
    """Screened Poisson surface reconstruction. `depth=14` is the paper's own reported "good
    compromise" octree depth - deep enough to retain fine inscribed detail without intractable
    runtime. Returns (mesh, densities): densities is Poisson's own per-vertex sample-density
    estimate. `remove_isolated_points` on the input cloud (called beforehand) catches disconnected
    debris; `trim_low_density_vertices`, below, catches a different failure mode Poisson causes
    even on a clean cloud - see its own docstring.
    """
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=depth, scale=scale, linear_fit=linear_fit
    )
    return mesh, np.asarray(densities)


def trim_low_density_vertices(
    mesh: o3d.geometry.TriangleMesh, densities: np.ndarray, quantile: float = 0.02
) -> o3d.geometry.TriangleMesh:
    """Drop Poisson's own lowest-confidence vertices (the `quantile` fraction with the lowest
    `densities`) and clean up what that leaves behind.

    Poisson fits one *global* smooth implicit function over the whole cloud, so wherever the
    input point density drops (a thin structure sampled by relatively few points), the
    reconstructed surface doesn't get thin along with it - it extrapolates past the last real
    point and balloons into a rounded "blob" with no points to support it. Diagnosed on this
    replica's own merged cloud (`outputs/merged/merged.ply`, 241,860 points, one object with a
    sparsely-sampled base): the raw mesh's z-range extended ~16mm past the point cloud's own
    lowest point, and every one of those extrapolated vertices fell in the bottom ~3.5% of
    Poisson's per-vertex density (3.0-8.0, against a global median of 9.9) - exactly the region
    the input cloud itself goes sparse (hundreds of points per 2mm z-slab there, against tens of
    thousands through the rest of the object). Trimming that low-density tail at the default
    `quantile=0.02` removed the blob almost entirely there, while a genuinely thin-but-real region
    elsewhere in the same cloud (sparse, but not unsupported) kept over 98% of its vertices - low
    density flags *unsupported extrapolation*, not thinness by itself, so real thin structure
    survives even though it sits closer to the threshold than the object's bulk.

    Trimming vertices out of the middle of a mesh leaves small disconnected shell fragments along
    the cut boundary; `remove_small_mesh_components` (below) should be run afterward to clear
    those.
    """
    to_remove = densities < np.quantile(densities, quantile)
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

    `trim_low_density_vertices` cuts through the mesh wherever density dips below its threshold,
    which can strand small shell fragments along the cut - debris left over from the trim, not
    real geometry. Diagnosed on this replica's own merged cloud after trimming at the default
    quantile: 1,522 connected components, but all but ~130 of them under 10 triangles each and
    together making up under 1% of the mesh's total triangle count - the object itself is the one
    dominant component. `min_triangles=100` clears essentially all of that debris while keeping
    99.2%+ of the mesh untouched.
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
