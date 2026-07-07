"""Point-Cloud Registration: bring Step 6's N per-set cropped point-clouds into one common
frame and combine them, following the paper's Section 2.4.4 in full (coarse alignment, fine
alignment, and the confidence-weighted merge that section also describes):

  1. **Coarse alignment**, paper spec: Super4PCS [Mellado et al. 2014]. Substituted here by a
     from-scratch RANSAC search over congruent point triples (matching pairwise distances,
     scored by largest-common-pointset overlap) - the same family of algorithm as (Super)4PCS
     (affine/rigid-invariant local-basis matching + global consensus scoring, no initial pose
     needed), implemented in plain numpy/scipy rather than Mellado et al.'s specific coplanar
     4-point/smart-indexing formulation, since neither 4PCS nor Super4PCS has a maintained pip
     package or Python binding to depend on instead. `coarse_align` finds the basis; the smart
     indexing that makes Super4PCS fast (vs. plain 4PCS) only affects runtime, not the result.
  2. **Fine alignment**: a Weighted ICP (`weighted_icp`), using the paper's per-point confidence
     metric (`compute_confidence` - surface-normal orientation combined with height above the
     crop's own base) to weight each correspondence, so well-lit, precisely-reconstructed points
     dominate the fit over grazing-angle, poorly-lit ones.
  3. **Confidence-weighted merge** (`merge_point_clouds`, the paper's Equation 4): once aligned,
     overlapping close point pairs across clouds are interpolated into one point rather than kept
     as two redundant, potentially conflicting ones; points unique to one cloud are kept as-is.

One thing Step 6's cropped output doesn't carry that this confidence metric needs: **per-point
surface normals**. `pycolmap`'s `stereo_fusion` returns a `Reconstruction` object, whose
`Point3D` has no normal field (unlike COLMAP's native `fused.ply`, which does) - so normals here
are instead estimated locally via PCA over each point's nearest neighbours (`estimate_normals`),
oriented outward from the cloud's own centroid. This is a standard substitute for the
photo-consistency-derived normals PatchMatchStereo/StereoFusion would otherwise have produced,
not a paper-specified step.
"""

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


def read_ply(path: Path) -> tuple[np.ndarray, np.ndarray | None]:
    """Read a binary-little-endian PLY of the shape written by `write_ply` below (and by
    dense_reconstruction.write_ply / cropping.write_ply): float32 x/y/z, optional uchar
    red/green/blue.
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


def apply_transform(points: np.ndarray, R: np.ndarray, t: np.ndarray) -> np.ndarray:
    return points @ R.T + t


def estimate_normals(points: np.ndarray, k: int = 16) -> np.ndarray:
    """PCA-based local surface normal estimate: the eigenvector of least variance among each
    point's `k` nearest neighbours, oriented outward from the cloud's own centroid (a reasonable
    heuristic for a single-viewpoint partial scan of a roughly star-convex object).
    """
    tree = cKDTree(points)
    _, idx = tree.query(points, k=min(k + 1, len(points)), workers=-1)
    neighbors = points[idx]  # (n, k+1, 3)
    centered = neighbors - neighbors.mean(axis=1, keepdims=True)
    cov = np.einsum("nki,nkj->nij", centered, centered)
    _, eigvecs = np.linalg.eigh(cov)  # ascending eigenvalue order
    normals = eigvecs[:, :, 0]  # eigenvector of the smallest eigenvalue

    centroid = points.mean(axis=0)
    outward = points - centroid
    flip = np.einsum("ni,ni->n", normals, outward) < 0
    normals[flip] *= -1
    return normals


def compute_confidence(
    points: np.ndarray, normals: np.ndarray, z_lower_limit: float, lam: float = 1.0
) -> np.ndarray:
    """The paper's Equation 2: c(p) = 1/2 (n_z + 1)(1 - exp(-z/lambda)), where n_z is the
    point's (outward) surface normal z-component and z is height above `z_lower_limit` (Step 6's
    own crop height, clipped at 0 - points can't be below the crop they came from). Low near the
    crop base (shadowed, poorly reconstructed) and on downward- or side-facing surfaces
    (grazing-angle, poorly lit); high on upward-facing surfaces well above the base.
    """
    nz = normals[:, 2]
    z_rel = np.clip(points[:, 2] - z_lower_limit, a_min=0, a_max=None)
    return 0.5 * (nz + 1) * (1 - np.exp(-z_rel / lam))


def weighted_kabsch(
    P: np.ndarray, Q: np.ndarray, weights: np.ndarray | None = None
) -> tuple[np.ndarray, np.ndarray]:
    """The optimal rigid transform (R, t) minimising sum_i w_i |R@P_i + t - Q_i|^2, for
    corresponding point sets P and Q (each (n, 3)). Closed-form via SVD (Kabsch/Umeyama).
    """
    if weights is None:
        weights = np.ones(len(P))
    weights = weights / weights.sum()

    p_mean = weights @ P
    q_mean = weights @ Q
    P_c = P - p_mean
    Q_c = Q - q_mean

    H = (weights[:, None] * P_c).T @ Q_c
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    t = q_mean - R @ p_mean
    return R, t


@dataclass
class ICPStep:
    iteration: int
    num_correspondences: int
    mean_weighted_error: float


def weighted_icp(
    source_points: np.ndarray,
    source_confidence: np.ndarray,
    target_points: np.ndarray,
    target_confidence: np.ndarray,
    init_R: np.ndarray | None = None,
    init_t: np.ndarray | None = None,
    max_iterations: int = 50,
    max_correspondence_distance: float = 5.0,
    tolerance: float = 1e-6,
) -> tuple[np.ndarray, np.ndarray, list[ICPStep]]:
    """The paper's Weighted ICP (Equation 3): standard ICP, except each nearest-neighbour
    correspondence's contribution to the rigid-transform fit is weighted by the product of its
    two points' confidence values, so precisely-reconstructed points dominate the alignment.
    Returns (R, t, history) - the total transform mapping `source_points` onto `target_points`,
    and per-iteration diagnostics.
    """
    R = np.eye(3) if init_R is None else init_R.copy()
    t = np.zeros(3) if init_t is None else init_t.copy()
    target_tree = cKDTree(target_points)
    history: list[ICPStep] = []

    for iteration in range(max_iterations):
        current = apply_transform(source_points, R, t)
        dist, idx = target_tree.query(current, k=1, workers=-1)
        mask = dist < max_correspondence_distance
        if mask.sum() < 3:
            break

        matched_source = current[mask]
        matched_target = target_points[idx[mask]]
        weights = source_confidence[mask] * target_confidence[idx[mask]]
        if weights.sum() <= 0:
            weights = np.ones(mask.sum())

        R_inc, t_inc = weighted_kabsch(matched_source, matched_target, weights)
        R = R_inc @ R
        t = R_inc @ t + t_inc

        refit = apply_transform(matched_source, R_inc, t_inc)
        mean_err = float(np.average(np.linalg.norm(matched_target - refit, axis=1), weights=weights))
        history.append(ICPStep(iteration, int(mask.sum()), mean_err))

        if iteration > 0 and abs(history[-2].mean_weighted_error - mean_err) < tolerance:
            break

    return R, t, history


@dataclass
class CoarseAlignmentResult:
    R: np.ndarray
    t: np.ndarray
    score: float
    num_candidates_evaluated: int


def coarse_align(
    source_points: np.ndarray,
    target_points: np.ndarray,
    n_search_points: int = 500,
    n_trials: int = 300,
    max_candidates_per_trial: int = 20,
    max_total_candidates: int = 4000,
    distance_tolerance: float = 1.5,
    overlap_threshold: float = 2.0,
    min_base_spread: float = 10.0,
    n_score_points: int = 1000,
    seed: int = 0,
) -> CoarseAlignmentResult:
    """RANSAC search for the rigid transform aligning `source_points` onto `target_points`,
    with no initial guess: repeatedly picks a random, well-spread point triple from a `source`
    working sample, finds all congruent triples in a `target` working sample (matching all three
    pairwise distances within `distance_tolerance` - the rigid-invariant matching (Super)4PCS is
    also built on), fits the rigid transform for each candidate via `weighted_kabsch`, and scores
    it by the fraction of a larger source sample landing within `overlap_threshold` of some
    target point after transforming (largest-common-pointset, the same acceptance criterion
    (Super)4PCS uses). The best-scoring transform over all trials is returned.
    """
    rng = np.random.default_rng(seed)

    src_idx = rng.choice(len(source_points), size=min(n_search_points, len(source_points)), replace=False)
    tgt_idx = rng.choice(len(target_points), size=min(n_search_points, len(target_points)), replace=False)
    S = source_points[src_idx]
    T = target_points[tgt_idx]
    D_T = np.linalg.norm(T[:, None, :] - T[None, :, :], axis=-1)

    target_tree = cKDTree(target_points)
    score_idx = rng.choice(len(source_points), size=min(n_score_points, len(source_points)), replace=False)
    score_points = source_points[score_idx]

    best = CoarseAlignmentResult(R=np.eye(3), t=np.zeros(3), score=-1.0, num_candidates_evaluated=0)
    num_evaluated = 0

    for _ in range(n_trials):
        if num_evaluated >= max_total_candidates:
            break

        i, j, k = rng.choice(len(S), size=3, replace=False)
        a, b, c = S[i], S[j], S[k]
        d_ab, d_ac, d_bc = np.linalg.norm(a - b), np.linalg.norm(a - c), np.linalg.norm(b - c)
        if min(d_ab, d_ac, d_bc) < min_base_spread:
            continue

        pq_candidates = np.argwhere(np.abs(D_T - d_ab) < distance_tolerance)
        pq_candidates = pq_candidates[pq_candidates[:, 0] != pq_candidates[:, 1]]
        if len(pq_candidates) == 0:
            continue
        if len(pq_candidates) > max_candidates_per_trial:
            pq_candidates = pq_candidates[
                rng.choice(len(pq_candidates), size=max_candidates_per_trial, replace=False)
            ]

        for p_i, q_i in pq_candidates:
            if num_evaluated >= max_total_candidates:
                break

            p, q = T[p_i], T[q_i]
            dist_from_p = np.linalg.norm(T - p, axis=1)
            dist_from_q = np.linalg.norm(T - q, axis=1)
            r_candidates = np.nonzero(
                (np.abs(dist_from_p - d_ac) < distance_tolerance)
                & (np.abs(dist_from_q - d_bc) < distance_tolerance)
            )[0]

            for r_i in r_candidates:
                if num_evaluated >= max_total_candidates:
                    break
                r = T[r_i]

                R_cand, t_cand = weighted_kabsch(np.stack([a, b, c]), np.stack([p, q, r]))
                transformed = apply_transform(score_points, R_cand, t_cand)
                dist, _ = target_tree.query(transformed, k=1, workers=-1)
                score = float(np.mean(dist < overlap_threshold))
                num_evaluated += 1

                if score > best.score:
                    best = CoarseAlignmentResult(R_cand, t_cand, score, num_evaluated)

    best.num_candidates_evaluated = num_evaluated
    return best


def merge_point_clouds(
    base_points: np.ndarray,
    base_colors: np.ndarray,
    base_confidence: np.ndarray,
    add_points: np.ndarray,
    add_colors: np.ndarray,
    add_confidence: np.ndarray,
    merge_radius: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """The paper's Equation 4: combine two already-registered point-clouds into one. Points in
    `add_*` within `merge_radius` of some `base_*` point are treated as redundant observations of
    the same surface and interpolated into one point, confidence-weighted:
    r = (c(p) A p + c(q) q) / (c(p) + c(q)) - rather than keeping both (the paper's "eliminating
    unreliable points where a better close-by alternative can be found in another partial
    model"). Where several `add` points match the same `base` point, only the most confident is
    kept for the interpolation (the rest are near-duplicates of it and carry no new information).
    Points with no close match in the other cloud are kept as-is - genuinely new geometry.
    """
    if len(base_points) == 0:
        return add_points, add_colors, add_confidence
    if len(add_points) == 0:
        return base_points, base_colors, base_confidence

    base_tree = cKDTree(base_points)
    dist, matched_base_idx = base_tree.query(add_points, k=1, workers=-1)
    is_overlap = dist <= merge_radius

    unique_add = ~is_overlap

    merged_points_list = [base_points.copy()]
    merged_colors_list = [base_colors.copy()]
    merged_confidence_list = [base_confidence.copy()]

    overlap_add_idx = np.nonzero(is_overlap)[0]
    if len(overlap_add_idx) > 0:
        groups: dict[int, list[int]] = {}
        for add_i in overlap_add_idx:
            groups.setdefault(int(matched_base_idx[add_i]), []).append(int(add_i))

        for base_i, add_group in groups.items():
            best_add_i = max(add_group, key=lambda i: add_confidence[i])
            cp = base_confidence[base_i]
            cq = add_confidence[best_add_i]
            total_c = cp + cq
            if total_c <= 0:
                weight_p, weight_q = 0.5, 0.5
                total_c = 1.0
            else:
                weight_p, weight_q = cp / total_c, cq / total_c

            merged_points_list[0][base_i] = (
                weight_p * base_points[base_i] + weight_q * add_points[best_add_i]
            )
            merged_colors_list[0][base_i] = (
                weight_p * base_colors[base_i] + weight_q * add_colors[best_add_i]
            )
            merged_confidence_list[0][base_i] = max(cp, cq)

    if unique_add.any():
        merged_points_list.append(add_points[unique_add])
        merged_colors_list.append(add_colors[unique_add])
        merged_confidence_list.append(add_confidence[unique_add])

    return (
        np.concatenate(merged_points_list, axis=0),
        np.concatenate(merged_colors_list, axis=0),
        np.concatenate(merged_confidence_list, axis=0),
    )
