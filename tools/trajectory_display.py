"""Pure NumPy helpers for selecting representative trajectory visualizations."""
from collections import defaultdict
import numpy as np


def _resample(path, samples=20):
    """Uniformly sample a polyline by normalized cumulative arc length."""
    pts = np.asarray(path, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 2 or len(pts) < 2:
        raise ValueError("Trajectory path must contain at least two 2D points")
    arclength = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]
    if arclength[-1] < 1e-10:
        return np.repeat(pts[:1], samples, axis=0)
    unique = np.r_[True, np.diff(arclength) > 1e-10]
    return np.column_stack([
        np.interp(np.linspace(0, arclength[-1], samples), arclength[unique], pts[unique, axis])
        for axis in range(2)
    ])


def select_representatives(trajectories, limit):
    """Choose a visually diverse subset; does not mutate or score planner results.

    The first trajectory is retained (planner rank), then farthest-first
    sampling picks curves least similar to the already selected examples.
    """
    paths = [trial for trial in trajectories if len(trial.path_st) >= 2]
    if limit <= 0 or not paths:
        return []
    if len(paths) <= limit:
        return paths
    coordinates = np.stack([_resample(t.path_st) for t in paths])
    chosen = [0]
    min_dist = np.linalg.norm(coordinates - coordinates[0], axis=2).mean(axis=1)
    min_dist[0] = -np.inf
    while len(chosen) < limit:
        index = int(np.argmax(min_dist))
        chosen.append(index)
        distance = np.linalg.norm(coordinates - coordinates[index], axis=2).mean(axis=1)
        min_dist = np.minimum(min_dist, distance)
        min_dist[chosen] = -np.inf
    return [paths[i] for i in chosen]


def group_rejections(trajectories, *, per_reason=5, include_other=False):
    """Sample clearance and curvature failures independently for display."""
    groups = defaultdict(list)
    for trial in trajectories:
        if len(trial.path_st) < 2:
            continue
        if trial.reason == "Too close to measured returns":
            group = "clearance"
        elif trial.reason == "Excessive curvature":
            group = "curvature"
        else:
            group = "other"
        groups[group].append(trial)
    names = ("clearance", "curvature", "other") if include_other else ("clearance", "curvature")
    return [item for name in names for item in select_representatives(groups[name], per_reason)]
