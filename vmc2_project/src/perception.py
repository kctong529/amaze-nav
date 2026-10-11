"""ROS-independent, observational LiDAR analysis for an individual directed maze edge.

All input coordinates must belong to the same fixed XY frame. Observed gaps are
NOT verified free corridors and must not be used directly as collision guarantees.
"""
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional

import numpy as np

Point = tuple[float, float]


class Passability(Enum):
    CLEAR = auto()
    UNKNOWN = auto()
    BLOCKED = auto()


@dataclass(frozen=True)
class Gap:
    left: float
    right: float

    @property
    def width(self) -> float:
        return self.right - self.left

    @property
    def midpoint(self) -> float:
        return (self.left + self.right) / 2.0


@dataclass(frozen=True)
class SliceObservation:
    s: float
    point_count: int
    obstacle_intervals: tuple[tuple[float, float], ...]
    gaps: tuple[Gap, ...]


@dataclass(frozen=True)
class EdgeObservation:
    start: Point
    end: Point
    obstacle_points_st: np.ndarray
    slices: tuple[SliceObservation, ...]

    @property
    def length(self) -> float:
        return float(np.linalg.norm(np.asarray(self.end) - np.asarray(self.start)))


@dataclass(frozen=True)
class BottleneckCandidate:
    s: float
    widest_gap: float
    required_width: float
    point_count: int
    obstacle_count: int


def detect_bottlenecks(
    observation: EdgeObservation,
    *,
    robot_radius: float = 0.089,
    safety_margin: float = 0.04,
    min_clusters: int = 2,
) -> list[BottleneckCandidate]:
    """
    Find cross-sections whose observed internal gaps are
    all narrower than the required robot clearance.

    A result is evidence of a possible bottleneck, NOT
    proof that the entire edge is blocked.
    """
    required_width = 2 * (robot_radius + safety_margin)

    bottlenecks = []

    for section in observation.slices:
        if len(section.obstacle_intervals) < min_clusters:
            continue

        if not section.gaps:
            continue

        widest = max(gap.width for gap in section.gaps)

        if widest < required_width:
            bottlenecks.append(
                BottleneckCandidate(
                    s=section.s,
                    widest_gap=widest,
                    required_width=required_width,
                    point_count=section.point_count,
                    obstacle_count=len(
                        section.obstacle_intervals
                    ),
                )
            )

    return bottlenecks


@dataclass(frozen=True)
class PassabilityResult:
    status: Passability
    reason: str
    minimum_observed_width: Optional[float] = None


@dataclass(frozen=True)
class TargetResult:
    target: Optional[Point]
    reason: str
    clearance_verified: bool = False


def _edge_basis(start: Point, end: Point):
    a = np.asarray(start, dtype=float)
    b = np.asarray(end, dtype=float)
    if a.shape != (2,) or b.shape != (2,) or not np.isfinite(a).all() or not np.isfinite(b).all():
        raise ValueError("Edge endpoints must be finite 2D coordinates")
    delta = b - a
    length = float(np.linalg.norm(delta))
    if length <= 0:
        raise ValueError("Edge endpoints must be distinct")
    u = delta / length
    n = np.array([-u[1], u[0]])
    return a, u, n, length


def points_to_edge_frame(points_world: np.ndarray, start: Point, end: Point) -> np.ndarray:
    """Return (s,t) where s is forward along A->B and +t is its left side."""
    a, u, n, _ = _edge_basis(start, end)
    points = np.asarray(points_world, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
        raise ValueError("points_world must be a finite Nx2 array")
    delta = points - a
    return np.column_stack((delta @ u, delta @ n))


def edge_to_world(s: float, t: float, start: Point, end: Point) -> Point:
    """Convert edge-frame (s,t) coordinates to the input edge's fixed XY frame."""
    a, u, n, _ = _edge_basis(start, end)
    if not np.isfinite([s, t]).all():
        raise ValueError("s and t must be finite")
    p = a + s * u + t * n
    return float(p[0]), float(p[1])


def extract_slice(edge_points: np.ndarray, s0: float, thickness: float) -> np.ndarray:
    if thickness <= 0 or not np.isfinite([s0, thickness]).all():
        raise ValueError("Slice position and thickness must be finite; thickness must be positive")
    points = np.asarray(edge_points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError("edge_points must be Nx2")
    return points[np.abs(points[:, 0] - s0) <= thickness / 2.0]


def cluster_lateral_points(slice_points: np.ndarray, max_gap: float = 0.05,
                           min_points: int = 3) -> list[tuple[float, float]]:
    """Experimentally cluster sorted lateral returns; NOT physical obstacle extents."""
    if max_gap <= 0 or not np.isfinite(max_gap) or min_points < 1:
        raise ValueError("Invalid clustering parameters")
    points = np.asarray(slice_points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
        raise ValueError("slice_points must be a finite Nx2 array")
    lateral = np.sort(points[:, 1])
    if not len(lateral):
        return []
    breaks = np.flatnonzero(np.diff(lateral) > max_gap) + 1
    chunks = np.split(lateral, breaks)
    return [(float(chunk[0]), float(chunk[-1])) for chunk in chunks if len(chunk) >= min_points]


def find_gaps(intervals: list[tuple[float, float]]) -> list[Gap]:
    """Return candidate openings between observed clusters only (no external free space)."""
    ordered = sorted(intervals, key=lambda x: x[0])
    gaps = []
    for previous, following in zip(ordered, ordered[1:]):
        left, right = previous[1], following[0]
        if right > left:
            gaps.append(Gap(left=left, right=right))
    return gaps


def analyze_edge(points_world: np.ndarray, edge_start: Point, edge_end: Point, *,
                 slice_step: float = 0.10, slice_thickness: float = 0.10,
                 cluster_max_gap: float = 0.05, cluster_min_points: int = 3,
                 lateral_range: float = 1.0) -> EdgeObservation:
    """Analyze current obstacle returns around a directed edge.

    No beam ray-tracing, visibility, trajectory, or footprint validation is done.
    Do not interpret absent clusters/gaps as CLEAR or BLOCKED.
    """
    _, _, _, length = _edge_basis(edge_start, edge_end)
    if (not np.isfinite([slice_step, slice_thickness, lateral_range]).all()
            or min(slice_step, slice_thickness, lateral_range) <= 0):
        raise ValueError("Slice spacing, thickness and lateral range must be positive")
    points_st = points_to_edge_frame(points_world, edge_start, edge_end)
    near = points_st[(points_st[:, 0] >= -0.2)
                     & (points_st[:, 0] <= length + 0.2)
                     & (np.abs(points_st[:, 1]) <= lateral_range)]
    slices = []
    for s0 in np.arange(slice_step / 2.0, length, slice_step):
        selected = extract_slice(near, float(s0), slice_thickness)
        intervals = cluster_lateral_points(selected, cluster_max_gap, cluster_min_points)
        slices.append(SliceObservation(
            s=float(s0), point_count=len(selected),
            obstacle_intervals=tuple(intervals), gaps=tuple(find_gaps(intervals)),
        ))
    return EdgeObservation(
        start=tuple(map(float, edge_start)), end=tuple(map(float, edge_end)),
        obstacle_points_st=near.copy(), slices=tuple(slices),
    )


def _has_blocking_point_barrier(
    observation: EdgeObservation,
    corridor_bounds: tuple[float, float],
    clearance_radius: float,
) -> bool:
    """Test whether inflated measured obstacle points connect both corridor sides.

    Each measured obstacle point excludes a disk of radius clearance_radius
    for the robot CENTER. A connected union of these disks spanning the two
    center-accessible corridor boundaries separates start from end.

    Preconditions: corridor_bounds are trustworthy, fixed, impassable lateral
    boundaries in the edge frame, spanning this whole edge; points are genuine
    obstacle returns in the same frame. No inferred boundaries are accepted.
    """
    low, high = corridor_bounds
    center_low = low + clearance_radius
    center_high = high - clearance_radius
    if center_low >= center_high:
        # A corridor narrower than the robot center's required clearance
        # cannot carry the robot, assuming both supplied walls are real.
        return True

    pts = observation.obstacle_points_st
    # Use only disks fully within the longitudinal interior. This avoids
    # claiming a separator due to obstacles past either end of the edge.
    interior = pts[
        (pts[:, 0] >= clearance_radius)
        & (pts[:, 0] <= observation.length - clearance_radius)
    ]
    if len(interior) == 0:
        return False

    n = len(interior)
    # Union-find across overlapping inflated point disks and the two walls.
    left, right = n, n + 1
    parent = list(range(n + 2))

    def root(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def join(x, y):
        parent[root(x)] = root(y)

    radius2 = (2.0 * clearance_radius) ** 2
    for i, (s, t) in enumerate(interior):
        if t - clearance_radius <= center_low:
            join(i, left)
        if t + clearance_radius >= center_high:
            join(i, right)
        for j in range(i):
            ds = s - interior[j, 0]
            dt = t - interior[j, 1]
            if ds * ds + dt * dt <= radius2:
                join(i, j)
        if root(left) == root(right):
            return True
    return False


def assess_passability(
    observation: EdgeObservation,
    robot_position: Point,
    *,
    robot_radius: float = 0.089,
    safety_margin: float = 0.04,
    corridor_bounds: Optional[tuple[float, float]] = None,
) -> PassabilityResult:
    """Early rejection only. Never concludes CLEAR from sparse LiDAR returns.

    corridor_bounds are trusted *physical* lateral wall coordinates (t_min,
    t_max), in the edge frame. They must enclose the edge over its full
    length. Do not substitute min/max observed LiDAR points. Without these
    bounds, a narrow internal gap is a warning, not a proof of blockage.
    """
    if not np.isfinite([robot_radius, safety_margin]).all() or robot_radius <= 0 or safety_margin < 0:
        raise ValueError("Invalid robot radius or safety margin")
    robot_position = np.asarray(robot_position, dtype=float)
    if robot_position.shape != (2,) or not np.isfinite(robot_position).all():
        raise ValueError("robot_position must be a finite XY point")

    clearance = robot_radius + safety_margin
    suspects = detect_bottlenecks(
        observation, robot_radius=robot_radius, safety_margin=safety_margin
    )
    minimum = min((b.widest_gap for b in suspects), default=None)

    if corridor_bounds is not None:
        bounds = np.asarray(corridor_bounds, dtype=float)
        if bounds.shape != (2,) or not np.isfinite(bounds).all() or bounds[0] >= bounds[1]:
            raise ValueError("corridor_bounds must be (t_min, t_max), increasing and finite")
        if _has_blocking_point_barrier(observation, (float(bounds[0]), float(bounds[1])), clearance):
            return PassabilityResult(
                Passability.BLOCKED,
                "Inflated observed obstacle returns form a continuous barrier "
                "between supplied trusted corridor boundaries",
                minimum,
            )

    if suspects:
        return PassabilityResult(
            Passability.UNKNOWN,
            f"{len(suspects)} possible bottleneck slice(s); "
            "no certified full-corridor barrier",
            minimum,
        )
    return PassabilityResult(
        Passability.UNKNOWN,
        "No verified blockage; LiDAR-only analysis cannot establish CLEAR",
    )


@dataclass(frozen=True)
class GapCandidate:
    s: float
    t: float
    width: float
    slice_s: float


@dataclass(frozen=True)
class TrajectoryResult:
    feasible: bool
    reason: str
    path_st: tuple[Point, ...] = ()
    min_observed_clearance: Optional[float] = None
    max_curvature: Optional[float] = None
    candidate: Optional[GapCandidate] = None


def find_feasible_gap_candidates(
    observation: EdgeObservation, *,
    robot_radius: float = 0.089,
    safety_margin: float = 0.04,
    samples_per_gap: int = 3,
) -> list[GapCandidate]:
    """Propose center positions strictly inside observed internal openings.

    Candidate openings have not been verified as free space. No openings
    outside pairs of retained clusters are inferred.
    """
    clearance = robot_radius + safety_margin
    if not np.isfinite([robot_radius, safety_margin]).all() or robot_radius <= 0 or safety_margin < 0:
        raise ValueError("Invalid robot clearance")
    if samples_per_gap < 1:
        raise ValueError("samples_per_gap must be positive")
    result = []
    for section in observation.slices:
        for gap in section.gaps:
            left = gap.left + clearance
            right = gap.right - clearance
            if right <= left:
                continue
            # Sample center and off-center options without touching the limit.
            fractions = [0.5] if samples_per_gap == 1 else np.linspace(0.15, 0.85, samples_per_gap)
            for fraction in fractions:
                result.append(GapCandidate(
                    s=section.s,
                    t=float(left + fraction * (right - left)),
                    width=gap.width,
                    slice_s=section.s,
                ))
    # Prefer restrictive (but navigable) gaps: large gaps offer more freedom.
    # Within the same gap, candidates are evaluated in their original order.
    return result


def evaluate_trajectory(
    observation: EdgeObservation,
    robot_position: Point,
    candidate: GapCandidate,
    *,
    robot_radius: float = 0.089,
    safety_margin: float = 0.04,
    sample_spacing: float = 0.025,
    max_curvature: float = 5.0,
) -> TrajectoryResult:
    """Evaluate a quadratic Bezier passing through a selected gap center.

    Collision checking uses distance to OBSERVED returns only. A feasible
    result is a *candidate*, not proof that occluded space is free.
    """
    if not np.isfinite([robot_radius, safety_margin, sample_spacing, max_curvature]).all() or robot_radius <= 0 or safety_margin < 0 or sample_spacing <= 0 or max_curvature <= 0:
        raise ValueError("Invalid trajectory parameters")
    start = points_to_edge_frame(np.asarray([robot_position], dtype=float), observation.start, observation.end)[0]
    end = np.array([observation.length, 0.0])
    gap = np.array([candidate.s, candidate.t])
    direction = end[0] - start[0]
    if direction <= 0.05 or gap[0] <= start[0] + 0.05 or gap[0] >= end[0] - 0.05:
        return TrajectoryResult(False, "Gap is not between the robot and edge destination", candidate=candidate)
    u = float((gap[0] - start[0]) / direction)
    p1 = (gap - (1 - u)**2 * start - u**2 * end) / (2 * u * (1 - u))
    n_samples = max(24, int(np.ceil(2 * np.linalg.norm(end-start) / sample_spacing)))
    ts = np.linspace(0, 1, n_samples + 1)
    points = ((1-ts)**2)[:,None]*start + (2*(1-ts)*ts)[:,None]*p1 + (ts**2)[:,None]*end
    path_st = tuple((float(x),float(y)) for x,y in points)
    # Disallow backward travel along the edge or large lateral excursions.
    if np.any(np.diff(points[:,0]) <= 0):
        return TrajectoryResult(False, "Trajectory reverses along the edge", path_st=path_st, candidate=candidate)
    if np.max(np.abs(points[:,1])) > 1.0:
        return TrajectoryResult(False, "Trajectory leaves the 1 m analysis band", path_st=path_st, candidate=candidate)
    derivatives = 2 * ((1-ts)[:,None]*(p1-start) + ts[:,None]*(end-p1))
    second = 2*(end - 2*p1 + start)
    speed2 = np.sum(derivatives**2, axis=1)
    if np.any(speed2 < 1e-8):
        return TrajectoryResult(False, "Degenerate curve", path_st=path_st, candidate=candidate)
    curvature = np.abs(derivatives[:,0]*second[1] - derivatives[:,1]*second[0]) / np.maximum(speed2, 1e-8)**1.5
    worst = float(np.max(curvature))
    if worst > max_curvature:
        return TrajectoryResult(False, "Excessive curvature", path_st=path_st, max_curvature=worst, candidate=candidate)
    obstacles = observation.obstacle_points_st
    if len(obstacles) == 0:
        return TrajectoryResult(False, "No measured obstacles; cannot validate this gap", path_st=path_st, candidate=candidate)
    # Chunked nearest-neighbor distances avoid allocating a massive matrix.
    min_d2 = float('inf')
    for chunk in np.array_split(points, max(1, int(np.ceil(len(points)/128)))):
        d2 = np.sum((chunk[:,None,:] - obstacles[None,:,:])**2, axis=2)
        min_d2 = min(min_d2, float(np.min(d2)))
    measured_clearance = float(np.sqrt(min_d2))
    if measured_clearance < robot_radius + safety_margin:
        return TrajectoryResult(False, "Too close to measured returns", path_st, measured_clearance, worst, candidate)
    return TrajectoryResult(True, "Sampled curve avoids observed returns (unknown space not checked)", path_st, measured_clearance, worst, candidate)


def plan_candidate_trajectory(
    observation: EdgeObservation,
    robot_position: Point,
    *,
    robot_radius: float = 0.089,
    safety_margin: float = 0.04,
    max_curvature: float = 5.0,
) -> TrajectoryResult:
    """Find a sampled candidate trajectory through an observed gap."""

    options = find_feasible_gap_candidates(
        observation,
        robot_radius=robot_radius,
        safety_margin=safety_margin,
    )

    if not options:
        return TrajectoryResult(
            False,
            "No measured internal gap wide enough",
        )

    valid = []
    rejected = []

    for option in options:
        trial = evaluate_trajectory(
            observation,
            robot_position,
            option,
            robot_radius=robot_radius,
            safety_margin=safety_margin,
            max_curvature=max_curvature,
        )

        if trial.feasible:
            valid.append(trial)
        else:
            rejected.append(trial.reason)

    if not valid:
        from collections import Counter

        counts = Counter(rejected)
        details = "; ".join(
            f"{reason}: {count}"
            for reason, count in counts.items()
        )

        return TrajectoryResult(
            False,
            (
                f"No feasible sampled curve among "
                f"{len(options)} gap targets. "
                f"Rejections: {details}"
            ),
        )

    return max(
        valid,
        key=lambda r:
            r.min_observed_clearance
            - 0.005 * r.max_curvature,
    )


def choose_avoidance_target(
    observation: EdgeObservation,
    robot_position: Point,
    nominal_target: Point,
    *,
    robot_radius: float = 0.089,
    safety_margin: float = 0.04,
    lookahead_distance: float = 0.30,
) -> TargetResult:
    """Select a world-frame lookahead from a sampled candidate trajectory.

    `nominal_target` must be the observation's edge destination for v0.3.
    Does not claim `clearance_verified`: occlusion/unknown space is untested.
    """
    if not np.allclose(nominal_target, observation.end, atol=1e-3):
        return TargetResult(None, "v0.3 only supports the edge endpoint as nominal_target")
    if lookahead_distance <= 0 or not np.isfinite(lookahead_distance):
        raise ValueError("lookahead_distance must be positive")
    trial = plan_candidate_trajectory(
        observation, robot_position, robot_radius=robot_radius,
        safety_margin=safety_margin,
    )
    if not trial.feasible:
        return TargetResult(None, trial.reason)
    path = np.asarray(trial.path_st)
    distances = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    index = min(int(np.searchsorted(distances, lookahead_distance)), len(path)-1)
    s,t = path[index]
    target = edge_to_world(s,t, observation.start, observation.end)
    return TargetResult(target, "Lookahead on candidate curve; only observed-return clearance checked", False)
