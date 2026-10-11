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
    robot_yaw: Optional[float] = None,
    max_heading_error: float = np.deg2rad(30.0),
    _accepted_results: Optional[list[TrajectoryResult]] = None,
    _evaluated_results: Optional[list[TrajectoryResult]] = None,
    waypoint_time_factors: tuple[float, ...] = (0.8, 1.0, 1.2),
) -> TrajectoryResult:
    """Evaluate sampled curves passing through one observed gap candidate.

    If robot_yaw is known, use a CUBIC Bezier whose first control point is
    along the robot's actual heading. The cubic also passes through the gap
    at a fixed interpolation parameter and ends at the edge endpoint.
    Different first-control-point distances are tried to find a usable bend.

    Without robot_yaw, preserve the original quadratic construction.
    Only observed LiDAR returns are collision checked (unknown space is not).
    """
    if (not np.isfinite([robot_radius, safety_margin, sample_spacing, max_curvature]).all()
            or robot_radius <= 0 or safety_margin < 0
            or sample_spacing <= 0 or max_curvature <= 0):
        raise ValueError("Invalid trajectory parameters")
    if not np.isfinite(max_heading_error) or not 0 < max_heading_error <= np.pi:
        raise ValueError("max_heading_error must be in (0, pi]")
    if robot_yaw is not None and not np.isfinite(robot_yaw):
        raise ValueError("robot_yaw must be finite")

    start = points_to_edge_frame(
        np.asarray([robot_position], dtype=float), observation.start, observation.end
    )[0]
    end = np.array([observation.length, 0.0])
    gap = np.array([candidate.s, candidate.t])
    remaining = end[0] - start[0]
    if remaining <= 0.05 or gap[0] <= start[0] + 0.05 or gap[0] >= end[0] - 0.05:
        result = TrajectoryResult(False, "Gap is not between the robot and edge destination", candidate=candidate)
        if _evaluated_results is not None:
            _evaluated_results.append(result)
        return result

    u = float((gap[0] - start[0]) / remaining)
    n_samples = max(24, int(np.ceil(2 * np.linalg.norm(end - start) / sample_spacing)))
    ts = np.linspace(0.0, 1.0, n_samples + 1)
    obstacles = observation.obstacle_points_st
    if len(obstacles) == 0:
        result = TrajectoryResult(False, "No measured obstacles; cannot validate this gap", candidate=candidate)
        if _evaluated_results is not None:
            _evaluated_results.append(result)
        return result

    control_sets = []
    if robot_yaw is None:
        control = (gap - (1-u)**2 * start - u**2 * end) / (2*u*(1-u))
        control_sets.append((start, control, end))
    else:
        edge_yaw = float(np.arctan2(
            observation.end[1] - observation.start[1],
            observation.end[0] - observation.start[0],
        ))
        relative_yaw = float(np.arctan2(
            np.sin(robot_yaw - edge_yaw), np.cos(robot_yaw - edge_yaw)
        ))
        direction = np.array([np.cos(relative_yaw), np.sin(relative_yaw)])
        # A forward-monotone candidate cannot begin facing sideways/backward.
        # This calls for alignment, not a declaration that the edge is blocked.
        if direction[0] <= 1e-5:
            result = TrajectoryResult(False, "Initial heading points away from edge; align first", candidate=candidate)
            if _evaluated_results is not None:
                _evaluated_results.append(result)
            return result
        for time_factor in waypoint_time_factors:
            if not np.isfinite(time_factor) or time_factor <= 0:
                raise ValueError("waypoint_time_factors must be finite positive numbers")
            waypoint_u = float(np.clip(u * time_factor, 0.12, 0.88))
            for factor in (0.12, 0.25, 0.40, 0.65, 0.90):
                p1 = start + factor * remaining * direction
                # Keep B(waypoint_u) exactly on the selected gap center.
                # Varying the parameter changes the approach shape while
                # preserving the robot-heading tangent at the start.
                denom = 3 * (1-waypoint_u) * waypoint_u**2
                p2 = (gap - (1-waypoint_u)**3*start
                      - 3*(1-waypoint_u)**2*waypoint_u*p1
                      - waypoint_u**3*end) / denom
                control_sets.append((start, p1, p2, end))

    failures = []
    successes = []
    for controls in control_sets:
        if len(controls) == 3:
            p0, p1, p2 = controls
            points = ((1-ts)**2)[:, None]*p0 + (2*(1-ts)*ts)[:, None]*p1 + (ts**2)[:, None]*p2
            first = 2*((1-ts)[:, None]*(p1-p0) + ts[:, None]*(p2-p1))
            second = np.broadcast_to(2*(p2-2*p1+p0), first.shape)
        else:
            p0, p1, p2, p3 = controls
            points = ((1-ts)**3)[:, None]*p0 + (3*(1-ts)**2*ts)[:, None]*p1 + (3*(1-ts)*ts**2)[:, None]*p2 + (ts**3)[:, None]*p3
            first = (
                3*((1-ts)**2)[:, None]*(p1-p0)
                + 6*((1-ts)*ts)[:, None]*(p2-p1)
                + 3*(ts**2)[:, None]*(p3-p2)
            )
            second = 6*(1-ts)[:, None]*(p2-2*p1+p0) + 6*ts[:, None]*(p3-2*p2+p1)
        path_st = tuple((float(x), float(y)) for x, y in points)
        def reject(reason, *, clearance=None, curvature=None):
            failures.append(reason)
            if _evaluated_results is not None:
                _evaluated_results.append(TrajectoryResult(
                    False, reason, path_st, clearance, curvature, candidate
                ))

        if np.any(np.diff(points[:, 0]) <= 1e-9):
            reject("Trajectory reverses along the edge")
            continue
        if np.max(np.abs(points[:, 1])) > 1.0:
            reject("Trajectory leaves the 1 m analysis band")
            continue
        speed2 = np.sum(first**2, axis=1)
        if np.any(speed2 < 1e-8):
            reject("Degenerate curve")
            continue
        curvature = np.abs(first[:, 0]*second[:, 1] - first[:, 1]*second[:, 0]) / speed2**1.5
        worst = float(np.max(curvature))
        if worst > max_curvature:
            reject("Excessive curvature", curvature=worst)
            continue
        min_d2 = float('inf')
        for chunk in np.array_split(points, max(1, int(np.ceil(len(points)/128)))):
            d2 = np.sum((chunk[:, None, :] - obstacles[None, :, :])**2, axis=2)
            min_d2 = min(min_d2, float(np.min(d2)))
        measured_clearance = float(np.sqrt(min_d2))
        if measured_clearance < robot_radius + safety_margin:
            reject("Too close to measured returns", clearance=measured_clearance, curvature=worst)
            continue
        accepted = TrajectoryResult(
            True, "Sampled heading-tangent curve avoids observed returns (unknown space not checked)"
            if robot_yaw is not None else "Sampled curve avoids observed returns (unknown space not checked)",
            path_st, measured_clearance, worst, candidate,
        )
        successes.append(accepted)
        if _evaluated_results is not None:
            _evaluated_results.append(accepted)
    if successes:
        if _accepted_results is not None:
            _accepted_results.extend(successes)
        return max(successes, key=lambda r: r.min_observed_clearance - 0.005*r.max_curvature)
    from collections import Counter
    failure_counts = Counter(failures)
    return TrajectoryResult(
        False, "No valid heading-tangent curve: " + "; ".join(
            f"{reason}: {count}" for reason, count in failure_counts.items()
        ), candidate=candidate,
    )


@dataclass(frozen=True)
class TrajectorySearchResult:
    feasible: tuple[TrajectoryResult, ...]
    rejected: tuple[TrajectoryResult, ...]


def search_candidate_trajectories(
    observation: EdgeObservation,
    robot_position: Point,
    *,
    robot_radius: float = 0.089,
    safety_margin: float = 0.04,
    max_curvature: float = 5.0,
    robot_yaw: Optional[float] = None,
    samples_per_gap: int = 5,
    max_gap_candidates: int = 40,
) -> TrajectorySearchResult:
    """Collect all evaluated curves, including failed variants for debugging.

    This search evaluates observed gaps only. Feasible means sampled geometric
    checks passed, not that free space or dynamic safety is guaranteed.
    """
    if max_gap_candidates < 1:
        raise ValueError("max_gap_candidates must be positive")
    options = find_feasible_gap_candidates(
        observation, robot_radius=robot_radius, safety_margin=safety_margin,
        samples_per_gap=samples_per_gap,
    )
    # Prefer gaps ahead of the robot and closer to the current edge segment.
    robot_s = float(points_to_edge_frame(
        np.asarray([robot_position], dtype=float), observation.start, observation.end
    )[0, 0])
    options = [o for o in options if robot_s + 0.05 < o.s < observation.length - 0.05]
    options.sort(key=lambda o: (o.s, -o.width, abs(o.t)))
    trials: list[TrajectoryResult] = []
    for option in options[:max_gap_candidates]:
        evaluate_trajectory(
            observation, robot_position, option,
            robot_radius=robot_radius, safety_margin=safety_margin,
            max_curvature=max_curvature, robot_yaw=robot_yaw,
            _evaluated_results=trials,
        )
    good = sorted((r for r in trials if r.feasible),
                  key=lambda r: r.min_observed_clearance - 0.005*r.max_curvature,
                  reverse=True)
    bad = tuple(r for r in trials if not r.feasible)
    return TrajectorySearchResult(tuple(good), bad)


def find_all_feasible_trajectories(
    observation: EdgeObservation,
    robot_position: Point,
    *,
    robot_radius: float = 0.089,
    safety_margin: float = 0.04,
    max_curvature: float = 5.0,
    robot_yaw: Optional[float] = None,
) -> list[TrajectoryResult]:
    """All accepted sampled curves, including alternate cubic control lengths.

    Sorted best-first by the same heuristic as plan_candidate_trajectory.
    Intended for debugging: these paths are NOT certified safe in unknown space.
    """
    return list(search_candidate_trajectories(
        observation, robot_position, robot_radius=robot_radius,
        safety_margin=safety_margin, max_curvature=max_curvature,
        robot_yaw=robot_yaw, samples_per_gap=3,
        max_gap_candidates=10000,
    ).feasible)


def plan_candidate_trajectory(
    observation: EdgeObservation,
    robot_position: Point,
    *,
    robot_radius: float = 0.089,
    safety_margin: float = 0.04,
    max_curvature: float = 5.0,
    robot_yaw: Optional[float] = None,
    max_heading_error: float = np.deg2rad(30.0),
) -> TrajectoryResult:
    """Return the same best path used by the debugger search."""
    from collections import Counter

    search = search_candidate_trajectories(
        observation, robot_position,
        robot_radius=robot_radius, safety_margin=safety_margin,
        max_curvature=max_curvature, robot_yaw=robot_yaw,
    )
    if search.feasible:
        return search.feasible[0]
    if not search.rejected:
        return TrajectoryResult(False, "No observed gap candidate ahead of the robot")
    counts = Counter(r.reason for r in search.rejected)
    breakdown = "; ".join(f"{reason}: {count}" for reason, count in counts.items())
    return TrajectoryResult(False, f"No feasible sampled curve. Rejections: {breakdown}")


def choose_avoidance_target(
    observation: EdgeObservation,
    robot_position: Point,
    nominal_target: Point,
    *,
    robot_radius: float = 0.089,
    safety_margin: float = 0.04,
    lookahead_distance: float = 0.30,
    robot_yaw: Optional[float] = None,
    max_heading_error: float = np.deg2rad(30.0),
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
        safety_margin=safety_margin, robot_yaw=robot_yaw,
        max_heading_error=max_heading_error,
    )
    if not trial.feasible:
        return TargetResult(None, trial.reason)
    path = np.asarray(trial.path_st)
    distances = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    index = min(int(np.searchsorted(distances, lookahead_distance)), len(path)-1)
    s,t = path[index]
    target = edge_to_world(s,t, observation.start, observation.end)
    return TargetResult(target, "Lookahead on candidate curve; only observed-return clearance checked", False)
