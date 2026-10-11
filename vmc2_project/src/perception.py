"""Perception Module v1.0: ROS-independent LiDAR analysis and trajectory planning.

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
        """Measured gap width between lateral obstacle-cluster boundaries."""
        return self.right - self.left

    @property
    def midpoint(self) -> float:
        """Lateral midpoint of an observed gap, not necessarily a safe target."""
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
        """Length of the directed graph edge in the fixed XY frame."""
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
    """Return edge-frame points in a longitudinal strip centered at ``s0``."""
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



def _progress_rejection(path: np.ndarray, *, backtrack_budget: float,
                        max_length_ratio: float) -> Optional[str]:
    """Permit small cumulative reverse-s motion, but reject loops and detours.

    The budget measures *total* negative longitudinal travel, not just the
    largest single reversed step; net forward movement remains mandatory.
    """
    if not np.isfinite([backtrack_budget, max_length_ratio]).all() or backtrack_budget < 0 or max_length_ratio < 1:
        raise ValueError("Invalid progress constraints")
    delta = np.diff(path, axis=0)
    longitudinal = delta[:, 0]
    net = float(path[-1, 0] - path[0, 0])
    if net <= 0.05:
        return "Insufficient net forward progress"
    reversed_s = float(np.maximum(-longitudinal, 0.0).sum())
    if reversed_s > backtrack_budget + 1e-9:
        return "Trajectory reverses along the edge"
    total_length = float(np.linalg.norm(delta, axis=1).sum())
    direct = float(np.linalg.norm(path[-1] - path[0]))
    if total_length > max_length_ratio * max(direct, 1e-6):
        return "Excessive trajectory length"
    return None


def _shift_gap_candidates(candidates: list[GapCandidate], edge_length: float,
                          offsets: tuple[float, ...]) -> list[GapCandidate]:
    """Longitudinally sample around observed gaps; shifted points are UNVERIFIED.

    Keep the originating slice_s for debugging; clearance checks still apply.
    """
    if not offsets or not np.isfinite(offsets).all():
        raise ValueError("Offsets must be finite and nonempty")
    shifted = []
    for candidate in candidates:
        for offset in offsets:
            s = candidate.s + offset
            if 0.0 < s < edge_length:
                shifted.append(GapCandidate(float(s), candidate.t, candidate.width,
                                            candidate.slice_s))
    return shifted

def evaluate_trajectory(
    observation: EdgeObservation,
    robot_position: Point,
    candidate: GapCandidate,
    *,
    robot_radius: float = 0.089,
    safety_margin: float = 0.04,
    sample_spacing: float = 0.025,
    max_curvature: float = 5.0,
    backtrack_budget: float = 0.05,
    max_length_ratio: float = 3.0,
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

        progress_reason = _progress_rejection(points, backtrack_budget=backtrack_budget,
                                              max_length_ratio=max_length_ratio)
        if progress_reason:
            reject(progress_reason)
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
    backtrack_budget: float = 0.05,
    max_length_ratio: float = 3.0,
    robot_yaw: Optional[float] = None,
    samples_per_gap: int = 5,
    max_gap_candidates: int = 40,
    longitudinal_offsets: tuple[float, ...] = (-0.10, -0.05, 0.0, 0.05, 0.10),
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
    options = _shift_gap_candidates(options[:max_gap_candidates], observation.length, longitudinal_offsets)
    for option in options:
        evaluate_trajectory(
            observation, robot_position, option,
            robot_radius=robot_radius, safety_margin=safety_margin,
            max_curvature=max_curvature, robot_yaw=robot_yaw,
            backtrack_budget=backtrack_budget, max_length_ratio=max_length_ratio,
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

    `nominal_target` must match the observation's directed edge endpoint.
    Does not claim `clearance_verified`: occlusion/unknown space is untested.
    """
    if not np.allclose(nominal_target, observation.end, atol=1e-3):
        return TargetResult(None, "nominal_target must match the directed edge endpoint")
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


def search_two_waypoint_trajectories(
    observation: EdgeObservation,
    robot_position: Point,
    *,
    robot_yaw: Optional[float] = None,
    robot_radius: float = 0.089,
    safety_margin: float = 0.04,
    max_curvature: float = 5.0,
    backtrack_budget: float = 0.05,
    max_length_ratio: float = 3.0,
    samples_per_gap: int = 3,
    max_pairs: int = 100,
    longitudinal_offsets: tuple[float, ...] = (-0.05, 0.0, 0.05),
    minimum_waypoint_separation: float = 0.12,
) -> TrajectorySearchResult:
    """Fallback: two gap waypoints joined by three tangent-continuous cubics.

    Heuristic only. Tests sampled obstacle-point clearance and curvature; it
    does not validate unseen space, actual wall surfaces, or controller dynamics.
    """
    if max_pairs < 1 or minimum_waypoint_separation <= 0:
        raise ValueError("Invalid fallback search settings")
    if robot_yaw is not None and not np.isfinite(robot_yaw):
        raise ValueError("robot_yaw must be finite")
    if robot_radius <= 0 or safety_margin < 0 or max_curvature <= 0:
        raise ValueError("Invalid clearance or curvature")
    start = points_to_edge_frame(np.asarray([robot_position], dtype=float),
                                 observation.start, observation.end)[0]
    goal = np.array([observation.length, 0.0])
    edge_yaw = np.arctan2(observation.end[1] - observation.start[1],
                          observation.end[0] - observation.start[0])
    heading = 0.0 if robot_yaw is None else float(np.arctan2(
        np.sin(robot_yaw - edge_yaw), np.cos(robot_yaw - edge_yaw)))
    if np.cos(heading) <= 1e-5:
        return TrajectorySearchResult((), (TrajectoryResult(
            False, "Initial heading points away from edge; align first"),))
    options = find_feasible_gap_candidates(
        observation, robot_radius=robot_radius,
        safety_margin=safety_margin, samples_per_gap=samples_per_gap)
    options = _shift_gap_candidates(options, observation.length, longitudinal_offsets)
    options = [c for c in options if start[0] + .05 < c.s < goal[0] - .05]
    # Select distinctive cross-sections, to avoid evaluating hundreds of
    # nearly identical pairs from adjacent slices.
    options.sort(key=lambda c: (c.s, abs(c.t)))
    pairs = [(a, b) for i, a in enumerate(options) for b in options[i + 1:]
             if b.s - a.s >= minimum_waypoint_separation]
    pairs.sort(key=lambda ab: (abs(ab[0].t - ab[1].t),
                                abs(ab[0].t) + abs(ab[1].t), -ab[1].s))
    trials = []
    obstacles = observation.obstacle_points_st
    if len(obstacles) == 0:
        return TrajectorySearchResult((), (TrajectoryResult(False, "No observed returns"),))

    for a, b in pairs[:max_pairs]:
        knots = np.asarray([start, (a.s, a.t), (b.s, b.t), goal], dtype=float)
        distances = np.linalg.norm(np.diff(knots, axis=0), axis=1)
        if np.any(distances < 0.03):
            continue
        for tangent_factor in (0.35, 0.65, 0.95):
            # Shared waypoint derivatives preserve tangent continuity.
            tangents = np.zeros((4, 2))
            tangents[0] = np.array([np.cos(heading), np.sin(heading)])
            tangents[1] = (knots[2] - knots[0]) / np.linalg.norm(knots[2] - knots[0])
            tangents[2] = (knots[3] - knots[1]) / np.linalg.norm(knots[3] - knots[1])
            tangents[3] = np.array([1., 0.])
            pieces = []
            derivs = []
            second_derivs = []
            # Derivatives are scaled by segment length; curvature calculation
            # uses per-segment parameterization and is invariant to this scale.
            for i in range(3):
                p0, p3 = knots[i], knots[i + 1]
                length = distances[i]
                p1 = p0 + (tangent_factor * length / 3.) * tangents[i]
                p2 = p3 - (tangent_factor * length / 3.) * tangents[i + 1]
                n = max(12, int(np.ceil(length / 0.025)))
                t = np.linspace(0., 1., n + 1)
                omt = 1. - t
                pts = (omt**3)[:, None]*p0 + (3*omt**2*t)[:, None]*p1 + (3*omt*t**2)[:, None]*p2 + (t**3)[:, None]*p3
                d1 = 3*(omt**2)[:, None]*(p1-p0) + 6*(omt*t)[:, None]*(p2-p1) + 3*(t**2)[:, None]*(p3-p2)
                d2 = 6*omt[:, None]*(p2 - 2*p1 + p0) + 6*t[:, None]*(p3 - 2*p2 + p1)
                if i:
                    pts, d1, d2 = pts[1:], d1[1:], d2[1:]
                pieces.append(pts)
                derivs.append(d1)
                second_derivs.append(d2)
            path = np.concatenate(pieces)
            d1s = np.concatenate(derivs)
            d2s = np.concatenate(second_derivs)
            path_st = tuple(map(tuple, path.tolist()))
            candidate = a
            reason = None
            min_dist = None
            worst = None
            progress_reason = _progress_rejection(path, backtrack_budget=backtrack_budget,
                                                  max_length_ratio=max_length_ratio)
            if progress_reason:
                reason = progress_reason
            elif np.max(np.abs(path[:, 1])) > 1.0:
                reason = "Trajectory leaves the 1 m analysis band"
            elif np.any(np.sum(d1s**2, axis=1) < 1e-8):
                reason = "Degenerate curve"
            else:
                speed2 = np.sum(d1s**2, axis=1)
                curvature = np.abs(d1s[:,0]*d2s[:,1] - d1s[:,1]*d2s[:,0]) / speed2**1.5
                worst = float(np.max(curvature))
                if worst > max_curvature:
                    reason = "Excessive curvature"
                else:
                    min_d2 = float('inf')
                    for chunk in np.array_split(path, max(1, int(np.ceil(len(path) / 128)))):
                        min_d2 = min(min_d2, float(np.min(np.sum(
                            (chunk[:,None,:]-obstacles[None,:,:])**2, axis=2))))
                    min_dist = float(np.sqrt(min_d2))
                    if min_dist < robot_radius + safety_margin:
                        reason = "Too close to measured returns"
            trials.append(TrajectoryResult(
                reason is None,
                reason or "Two-waypoint sampled path avoids observed returns (unknown space not checked)",
                path_st, min_dist, worst, candidate))
    feasible = sorted((t for t in trials if t.feasible),
                      key=lambda t: t.min_observed_clearance - 0.005*t.max_curvature,
                      reverse=True)
    return TrajectorySearchResult(tuple(feasible), tuple(t for t in trials if not t.feasible))


def search_approach_trajectories(
    observation: EdgeObservation,
    robot_position: Point,
    *,
    robot_yaw: Optional[float] = None,
    robot_radius: float = 0.089,
    safety_margin: float = 0.04,
    max_curvature: float = 5.0,
    backtrack_budget: float = 0.05,
    max_length_ratio: float = 3.0,
    max_gap_candidates: int = 30,
    longitudinal_offsets: tuple[float, ...] = (-0.05, 0., 0.05),
    beyond_gap: float = 0.025,
    arrival_heading_offsets: tuple[float, ...] = (-0.35, 0., 0.35),
) -> TrajectorySearchResult:
    """Plan short robot-heading-tangent curves toward observed openings.

    Unlike full-edge search, the terminal tangent follows the line from the
    robot to each local goal, with small angular variations. The endpoint is
    near the opening, not at the graph node. Results are observations-only.
    """
    if (not np.isfinite([robot_radius, safety_margin, max_curvature,
                         backtrack_budget, max_length_ratio, beyond_gap]).all()
            or robot_radius <= 0 or safety_margin < 0 or max_curvature <= 0
            or backtrack_budget < 0 or max_length_ratio <= 1
            or max_gap_candidates < 1 or beyond_gap < 0):
        raise ValueError("Invalid approach settings")
    if robot_yaw is not None and not np.isfinite(robot_yaw):
        raise ValueError("robot_yaw must be finite")
    start = points_to_edge_frame(np.asarray([robot_position], dtype=float),
                                 observation.start, observation.end)[0]
    edge_yaw = float(np.arctan2(observation.end[1] - observation.start[1],
                                observation.end[0] - observation.start[0]))
    relative_yaw = 0. if robot_yaw is None else float(np.arctan2(
        np.sin(robot_yaw - edge_yaw), np.cos(robot_yaw - edge_yaw)))
    initial_heading = np.array([np.cos(relative_yaw), np.sin(relative_yaw)])
    if initial_heading[0] <= 1e-5:
        return TrajectorySearchResult((), (TrajectoryResult(
            False, "Initial heading points away from edge; align first"),))
    options = find_feasible_gap_candidates(
        observation, robot_radius=robot_radius, safety_margin=safety_margin,
        samples_per_gap=3)
    options = _shift_gap_candidates(options, observation.length, longitudinal_offsets)
    options = [c for c in options if start[0] + 0.10 < c.s
               and c.s + beyond_gap < observation.length - 0.02]
    if not options:
        return TrajectorySearchResult((), (TrajectoryResult(
            False, "No observed opening ahead for a short approach"),))
    # Spread the budget across longitudinal sections; nearby duplicates
    # should not monopolize the candidate search.
    options.sort(key=lambda c: (abs(c.s - (start[0] + 0.45)), abs(c.t - start[1])))
    selected = []
    seen = set()
    for c in options:
        key = (round(c.s / 0.05), round(c.t / 0.04))
        if key in seen:
            continue
        seen.add(key)
        selected.append(c)
        if len(selected) >= max_gap_candidates:
            break
    obstacles = observation.obstacle_points_st
    if not len(obstacles):
        return TrajectorySearchResult((), (TrajectoryResult(False, "No observed returns"),))
    trials = []
    for candidate in selected:
        goal = np.array([candidate.s + beyond_gap, candidate.t], dtype=float)
        delta = goal - start
        chord = float(np.linalg.norm(delta))
        if chord < .10:
            continue
        direct_angle = float(np.arctan2(delta[1], delta[0]))
        # Direct tangent, with modest variations to account for diagonal walls.
        for offset in arrival_heading_offsets:
            arrival = np.array([np.cos(direct_angle + offset),
                                np.sin(direct_angle + offset)])
            for begin_factor in (0.12, 0.25, 0.40):
                for end_factor in (0.12, 0.25, 0.40):
                    p0 = start
                    p1 = start + begin_factor * chord * initial_heading
                    p3 = goal
                    p2 = goal - end_factor * chord * arrival
                    n = max(30, int(np.ceil(chord / .018)))
                    t = np.linspace(0., 1., n + 1)
                    omt = 1. - t
                    path = ((omt**3)[:,None]*p0 + (3*omt**2*t)[:,None]*p1
                            + (3*omt*t**2)[:,None]*p2 + (t**3)[:,None]*p3)
                    d1 = (3*omt**2)[:,None]*(p1-p0) + (6*omt*t)[:,None]*(p2-p1) + (3*t**2)[:,None]*(p3-p2)
                    d2 = (6*omt)[:,None]*(p2-2*p1+p0) + (6*t)[:,None]*(p3-2*p2+p1)
                    reason = _progress_rejection(path, backtrack_budget=backtrack_budget,
                                                  max_length_ratio=max_length_ratio)
                    clearance = None
                    worst = None
                    if reason is None and np.max(np.abs(path[:,1])) > 1.0:
                        reason = "Trajectory leaves the 1 m analysis band"
                    if reason is None:
                        speed2 = np.sum(d1*d1, axis=1)
                        if np.any(speed2 < 1e-8):
                            reason = "Degenerate curve"
                        else:
                            kappa = np.abs(d1[:,0]*d2[:,1]-d1[:,1]*d2[:,0]) / speed2**1.5
                            worst = float(np.max(kappa))
                            if worst > max_curvature:
                                reason = "Excessive curvature"
                            else:
                                minimum = float('inf')
                                for chunk in np.array_split(path, max(1, int(np.ceil(len(path)/128)))):
                                    minimum = min(minimum, float(np.min(np.sum(
                                        (chunk[:,None,:]-obstacles[None,:,:])**2, axis=2))))
                                clearance = float(np.sqrt(minimum))
                                if clearance < robot_radius + safety_margin:
                                    reason = "Too close to measured returns"
                    trials.append(TrajectoryResult(
                        reason is None,
                        reason or "Local approach to observed opening; edge traversal not established",
                        tuple((float(x),float(y)) for x,y in path),
                        clearance, worst, candidate))
    feasible = sorted((r for r in trials if r.feasible),
                      key=lambda r: r.min_observed_clearance - .005*r.max_curvature,
                      reverse=True)
    return TrajectorySearchResult(tuple(feasible), tuple(r for r in trials if not r.feasible))


def search_traverse_trajectories(
    observation: EdgeObservation,
    robot_position: Point,
    *,
    robot_yaw: Optional[float] = None,
    robot_radius: float = 0.089,
    safety_margin: float = 0.04,
    max_curvature: float = 8.0,
    backtrack_budget: float = 0.05,
    max_length_ratio: float = 3.0,
    near_distance: float = 0.22,
    travel_distance: float = 0.20,
) -> TrajectorySearchResult:
    """Try short, heading-tangent continuations at an observed opening.

    Do not force a return to the edge endpoint or an already-passed gap.
    Observed returns are *not* a free-space certificate.
    """
    if (not np.isfinite([robot_radius, safety_margin, max_curvature,
                         backtrack_budget, max_length_ratio, near_distance,
                         travel_distance]).all()
            or robot_radius <= 0 or safety_margin < 0 or max_curvature <= 0
            or backtrack_budget < 0 or max_length_ratio < 1
            or near_distance <= 0 or travel_distance <= 0):
        raise ValueError("Invalid traversal settings")
    if robot_yaw is not None and not np.isfinite(robot_yaw):
        raise ValueError("robot_yaw must be finite")
    start = points_to_edge_frame(np.asarray([robot_position], dtype=float),
                                 observation.start, observation.end)[0]
    edge_yaw = float(np.arctan2(observation.end[1]-observation.start[1],
                                observation.end[0]-observation.start[0]))
    yaw = edge_yaw if robot_yaw is None else robot_yaw
    relative = yaw - edge_yaw
    heading = np.array([np.cos(relative), np.sin(relative)])
    if heading[0] <= 0.0:
        return TrajectorySearchResult((), (TrajectoryResult(False, "Initial heading points away from edge; align first"),))

    gaps = find_feasible_gap_candidates(observation, robot_radius=robot_radius,
                                        safety_margin=safety_margin, samples_per_gap=3)
    # A gap only needs to be near the robot, not necessarily ahead of its center.
    # Do not extrapolate validity from distant or already-passed openings.
    near = [g for g in gaps if abs(g.s-start[0]) <= near_distance]
    if not near:
        return TrajectorySearchResult((), (TrajectoryResult(False, "No observed opening near robot for traversal"),))
    near.sort(key=lambda g: (abs(g.s-start[0]), abs(g.t-start[1])))
    obstacles = observation.obstacle_points_st
    if len(obstacles) == 0:
        return TrajectorySearchResult((), (TrajectoryResult(False, "No observed obstacle returns"),))

    trials = []
    for gap in near[:12]:
        # Keep target ahead. When at the throat, continuing in the robot's
        # heading can be preferable to recentering at the detected gap.
        target_s = min(observation.length-0.02,
                       max(start[0]+0.10, gap.s+0.10))
        if target_s <= start[0]+0.05:
            continue
        for blend in (0.0, 0.5, 1.0):
            target_t = start[1] + blend*(gap.t-start[1])
            goal = np.array([target_s, target_t])
            displacement = goal-start
            for handle in (0.15, 0.30, 0.50):
                # Cubic initial tangent follows robot yaw exactly.
                p0 = start
                p1 = start + handle*np.linalg.norm(displacement)*heading
                p2 = goal - 0.25*np.linalg.norm(displacement)*heading
                p3 = goal
                t = np.linspace(0., 1., max(30, int(np.ceil(np.linalg.norm(displacement)/.015)))+1)
                omt = 1-t
                path = ((omt**3)[:,None]*p0 + (3*omt**2*t)[:,None]*p1
                        + (3*omt*t**2)[:,None]*p2 + (t**3)[:,None]*p3)
                d1 = ((3*omt**2)[:,None]*(p1-p0) + (6*omt*t)[:,None]*(p2-p1)
                      + (3*t**2)[:,None]*(p3-p2))
                d2 = ((6*omt)[:,None]*(p2-2*p1+p0)
                      + (6*t)[:,None]*(p3-2*p2+p1))
                reason = _progress_rejection(path, backtrack_budget=backtrack_budget,
                                              max_length_ratio=max_length_ratio)
                worst = None
                clearance = None
                if reason is None and np.max(np.abs(path[:,1])) > 1.0:
                    reason = "Trajectory leaves the 1 m analysis band"
                if reason is None:
                    speed2 = np.sum(d1*d1, axis=1)
                    if np.any(speed2 < 1e-8):
                        reason = "Degenerate curve"
                    else:
                        curvature = np.abs(d1[:,0]*d2[:,1]-d1[:,1]*d2[:,0])/speed2**1.5
                        worst = float(np.max(curvature))
                        if worst > max_curvature:
                            reason = "Excessive curvature"
                        else:
                            dist2 = np.sum((path[:,None,:]-obstacles[None,:,:])**2, axis=2)
                            clearance = float(np.sqrt(np.min(dist2)))
                            if clearance < robot_radius+safety_margin:
                                reason = "Too close to measured returns"
                trials.append(TrajectoryResult(reason is None,
                    reason or "Short continuation near opening; edge traversal not established",
                    tuple((float(x),float(y)) for x,y in path), clearance, worst, gap))
    good = sorted((r for r in trials if r.feasible),
                  key=lambda r: r.min_observed_clearance-.005*r.max_curvature,
                  reverse=True)
    return TrajectorySearchResult(tuple(good), tuple(r for r in trials if not r.feasible))


def select_forward_observation(
    observation: EdgeObservation,
    robot_position: Point,
    *,
    robot_radius: float = 0.089,
    passed_tolerance: float = 0.03,
) -> EdgeObservation:
    """Keep candidate-opening slices not yet cleared by the robot's rear.

    Observed obstacle points are intentionally NOT filtered: a wall immediately
    behind or beside the robot can still constrain the next proposed curve.
    This produces a per-cycle view, with no persistent map or gap identifiers.
    """
    if (not np.isfinite([robot_radius, passed_tolerance]).all()
            or robot_radius <= 0 or passed_tolerance < 0):
        raise ValueError("Invalid robot radius or passed tolerance")
    robot_s = float(points_to_edge_frame(
        np.asarray([robot_position], dtype=float), observation.start,
        observation.end,
    )[0, 0])
    # Only mark an opening passed when the whole robot footprint is ahead of it.
    last_passed_s = robot_s - robot_radius - passed_tolerance
    upcoming = tuple(section for section in observation.slices
                     if section.s >= last_passed_s)
    return EdgeObservation(observation.start, observation.end,
                           observation.obstacle_points_st, upcoming)


@dataclass(frozen=True)
class LocalPassage:
    """Consecutive observed gaps with overlapping robot-center intervals."""
    s_start: float
    s_end: float
    usable_left: float
    usable_right: float
    slice_positions: tuple[float, ...]

    @property
    def target_t(self) -> float:
        return (self.usable_left + self.usable_right) / 2.0


def find_local_passages(
    observation: EdgeObservation, *, robot_radius: float = 0.089,
    safety_margin: float = 0.03, max_slice_separation: float = 0.16,
) -> tuple[LocalPassage, ...]:
    """Link adjacent, overlapping *observed* usable openings.

    This is a local-goal heuristic, not a visibility or free-space proof.
    A passage ends when the interval intersection closes; no unknown slice
    is silently bridged. Parallel openings on the same slice stay separate.
    """
    if (not np.isfinite([robot_radius, safety_margin, max_slice_separation]).all()
            or robot_radius <= 0 or safety_margin < 0 or max_slice_separation <= 0):
        raise ValueError("Invalid passage parameters")
    margin = robot_radius + safety_margin
    active = []  # [start, end, left, right, slice_positions]
    finished = []
    previous_s = None
    for section in sorted(observation.slices, key=lambda section: section.s):
        if previous_s is not None and section.s - previous_s > max_slice_separation:
            finished.extend(active)
            active = []
        intervals = [(g.left + margin, g.right - margin) for g in section.gaps
                     if g.right - g.left > 2 * margin]
        next_active = []
        for left, right in intervals:
            matches = [run for run in active if run[2] <= right and left <= run[3]]
            if matches:
                # Don't combine distinct routes through separate obstacles.
                run = max(matches, key=lambda r: min(right, r[3]) - max(left, r[2]))
                next_active.append([run[0], section.s, max(left, run[2]),
                                    min(right, run[3]), run[4] + (section.s,)])
            else:
                next_active.append([section.s, section.s, left, right, (section.s,)])
        for run in active:
            if not any(run[0] == nxt[0] and run[4][0] == nxt[4][0] for nxt in next_active):
                finished.append(run)
        active = next_active
        previous_s = section.s
    finished.extend(active)
    return tuple(sorted((LocalPassage(*run) for run in finished),
                        key=lambda run: (run.s_start, -run.s_end, run.target_t)))


def select_next_local_passage(
    observation: EdgeObservation, robot_position: Point, *,
    robot_radius: float = 0.089, safety_margin: float = 0.03,
    passed_tolerance: float = 0.03, max_lookahead: float = 0.90,
) -> Optional[LocalPassage]:
    """Prefer the nearest upcoming observed passage, not the edge endpoint."""
    robot_s, robot_t = points_to_edge_frame(np.asarray([robot_position]),
                                            observation.start, observation.end)[0]
    passages = find_local_passages(observation, robot_radius=robot_radius,
                                   safety_margin=safety_margin)
    upcoming = [p for p in passages
                if p.s_end >= robot_s - robot_radius - passed_tolerance
                and p.s_start <= robot_s + max_lookahead]
    if not upcoming:
        return None
    return min(upcoming, key=lambda p: (max(0., p.s_start - robot_s),
                                       -p.s_end, abs(p.target_t - robot_t)))


def search_next_opening(
    observation: EdgeObservation, robot_position: Point, *,
    robot_yaw: Optional[float] = None,
    robot_radius: float = 0.089, safety_margin: float = 0.03,
    max_curvature: float = 8.0, backtrack_budget: float = 0.05,
    max_length_ratio: float = 3.0,
) -> tuple[TrajectorySearchResult, Optional[LocalPassage]]:
    """Plan to the *next passage*, retaining ALL obstacles for collision checks."""
    passage = select_next_local_passage(observation, robot_position,
                                        robot_radius=robot_radius, safety_margin=safety_margin)
    if passage is None:
        return TrajectorySearchResult((), (TrajectoryResult(False, "No observed forward passage"),)), None
    robot_s = float(points_to_edge_frame(np.asarray([robot_position]),
                                           observation.start, observation.end)[0, 0])
    # Target the furthest observed slice in the chosen connected passage.
    # Use its common usable interval, rather than a shifting midpoint.
    end_s = passage.s_end
    width = passage.usable_right - passage.usable_left + 2*(robot_radius + safety_margin)
    synthetic = SliceObservation(end_s, 1, (),
        (Gap(passage.usable_left - (robot_radius+safety_margin),
             passage.usable_right + (robot_radius+safety_margin)),))
    local = EdgeObservation(observation.start, observation.end,
                            observation.obstacle_points_st, (synthetic,))
    if end_s <= robot_s + 0.10:
        # Already crossing this passage: use its original observed slices for
        # the existing short traverse evaluator, not an imaginary distant goal.
        local = EdgeObservation(observation.start, observation.end,
                    observation.obstacle_points_st,
                    tuple(s for s in observation.slices if s.s in passage.slice_positions))
        return search_traverse_trajectories(local, robot_position,
                     robot_yaw=robot_yaw, robot_radius=robot_radius,
                     safety_margin=safety_margin, max_curvature=max_curvature,
                     backtrack_budget=backtrack_budget,
                     max_length_ratio=max_length_ratio), passage
    result = search_approach_trajectories(
        local, robot_position, robot_yaw=robot_yaw, robot_radius=robot_radius,
        safety_margin=safety_margin, max_curvature=max_curvature,
        backtrack_budget=backtrack_budget, max_length_ratio=max_length_ratio,
        longitudinal_offsets=(0.0,), beyond_gap=0.025)
    return result, passage



def local_goal_reached(robot_s: float, robot_t: float, goal_s: float,
                       usable_t: tuple[float, float], *,
                       longitudinal_tolerance: float = 0.05,
                       lateral_tolerance: float = 0.01) -> bool:
    """Waypoint proximity, NOT evidence the robot footprint cleared a wall."""
    values = (robot_s, robot_t, goal_s, *usable_t,
              longitudinal_tolerance, lateral_tolerance)
    if not np.isfinite(values).all() or longitudinal_tolerance < 0 or lateral_tolerance < 0:
        raise ValueError("Invalid local-goal geometry or tolerances")
    left, right = usable_t
    if left > right:
        raise ValueError("Invalid usable interval")
    return (abs(goal_s - robot_s) <= longitudinal_tolerance
            and left - lateral_tolerance <= robot_t <= right + lateral_tolerance)


def search_goal_reached_continuation(
    observation: EdgeObservation, robot_position: Point, *,
    robot_yaw: Optional[float] = None,
    robot_radius: float = 0.089, safety_margin: float = 0.03,
    horizon: float = 0.15, passed_tolerance: float = 0.03,
    longitudinal_tolerance: float = 0.05,
    lateral_tolerance: float = 0.01,
) -> TrajectorySearchResult:
    """Continue briefly once an opening waypoint is effectively reached.

    The robot remains TRAVERSING, never CLEARED; a path is only checked against
    observed obstacle points. This function does not certify unknown space.
    """
    vals = (robot_radius, safety_margin, horizon, passed_tolerance,
            longitudinal_tolerance, lateral_tolerance)
    if (not np.isfinite(vals).all() or robot_radius <= 0 or safety_margin < 0
            or horizon <= 0 or passed_tolerance < 0 or longitudinal_tolerance < 0
            or lateral_tolerance < 0):
        raise ValueError("Invalid continuation parameters")
    if robot_yaw is not None and not np.isfinite(robot_yaw):
        raise ValueError("robot_yaw must be finite")
    robot_st = points_to_edge_frame(np.asarray([robot_position], dtype=float),
                                    observation.start, observation.end)[0]
    passage = select_next_local_passage(
        observation, robot_position, robot_radius=robot_radius,
        safety_margin=safety_margin, passed_tolerance=passed_tolerance)
    if passage is None or not local_goal_reached(
        robot_st[0], robot_st[1], passage.s_end,
        (passage.usable_left, passage.usable_right),
        longitudinal_tolerance=longitudinal_tolerance,
        lateral_tolerance=lateral_tolerance):
        return TrajectorySearchResult((), (TrajectoryResult(False, "Local opening goal not reached"),))

    edge_yaw = float(np.arctan2(observation.end[1]-observation.start[1],
                                observation.end[0]-observation.start[0]))
    yaw = edge_yaw if robot_yaw is None else robot_yaw
    relative = yaw - edge_yaw
    heading = np.array([np.cos(relative), np.sin(relative)])
    if heading[0] <= 0.0:
        return TrajectorySearchResult((), (TrajectoryResult(False, "Align first: heading points away from edge"),))
    remaining_s = observation.length - robot_st[0]
    if remaining_s <= 0:
        return TrajectorySearchResult((), (TrajectoryResult(False, "Destination already reached longitudinally"),))
    # Bound by remaining graph-edge progress; no repeated targeting of midpoint.
    length = min(horizon, remaining_s / heading[0])
    if length <= 0.01:
        return TrajectorySearchResult((), (TrajectoryResult(False, "Insufficient room for continuation"),))
    samples = max(21, int(np.ceil(length / 0.005)) + 1)
    path = robot_st[None, :] + np.linspace(0, length, samples)[:, None] * heading[None, :]
    if np.max(np.abs(path[:, 1])) > 1.0:
        return TrajectorySearchResult((), (TrajectoryResult(False, "Trajectory leaves the 1 m analysis band", tuple(map(tuple,path))),))
    obstacles = observation.obstacle_points_st
    if len(obstacles) == 0:
        return TrajectorySearchResult((), (TrajectoryResult(False, "Unknown: no observed obstacles", tuple(map(tuple,path))),))
    clearance = float(np.sqrt(np.min(np.sum((path[:,None,:]-obstacles[None,:,:])**2,axis=2))))
    packed = tuple((float(a),float(b)) for a,b in path)
    if clearance < robot_radius + safety_margin:
        return TrajectorySearchResult((), (TrajectoryResult(False,"Too close to measured returns",packed,clearance,0.0),))
    trial = TrajectoryResult(True, "Goal reached; short straight continuation (passage still traversing)",
                             packed, clearance, 0.0)
    return TrajectorySearchResult((trial,), ())


def search_final_approach(
    observation: EdgeObservation, robot_position: Point, *,
    robot_yaw: Optional[float] = None,
    robot_radius: float = 0.089, safety_margin: float = 0.03,
    max_curvature: float = 8.0, backtrack_budget: float = 0.05,
    max_length_ratio: float = 3.0, horizon: float = 0.20,
    passed_tolerance: float = 0.03,
) -> TrajectorySearchResult:
    """Short goal-directed steps after the last *observed* passage was passed.

    This is a local observed-points check, NEVER a free-space certificate.
    No observed passage history -> UNKNOWN, not permission to drive.
    """
    vals = (robot_radius, safety_margin, max_curvature, backtrack_budget,
            max_length_ratio, horizon, passed_tolerance)
    if (not np.isfinite(vals).all() or robot_radius <= 0 or safety_margin < 0
            or max_curvature <= 0 or backtrack_budget < 0
            or max_length_ratio <= 1 or horizon <= 0 or passed_tolerance < 0):
        raise ValueError("Invalid final approach parameters")
    if robot_yaw is not None and not np.isfinite(robot_yaw):
        raise ValueError("robot_yaw must be finite")
    start = points_to_edge_frame(np.asarray([robot_position], dtype=float),
                                 observation.start, observation.end)[0]
    remaining = np.array([observation.length, 0.]) - start
    dist = float(np.linalg.norm(remaining))
    if dist < .035:
        return TrajectorySearchResult((), (TrajectoryResult(False, "Already near graph destination"),))
    passages = find_local_passages(observation, robot_radius=robot_radius,
                                    safety_margin=safety_margin)
    if not passages:
        return TrajectorySearchResult((), (TrajectoryResult(False, "Unknown: no observed passage history"),))
    rear_s = start[0] - robot_radius - passed_tolerance
    if any(p.s_end >= rear_s for p in passages):
        return TrajectorySearchResult((), (TrajectoryResult(False, "An observed passage remains ahead or under robot"),))
    obstacles = observation.obstacle_points_st
    if len(obstacles) == 0:
        return TrajectorySearchResult((), (TrajectoryResult(False, "Unknown: no observed obstacle points"),))
    edge_yaw = float(np.arctan2(observation.end[1]-observation.start[1],
                                observation.end[0]-observation.start[0]))
    relative = 0. if robot_yaw is None else float(np.arctan2(
        np.sin(robot_yaw-edge_yaw), np.cos(robot_yaw-edge_yaw)))
    heading = np.array([np.cos(relative), np.sin(relative)])
    if heading[0] <= 0:
        return TrajectorySearchResult((), (TrajectoryResult(False, "Initial heading points away from edge; align first"),))
    length = min(horizon, dist)
    endpoint = start + remaining / dist * length
    candidates = []
    # Include straight-ahead continuation, plus short goal-directed cubic paths.
    straight = start + np.linspace(0, length, max(20, int(length/.01)+1))[:,None]*heading
    candidates.append((straight, None))
    for handle1 in (.20, .40, .65):
        for handle2 in (.20, .40):
            p0=start
            p1=start + length*handle1*heading
            p2=endpoint - length*handle2*(remaining/dist)
            p3=endpoint
            t=np.linspace(0, 1, max(25, int(length/.01)+1))
            omt=1-t
            path=((omt**3)[:,None]*p0+(3*omt**2*t)[:,None]*p1
                  +(3*omt*t**2)[:,None]*p2+(t**3)[:,None]*p3)
            d1=((3*omt**2)[:,None]*(p1-p0)+(6*omt*t)[:,None]*(p2-p1)
                +(3*t**2)[:,None]*(p3-p2))
            d2=((6*omt)[:,None]*(p2-2*p1+p0)+(6*t)[:,None]*(p3-2*p2+p1))
            speed2=np.sum(d1*d1,axis=1)
            curvature=(float(np.max(np.abs(d1[:,0]*d2[:,1]-d1[:,1]*d2[:,0])
                             /np.maximum(speed2,1e-12)**1.5))
                       if np.all(speed2>1e-8) else float('inf'))
            candidates.append((path,curvature))
    trials=[]
    for path,curvature in candidates:
        reason=_progress_rejection(path,backtrack_budget=backtrack_budget,
                                    max_length_ratio=max_length_ratio)
        if reason is None and np.max(np.abs(path[:,1]))>1.0:
            reason="Trajectory leaves the 1 m analysis band"
        if reason is None and curvature is not None and curvature>max_curvature:
            reason="Excessive curvature"
        clearance=None
        if reason is None:
            clearance=float(np.sqrt(np.min(np.sum(
                (path[:,None,:]-obstacles[None,:,:])**2,axis=2))))
            if clearance < robot_radius+safety_margin:
                reason="Too close to measured returns"
        trials.append(TrajectoryResult(reason is None,
            reason or "Short final approach; free space not certified",
            tuple((float(x),float(y)) for x,y in path), clearance, curvature))
    feasible=sorted((r for r in trials if r.feasible),
                    key=lambda r: (r.path_st[-1][0]-start[0], r.min_observed_clearance),
                    reverse=True)
    return TrajectorySearchResult(tuple(feasible),tuple(r for r in trials if not r.feasible))


@dataclass(frozen=True)
class DestinationAlignment:
    """A proposed rotation followed by a direct segment, not a continuous path."""
    departure_yaw: float
    path_st: tuple[Point, ...]
    clearance: float


def _destination_segment(observation: EdgeObservation, robot_position: Point,
                         robot_radius: float, safety_margin: float):
    """Sample robot-center segment and test measured obstacle-point clearance."""
    if (not np.isfinite([robot_radius, safety_margin]).all()
            or robot_radius <= 0 or safety_margin < 0):
        raise ValueError("Invalid robot clearance")
    start = points_to_edge_frame(np.asarray([robot_position], float),
                                 observation.start, observation.end)[0]
    dest = np.array([observation.length, 0.0])
    dist = float(np.linalg.norm(dest - start))
    if dist <= 0.035:
        return None, "Destination already reached", None
    if dest[0] <= start[0]:
        return None, "Destination is not ahead along selected edge", None
    if dist > 1.6:
        return None, "Destination beyond local direct-search horizon", None
    obstacles = observation.obstacle_points_st
    if len(obstacles) == 0:
        return None, "Unknown: no obstacle observations", None
    steps = max(21, int(np.ceil(dist / 0.005)) + 1)
    path = start[None,:] + np.linspace(0.,1.,steps)[:,None]*(dest-start)[None,:]
    clearance = float(np.sqrt(np.min(np.sum((path[:,None,:] - obstacles[None,:,:])**2,axis=2))))
    packed = tuple(tuple(map(float, xy)) for xy in path)
    if clearance < robot_radius + safety_margin:
        return packed, "Too close to measured returns", clearance
    return packed, None, clearance


def search_direct_destination(observation: EdgeObservation, robot_position: Point, *,
                              robot_yaw: Optional[float] = None,
                              robot_radius: float = 0.089, safety_margin: float = 0.03,
                              heading_tolerance: float = np.deg2rad(12.0)) -> TrajectorySearchResult:
    """Direct straight segment only when initial yaw matches its direction.

    Clearance is checked against observed LiDAR points; unseen space remains unknown.
    """
    if not np.isfinite(heading_tolerance) or not 0 <= heading_tolerance <= np.pi:
        raise ValueError("Invalid heading tolerance")
    path, reason, clearance = _destination_segment(observation, robot_position, robot_radius, safety_margin)
    if reason is not None:
        return TrajectorySearchResult((), (TrajectoryResult(False, reason, path or (), clearance),))
    if robot_yaw is not None:
        if not np.isfinite(robot_yaw):
            raise ValueError("Invalid yaw")
        dx = observation.end[0] - robot_position[0]
        dy = observation.end[1] - robot_position[1]
        bearing = float(np.arctan2(dy, dx))
        delta = float(np.arctan2(np.sin(robot_yaw-bearing), np.cos(robot_yaw-bearing)))
        if abs(delta) > heading_tolerance:
            return TrajectorySearchResult((), (TrajectoryResult(False, "Direct destination requires alignment", path, clearance),))
    return TrajectorySearchResult((TrajectoryResult(True, "Direct to destination (observed-points only)", path, clearance, 0.0),), ())


def search_rotate_to_destination(observation: EdgeObservation, robot_position: Point, *,
                                 robot_yaw: float, robot_radius: float = 0.089,
                                 safety_margin: float = 0.03,
                                 heading_tolerance: float = np.deg2rad(12.0)) -> Optional[DestinationAlignment]:
    """Propose stop -> rotate -> re-scan -> drive for an observed-clear direct line."""
    if not np.isfinite(robot_yaw):
        raise ValueError("Invalid yaw")
    path, reason, clearance = _destination_segment(observation, robot_position, robot_radius, safety_margin)
    if reason is not None or path is None:
        return None
    bearing = float(np.arctan2(observation.end[1] - robot_position[1],
                               observation.end[0] - robot_position[0]))
    diff = float(np.arctan2(np.sin(bearing-robot_yaw), np.cos(bearing-robot_yaw)))
    if abs(diff) <= heading_tolerance:
        return None
    # A circular footprint has identical translational and rotational radius.
    # This is a necessary check only, not a certificate of visibility/safety.
    return DestinationAlignment(bearing, path, float(clearance))


def search_smooth_destination(observation: EdgeObservation, robot_position: Point, *,
                              robot_yaw: Optional[float] = None,
                              robot_radius: float = 0.089, safety_margin: float = 0.03,
                              max_curvature: float = 8.0,
                              backtrack_budget: float = 0.05,
                              max_length_ratio: float = 3.0) -> TrajectorySearchResult:
    """Try heading-tangent cubics ending at the destination, without gap waypoints."""
    start = points_to_edge_frame(np.asarray([robot_position], float),
                                 observation.start, observation.end)[0]
    end = np.array([observation.length, 0.])
    dist = float(np.linalg.norm(end-start))
    if dist <= 0.035 or end[0] <= start[0]:
        return TrajectorySearchResult((), (TrajectoryResult(False, "Destination not ahead or already reached"),))
    obstacles = observation.obstacle_points_st
    if len(obstacles) == 0:
        return TrajectorySearchResult((), (TrajectoryResult(False,"Unknown: no obstacle observations"),))
    edge_yaw = float(np.arctan2(observation.end[1]-observation.start[1],
                                observation.end[0]-observation.start[0]))
    relative = 0. if robot_yaw is None else float(np.arctan2(np.sin(robot_yaw-edge_yaw), np.cos(robot_yaw-edge_yaw)))
    start_direction = np.array([np.cos(relative), np.sin(relative)])
    target_direction = (end-start)/dist
    samples = np.linspace(0.,1.,max(35,int(np.ceil(2*dist/.01))))
    trials=[]
    for initial_strength in (.15,.30,.50,.75):
        for final_strength in (.15,.30,.50,.75):
            p0=start; p3=end
            p1=start+dist*initial_strength*start_direction
            p2=end-dist*final_strength*target_direction
            u=samples[:,None]
            path=(1-u)**3*p0+3*(1-u)**2*u*p1+3*(1-u)*u**2*p2+u**3*p3
            d1=3*((1-u)**2*(p1-p0)+2*(1-u)*u*(p2-p1)+u**2*(p3-p2))
            d2=6*((1-u)*(p2-2*p1+p0)+u*(p3-2*p2+p1))
            speed2=np.sum(d1*d1,axis=1)
            curvature=float(np.max(np.abs(d1[:,0]*d2[:,1]-d1[:,1]*d2[:,0])/np.maximum(speed2,1e-12)**1.5)) if np.all(speed2>1e-8) else float('inf')
            reason=_progress_rejection(path,backtrack_budget=backtrack_budget,max_length_ratio=max_length_ratio)
            if reason is None and np.max(np.abs(path[:,1]))>1.0:
                reason="Trajectory leaves the 1 m analysis band"
            if reason is None and curvature>max_curvature:
                reason="Excessive curvature"
            clearance=None
            if reason is None:
                clearance=float(np.sqrt(np.min(np.sum((path[:,None,:]-obstacles[None,:,:])**2,axis=2))))
                if clearance<robot_radius+safety_margin:
                    reason="Too close to measured returns"
            packed=tuple(tuple(map(float,x)) for x in path)
            trials.append(TrajectoryResult(reason is None, reason or "Smooth to destination (observed-points only)",packed,clearance,curvature))
    good=sorted((x for x in trials if x.feasible),key=lambda x:(x.min_observed_clearance,-x.max_curvature),reverse=True)
    return TrajectorySearchResult(tuple(good),tuple(x for x in trials if not x.feasible))


def search_with_fallback(
    observation: EdgeObservation,
    robot_position: Point,
    *,
    robot_yaw: Optional[float] = None,
    robot_radius: float = 0.089,
    safety_margin: float = 0.04,
    max_curvature: float = 5.0,
    backtrack_budget: float = 0.05,
    max_length_ratio: float = 3.0,
    max_fallback_pairs: int = 100,
    diagnostics: Optional[dict] = None,
) -> tuple[TrajectorySearchResult, str]:
    """Search destination-reaching routes first, then local passage maneuvers.

    Returns the first stage with a feasible sampled route and its stage label.
    Local paths do not certify whole-edge traversability. This function does
    not issue velocity commands or execute an in-place rotation. The optional
    diagnostics mapping contains per-stage search results for debugging.
    """
    # Prefer destination-reaching motion independent of opening tracking.
    direct = search_direct_destination(
        observation, robot_position, robot_yaw=robot_yaw,
        robot_radius=robot_radius, safety_margin=safety_margin)
    if diagnostics is not None:
        diagnostics["direct destination"] = direct
    if direct.feasible:
        return direct, "direct destination"
    smooth = search_smooth_destination(
        observation, robot_position, robot_yaw=robot_yaw,
        robot_radius=robot_radius, safety_margin=safety_margin,
        max_curvature=max_curvature, backtrack_budget=backtrack_budget,
        max_length_ratio=max_length_ratio)
    if diagnostics is not None:
        diagnostics["smooth destination"] = smooth
    if smooth.feasible:
        return smooth, "smooth destination"
    # If the straight segment is observed-clear but requires a stop/rotation,
    # defer the motion request to the explorer/controller. Do NOT encode a
    # stationary rotation as a continuous forward trajectory.
    if robot_yaw is not None:
        dest_align = search_rotate_to_destination(
            observation, robot_position, robot_yaw=robot_yaw,
            robot_radius=robot_radius, safety_margin=safety_margin)
        if dest_align is not None:
            if diagnostics is not None:
                diagnostics["destination alignment"] = dest_align
            return TrajectorySearchResult((), direct.rejected+smooth.rejected), "rotate then destination"
    # Apply the same opening-progress policy to EVERY search stage.
    # In particular, neither the single-gap nor two-waypoint fallback should
    # resurrect an opening already cleared by the robot's rear footprint.
    full_observation = observation
    observation = select_forward_observation(
        observation, robot_position, robot_radius=robot_radius,
    )
    # Reaching a local opening waypoint is distinct from clearing the obstacle.
    # First try continuing in the current heading, without forcing an artificial
    # 2-cm lateral correction to the passage midpoint.
    goal_continuation = search_goal_reached_continuation(
        full_observation, robot_position, robot_yaw=robot_yaw,
        robot_radius=robot_radius, safety_margin=safety_margin)
    if diagnostics is not None:
        diagnostics["goal-reached continuation"] = goal_continuation
    if goal_continuation.feasible:
        return goal_continuation, "goal-reached continuation only"
    # When the robot reaches a narrow throat, continuing locally is more
    # useful than forcing a full-edge cubic through an already-near gap.
    traverse = search_traverse_trajectories(
        observation, robot_position, robot_yaw=robot_yaw,
        robot_radius=robot_radius, safety_margin=safety_margin,
        max_curvature=max_curvature, backtrack_budget=backtrack_budget,
        max_length_ratio=max_length_ratio)
    if diagnostics is not None:
        diagnostics["near-opening traverse"] = traverse
    if traverse.feasible:
        return traverse, "near-opening traverse only"
    # The immediate planning objective is the next observed connected passage.
    local, local_passage = search_next_opening(
        observation, robot_position, robot_yaw=robot_yaw,
        robot_radius=robot_radius, safety_margin=safety_margin,
        max_curvature=max_curvature, backtrack_budget=backtrack_budget,
        max_length_ratio=max_length_ratio)
    if diagnostics is not None:
        diagnostics["next opening"] = local
        diagnostics["selected_passage"] = local_passage
    if local.feasible:
        robot_s = float(points_to_edge_frame(np.asarray([robot_position]),
            observation.start, observation.end)[0, 0])
        return local, ("near-opening traverse only" if local_passage is not None
                       and local_passage.s_end <= robot_s + 0.10
                       else "next opening only")
    # After clearing the final observed passage, advance incrementally toward
    # the graph destination rather than forcing a full-edge Bezier.
    final = search_final_approach(
        full_observation, robot_position, robot_yaw=robot_yaw,
        robot_radius=robot_radius, safety_margin=safety_margin,
        max_curvature=max_curvature, backtrack_budget=backtrack_budget,
        max_length_ratio=max_length_ratio)
    if diagnostics is not None:
        diagnostics["final approach"] = final
    if final.feasible:
        return final, "final approach only"
    # Legacy searches remain diagnostic fallbacks, not the primary objective.
    primary = search_candidate_trajectories(
        observation, robot_position, robot_yaw=robot_yaw,
        robot_radius=robot_radius, safety_margin=safety_margin,
        max_curvature=max_curvature, backtrack_budget=backtrack_budget,
        max_length_ratio=max_length_ratio)
    if diagnostics is not None:
        diagnostics["single-gap full-edge"] = primary
    if primary.feasible:
        return primary, "single-gap full-edge"
    approach = search_approach_trajectories(
        observation, robot_position, robot_yaw=robot_yaw,
        robot_radius=robot_radius, safety_margin=safety_margin,
        max_curvature=max_curvature, backtrack_budget=backtrack_budget,
        max_length_ratio=max_length_ratio)
    if diagnostics is not None:
        diagnostics["short approach"] = approach
    if approach.feasible:
        return TrajectorySearchResult(approach.feasible,
                                      traverse.rejected + primary.rejected + approach.rejected), "short approach only"
    fallback = search_two_waypoint_trajectories(
        observation, robot_position, robot_yaw=robot_yaw,
        robot_radius=robot_radius, safety_margin=safety_margin,
        max_curvature=max_curvature, max_pairs=max_fallback_pairs,
        backtrack_budget=backtrack_budget, max_length_ratio=max_length_ratio)
    if diagnostics is not None:
        diagnostics["two-waypoint full-edge"] = fallback
    return TrajectorySearchResult(fallback.feasible,
                                  traverse.rejected + primary.rejected + approach.rejected + fallback.rejected), "two-waypoint full-edge fallback"



@dataclass(frozen=True)
class AlignmentManeuver:
    """Experimental translation -> stopped rotation -> translation plan.

    Both paths are in edge (s,t) coordinates. `departure_yaw` is in the
    world/odom XY frame. This is NOT an executable velocity command.
    """
    approach: TrajectoryResult
    alignment_st: Point
    departure_yaw: float
    continuation_st: tuple[Point, ...]
    clearance: float
    next_gap: GapCandidate


def search_intermediate_alignment(
    observation: EdgeObservation,
    robot_position: Point,
    *,
    robot_yaw: float,
    robot_radius: float = 0.089,
    safety_margin: float = 0.04,
    max_curvature: float = 8.0,
    max_approaches: int = 40,
    min_continuation: float = 0.10,
    continuation_step: float = 0.015,
) -> Optional[AlignmentManeuver]:
    """Find a short approach, a stationary turn, and an observed-clear continuation.

    Select a short approach with sufficient clearance at its endpoint, then
    aim at a *subsequent* observed opening. Reject any straight continuation
    that comes too close to LiDAR returns. Unknown space remains unchecked.
    This proposal checks observed-return clearance; it does not certify edge passability.
    """
    if (not np.isfinite([robot_yaw, robot_radius, safety_margin, max_curvature,
                         min_continuation, continuation_step]).all()
            or robot_radius <= 0 or safety_margin < 0 or max_curvature <= 0
            or min_continuation <= 0 or continuation_step <= 0
            or max_approaches < 1):
        raise ValueError('Invalid intermediate-alignment settings')
    obstacle_points = observation.obstacle_points_st
    if len(obstacle_points) == 0:
        return None
    radius = robot_radius + safety_margin
    forward = select_forward_observation(observation, robot_position,
                                         robot_radius=robot_radius)
    approaches = search_approach_trajectories(
        forward, robot_position, robot_yaw=robot_yaw,
        robot_radius=robot_radius, safety_margin=safety_margin,
        max_curvature=max_curvature, max_gap_candidates=30)
    if not approaches.feasible:
        return None
    gaps = find_feasible_gap_candidates(forward, robot_radius=robot_radius,
                                        safety_margin=safety_margin, samples_per_gap=3)
    edge_yaw = float(np.arctan2(observation.end[1] - observation.start[1],
                                observation.end[0] - observation.start[0]))
    best = None
    best_score = -float('inf')
    for approach in approaches.feasible[:max_approaches]:
        if len(approach.path_st) < 2:
            continue
        end = np.asarray(approach.path_st[-1], dtype=float)
        # Room to rotate in place: circular model, same horizontal footprint.
        turn_clearance = float(np.min(np.linalg.norm(obstacle_points - end, axis=1)))
        if turn_clearance < radius:
            continue
        targets = sorted((g for g in gaps if g.slice_s > end[0] + min_continuation),
                         key=lambda g: (g.slice_s, abs(g.t - end[1])))
        for gap in targets[:15]:
            destination = np.array([gap.s, gap.t], dtype=float)
            delta = destination - end
            distance = float(np.linalg.norm(delta))
            if distance < min_continuation:
                continue
            n = max(2, int(np.ceil(distance / continuation_step)) + 1)
            path = end[None, :] + np.linspace(0., 1., n)[:, None] * delta[None, :]
            if np.max(np.abs(path[:, 1])) > 1.0 or np.any(np.diff(path[:, 0]) < -1e-8):
                continue
            minimum = float(np.sqrt(np.min(np.sum(
                (path[:, None, :] - obstacle_points[None, :, :]) ** 2, axis=2))))
            if minimum < radius:
                continue
            departure_yaw = edge_yaw + float(np.arctan2(delta[1], delta[0]))
            departure_yaw = float(np.arctan2(np.sin(departure_yaw), np.cos(departure_yaw)))
            combined_clearance = min(minimum, turn_clearance,
                                     approach.min_observed_clearance or minimum)
            # Reward forward progress, not only minimum clearance; cap nearby options.
            score = 2.0 * (destination[0] - end[0]) + combined_clearance - 0.02 * abs(destination[1])
            if score > best_score:
                best_score = score
                best = AlignmentManeuver(
                    approach=approach,
                    alignment_st=(float(end[0]), float(end[1])),
                    departure_yaw=departure_yaw,
                    continuation_st=tuple((float(pt[0]), float(pt[1])) for pt in path),
                    clearance=combined_clearance,
                    next_gap=gap,
                )
    return best

@dataclass(frozen=True)
class RotateNowManeuver:
    """Stop at the present position, rotate, then attempt a short translation.

    The proposed translation is in edge (s,t) coordinates. This is diagnostic
    geometry, not a motor command or a guarantee that unknown space is clear.
    """
    alignment_st: Point
    departure_yaw: float
    continuation_st: tuple[Point, ...]
    clearance: float
    next_gap: GapCandidate


def search_rotate_now(
    observation: EdgeObservation,
    robot_position: Point,
    *,
    robot_yaw: float,
    robot_radius: float = 0.089,
    safety_margin: float = 0.04,
    min_forward_progress: float = 0.10,
    max_lookahead: float = 0.70,
    continuation_step: float = 0.015,
) -> Optional[RotateNowManeuver]:
    """Try a rotation at the CURRENT position followed by an observed-clear ray.

    This is a fallback when continuous curved translations are not accepted.
    Candidate openings must lie forward in edge s; any point behind the robot
    or an occluded/unknown region is not thereby certified free.
    """
    settings = (robot_yaw, robot_radius, safety_margin, min_forward_progress,
                max_lookahead, continuation_step)
    if (not np.isfinite(settings).all() or robot_radius <= 0
            or safety_margin < 0 or min_forward_progress <= 0
            or max_lookahead < min_forward_progress or continuation_step <= 0):
        raise ValueError("Invalid rotate-now settings")
    obstacles = observation.obstacle_points_st
    if len(obstacles) == 0:
        return None
    start = points_to_edge_frame(
        np.asarray([robot_position], dtype=float), observation.start, observation.end
    )[0]
    clearance_needed = robot_radius + safety_margin
    at_start = float(np.min(np.linalg.norm(obstacles - start, axis=1)))
    if at_start < clearance_needed:
        return None

    # Use the same forward-progress filter as the other search stages.
    forward = select_forward_observation(observation, robot_position,
                                         robot_radius=robot_radius)
    candidates = find_feasible_gap_candidates(
        forward, robot_radius=robot_radius, safety_margin=safety_margin,
        samples_per_gap=5,
    )
    eligible = [gap for gap in candidates
                if min_forward_progress <= gap.s-start[0] <= max_lookahead]
    edge_yaw = float(np.arctan2(observation.end[1]-observation.start[1],
                                observation.end[0]-observation.start[0]))
    best = None
    best_score = -float('inf')
    for gap in eligible:
        endpoint = np.array([gap.s, gap.t], dtype=float)
        delta = endpoint-start
        distance = float(np.linalg.norm(delta))
        if distance <= 0:
            continue
        n = max(2, int(np.ceil(distance / continuation_step))+1)
        path = start[None,:] + np.linspace(0.0,1.0,n)[:,None]*delta[None,:]
        if np.any(np.diff(path[:,0]) < -1e-8) or np.max(np.abs(path[:,1])) > 1.0:
            continue
        observed_clearance = float(np.min(np.linalg.norm(
            path[:,None,:]-obstacles[None,:,:], axis=2)))
        if observed_clearance < clearance_needed:
            continue
        desired_yaw = edge_yaw + float(np.arctan2(delta[1],delta[0]))
        desired_yaw = float(np.arctan2(np.sin(desired_yaw),np.cos(desired_yaw)))
        yaw_change = abs(float(np.arctan2(np.sin(desired_yaw-robot_yaw),
                                         np.cos(desired_yaw-robot_yaw))))
        # Reward useful progress and clearance; mildly penalize large rotations.
        score = 2.0*(endpoint[0]-start[0]) + observed_clearance - 0.05*yaw_change
        if score > best_score:
            best_score = score
            best = RotateNowManeuver(
                alignment_st=(float(start[0]),float(start[1])),
                departure_yaw=desired_yaw,
                continuation_st=tuple((float(s),float(t)) for s,t in path),
                clearance=min(at_start,observed_clearance),
                next_gap=gap,
            )
    return best


@dataclass(frozen=True)
class HeadingAlignmentSearch:
    """Debug-only hypothesized rotate-then-replan outcome.

    `continuation` is a sampled curve assuming the robot *has already rotated*
    to `departure_yaw`. The real robot must re-scan after the rotation.
    """
    departure_yaw: float
    yaw_change: float
    alignment_st: Point
    continuation: TrajectoryResult
    headings_tested: int
    headings_with_paths: int
    heading_rejections: tuple[tuple[float, str], ...]


def search_rotate_then_approach(
    observation: EdgeObservation,
    robot_position: Point,
    *,
    robot_yaw: float,
    robot_radius: float = 0.089,
    safety_margin: float = 0.04,
    max_curvature: float = 8.0,
    heading_offsets: tuple[float, ...] = (-0.5, -0.25, 0., 0.25, 0.5),
    max_headings: int = 15,
) -> Optional[HeadingAlignmentSearch]:
    """Try curved local approaches from alternative hypothetical robot headings.

    Rotation happens at the present position, followed by reobservation before
    executing any translation. Only measured LiDAR returns are checked.
    This function does not issue commands and is not an edge-passability proof.
    """
    from collections import Counter
    values=(robot_yaw,robot_radius,safety_margin,max_curvature)
    if (not np.isfinite(values).all() or robot_radius<=0 or safety_margin<0
            or max_curvature<=0 or max_headings<1 or not np.isfinite(heading_offsets).all()):
        raise ValueError("Invalid hypothetical alignment settings")
    obstacles=observation.obstacle_points_st
    if not len(obstacles):
        return None
    start=points_to_edge_frame(np.asarray([robot_position],dtype=float),
                               observation.start,observation.end)[0]
    required=robot_radius+safety_margin
    if float(np.min(np.linalg.norm(obstacles-start,axis=1)))<required:
        return None
    forward=select_forward_observation(observation,robot_position,robot_radius=robot_radius)
    options=find_feasible_gap_candidates(forward,robot_radius=robot_radius,
                                          safety_margin=safety_margin,samples_per_gap=3)
    ahead=[o for o in options if 0.10 <= o.s-start[0] <= 0.8]
    if not ahead:
        return None
    edge_yaw=float(np.arctan2(observation.end[1]-observation.start[1],
                              observation.end[0]-observation.start[0]))
    seeds=[edge_yaw]
    for gap in sorted(ahead,key=lambda g:g.s)[:6]:
        seeds.append(edge_yaw+float(np.arctan2(gap.t-start[1],gap.s-start[0])))
    def normalize(angle): return float(np.arctan2(np.sin(angle),np.cos(angle)))
    headings=[]; seen=set()
    for seed in seeds:
        for offset in heading_offsets:
            yaw=normalize(seed+offset)
            key=round(yaw,3)
            if key not in seen:
                headings.append(yaw); seen.add(key)
    headings.sort(key=lambda yaw:abs(normalize(yaw-robot_yaw)))
    headings=headings[:max_headings]
    best=None; best_score=-float('inf'); success_count=0; errors=[]
    for yaw in headings:
        # Preserve all original obstacle returns even though target slices are filtered.
        result=search_approach_trajectories(
            forward,robot_position,robot_yaw=yaw,
            robot_radius=robot_radius,safety_margin=safety_margin,
            max_curvature=max_curvature,
        )
        if not result.feasible:
            counts=Counter(r.reason for r in result.rejected)
            reason=counts.most_common(1)[0][0] if counts else 'No candidate'
            errors.append((yaw,reason))
            continue
        success_count+=1
        for trial in result.feasible[:5]:
            end=np.asarray(trial.path_st[-1])
            change=abs(normalize(yaw-robot_yaw))
            score=(2.0*(end[0]-start[0]) + .4*trial.min_observed_clearance
                   - .005*trial.max_curvature - .07*change)
            if score>best_score:
                best_score=score; best=(yaw,trial)
    if best is None: return None
    yaw,trial=best
    return HeadingAlignmentSearch(
        departure_yaw=yaw,yaw_change=normalize(yaw-robot_yaw),
        alignment_st=(float(start[0]),float(start[1])),continuation=trial,
        headings_tested=len(headings),headings_with_paths=success_count,
        heading_rejections=tuple(errors),
    )


# Diagnostic-only state. The existing planner's forward filter is unchanged.
@dataclass(frozen=True)
class OpeningDiagnostic:
    slice_s: float
    gap_left: float
    gap_right: float
    usable_left: float
    usable_right: float
    state: str
    passed_by_longitudinal_rule: bool
    observed_entry: bool


class OpeningProgressTracker:
    """Track observed entry into openings on one directed edge.

    This diagnostic tracks slice/gap identity approximately across scans. It
    is NOT a safety-certified passage detector. A missed observation cannot
    prove that an opening was not traversed.
    """

    def __init__(self, robot_radius=0.089, safety_margin=0.04,
                 passed_tolerance=0.03, entry_window=0.06,
                 matching_tolerance=0.16):
        self.robot_radius = robot_radius
        self.safety_margin = safety_margin
        self.passed_tolerance = passed_tolerance
        self.entry_window = entry_window
        self.matching_tolerance = matching_tolerance
        self.reset()

    def reset(self):
        self.edge = None
        self.entered = {}  # (quantized slice index, approximate midpoint) -> bool

    def update(self, observation: EdgeObservation, robot_position: Point):
        edge = (observation.start, observation.end)
        if self.edge != edge:
            self.reset()
            self.edge = edge
        robot_s, robot_t = points_to_edge_frame(
            np.asarray([robot_position], dtype=float),
            observation.start, observation.end,
        )[0]
        robot_s, robot_t = float(robot_s), float(robot_t)
        clearance = self.robot_radius + self.safety_margin
        results = []
        for section in observation.slices:
            for gap in section.gaps:
                left = gap.left + clearance
                right = gap.right - clearance
                if left > right:
                    continue
                # Slice centers are fixed by analyze_edge's sampling step.
                slice_key = round(section.s, 3)
                possible_keys = [k for k in self.entered
                                 if k[0] == slice_key and
                                 abs(k[1] - gap.midpoint) <= self.matching_tolerance]
                key = min(possible_keys,
                          key=lambda k: abs(k[1] - gap.midpoint)) if possible_keys else (slice_key, round(gap.midpoint, 3))
                inside_lateral = left <= robot_t <= right
                near_slice = abs(robot_s - section.s) <= self.entry_window
                if inside_lateral and near_slice:
                    self.entered[key] = True
                entered = self.entered.get(key, False)
                passed = robot_s - self.robot_radius - self.passed_tolerance > section.s
                if passed:
                    state = 'CLEARED' if entered else 'OFF_PATH'
                elif near_slice and inside_lateral:
                    state = 'TRAVERSING'
                elif robot_s < section.s - self.entry_window:
                    state = 'AHEAD'
                else:
                    state = 'MISALIGNED'
                results.append(OpeningDiagnostic(
                    section.s, gap.left, gap.right, left, right,
                    state, passed, entered,
                ))
        return (robot_s, robot_t), tuple(results)


@dataclass(frozen=True)
class RepositionManeuver:
    """Two individually checked translation legs separated by a stationary turn.

    All paths and the rotation point are in edge ST coordinates; yaws are in
    the same fixed XY frame as the edge. Diagnostic proposal ONLY.
    """
    first_leg_st: tuple[Point, ...]
    rotation_st: Point
    departure_yaw: float
    second_leg_st: tuple[Point, ...]
    observed_clearance: float
    next_gap: GapCandidate


def search_reposition_then_rotate(
    observation: EdgeObservation,
    robot_position: Point,
    *,
    robot_yaw: float,
    robot_radius: float = 0.089,
    safety_margin: float = 0.04,
    min_lateral_shift: float = 0.04,
    max_first_leg: float = 0.22,
    max_second_leg: float = 0.65,
    step: float = 0.015,
    diagnostics: Optional[dict] = None,
) -> Optional[RepositionManeuver]:
    """Try move in current heading -> stop/rotate -> move toward forward gap.

    Unlike the full-edge planner, the first leg MAY have negligible progress
    along the graph edge. Both legs must clear measured obstacle points. This
    does not validate occluded space, dynamic control, or full-edge traversal.
    """
    settings=(robot_yaw, robot_radius, safety_margin, min_lateral_shift,
              max_first_leg, max_second_leg, step)
    if (not np.isfinite(settings).all() or robot_radius <= 0 or safety_margin < 0
            or min_lateral_shift < 0 or max_first_leg <= 0
            or max_second_leg <= 0 or step <= 0):
        raise ValueError('Invalid reposition settings')
    counters={'no_obstacles':0, 'start_clearance':0, 'no_forward_gap':0,
              'first_leg_clearance':0, 'rotation_clearance':0,
              'second_leg_clearance':0, 'insufficient_shift':0,
              'inadequate_progress':0}
    def fail(key):
        counters[key]+=1
    def finish(value):
        if diagnostics is not None:
            diagnostics.update(counters)
        return value
    obstacles=observation.obstacle_points_st
    if len(obstacles)==0:
        fail('no_obstacles'); return finish(None)
    start=points_to_edge_frame(np.asarray([robot_position],dtype=float),
                               observation.start,observation.end)[0]
    required=robot_radius+safety_margin
    def clearance(path):
        return float(np.min(np.linalg.norm(path[:,None,:]-obstacles[None,:,:],axis=2)))
    if clearance(start[None,:]) < required:
        fail('start_clearance'); return finish(None)
    forward=select_forward_observation(observation,robot_position,robot_radius=robot_radius)
    gaps=find_feasible_gap_candidates(forward,robot_radius=robot_radius,
                                       safety_margin=safety_margin,samples_per_gap=3)
    gaps=[g for g in gaps if start[0]+.06 < g.s <= min(observation.length,start[0]+.8)]
    if not gaps:
        fail('no_forward_gap'); return finish(None)
    edge_yaw=float(np.arctan2(observation.end[1]-observation.start[1],
                              observation.end[0]-observation.start[0]))
    theta=robot_yaw-edge_yaw
    direction=np.array([np.cos(theta),np.sin(theta)])
    best=None; best_score=-np.inf
    for shift in np.linspace(.04,max_first_leg,7):
        station=start+shift*direction
        # No large reverse motions or lateral excursions outside analysis band.
        if station[0] < start[0]-.03 or abs(station[1]) > 1.0:
            fail('inadequate_progress'); continue
        n=max(2,int(np.ceil(shift/step))+1)
        first=start[None,:]+np.linspace(0.,1.,n)[:,None]*(station-start)
        first_clear=clearance(first)
        if first_clear < required:
            fail('first_leg_clearance'); continue
        if clearance(station[None,:]) < required:
            fail('rotation_clearance'); continue
        for gap in gaps:
            goal=np.array([gap.s,gap.t]); delta=goal-station
            dist=float(np.linalg.norm(delta))
            if delta[0] < .06 or dist > max_second_leg:
                fail('inadequate_progress'); continue
            if abs(station[1]-start[1]) < min_lateral_shift:
                fail('insufficient_shift'); continue
            m=max(2,int(np.ceil(dist/step))+1)
            second=station[None,:]+np.linspace(0.,1.,m)[:,None]*delta
            second_clear=clearance(second)
            if second_clear < required:
                fail('second_leg_clearance'); continue
            yaw=edge_yaw+float(np.arctan2(delta[1],delta[0]))
            yaw=float(np.arctan2(np.sin(yaw),np.cos(yaw)))
            turn=abs(float(np.arctan2(np.sin(yaw-robot_yaw),np.cos(yaw-robot_yaw))))
            margin=min(first_clear,second_clear)
            score=2.0*(goal[0]-start[0])+0.4*margin-0.15*dist-0.06*turn
            if score>best_score:
                best_score=score
                best=RepositionManeuver(
                    first_leg_st=tuple(map(lambda x:(float(x[0]),float(x[1])),first)),
                    rotation_st=(float(station[0]),float(station[1])),
                    departure_yaw=yaw,
                    second_leg_st=tuple(map(lambda x:(float(x[0]),float(x[1])),second)),
                    observed_clearance=margin,
                    next_gap=gap,
                )
    return finish(best)


# ---------------------------------------------------------------------------
# High-level perception interface used by the explorer and future navigator.
# Keep the individual search_* functions available for unit tests and research,
# but avoid duplicating fallback ordering in each caller.
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class MotionPlan:
    """A *proposal*, not an executable controller command.

    ``segments_st`` contains distinct robot-center translation polylines in
    edge-local (s,t) meters. Never join them across a required rotation.
    ``rotation_st`` is the stop position in the same frame, or None.
    ``departure_yaw`` is in radians in the fixed world/odom XY frame.
    ``mode`` describes the selected search strategy (not a ROS state).
    ``requires_rescan`` means the controller must re-observe before translating
    after the proposed rotation. No return value certifies unknown free space.
    """
    mode: str
    segments_st: tuple[tuple[Point, ...], ...] = ()
    rotation_st: Optional[Point] = None
    departure_yaw: Optional[float] = None
    requires_rescan: bool = False
    clearance: Optional[float] = None
    candidate: Optional[GapCandidate] = None

    @property
    def requires_rotation(self) -> bool:
        """Whether a separate stop-and-rotate phase is proposed."""
        return self.rotation_st is not None


@dataclass(frozen=True)
class PlanningReport:
    """Unified output: primary search, optional recovery and diagnostics."""
    plan: Optional[MotionPlan]
    search: TrajectorySearchResult
    mode: str
    diagnostics: dict


def plan_motion(
    observation: EdgeObservation,
    robot_position: Point,
    *,
    robot_yaw: float,
    robot_radius: float = 0.089,
    safety_margin: float = 0.03,
    max_curvature: float = 8.0,
) -> PlanningReport:
    """Return one destination-first, clearance-checked motion proposal.

    This is the recommended public orchestration API. It first uses
    ``search_with_fallback`` (destination, passage, short motion), then tries
    rotation/reposition primitives only when that search found no forward path.
    Each fallback retains its own geometric constraints. A rotation and its
    hypothetical continuation are separate phases and require fresh scanning.
    Observed-point clearance is not a guarantee of collision-free unknown space.
    """
    diagnostics = {}
    kwargs = dict(robot_yaw=robot_yaw, robot_radius=robot_radius,
                  safety_margin=safety_margin, max_curvature=max_curvature)
    search, mode = search_with_fallback(
        observation, robot_position, diagnostics=diagnostics, **kwargs)
    if search.feasible:
        best = search.feasible[0]
        return PlanningReport(MotionPlan(mode, (best.path_st,),
                                         clearance=best.min_observed_clearance,
                                         candidate=best.candidate),
                              search, mode, diagnostics)

    aligned = diagnostics.get("destination alignment")
    if aligned is not None:
        plan = MotionPlan("rotate then destination", (aligned.path_st,),
                          tuple(map(float, points_to_edge_frame(
                              np.asarray([robot_position]), observation.start,
                              observation.end)[0])), aligned.departure_yaw,
                          True, aligned.clearance)
        return PlanningReport(plan, search, plan.mode, diagnostics)

    common = dict(robot_yaw=robot_yaw, robot_radius=robot_radius,
                  safety_margin=safety_margin)
    rotate = search_rotate_now(observation, robot_position, **common)
    if rotate is not None:
        plan = MotionPlan("rotate now", (rotate.continuation_st,),
                          rotate.alignment_st, rotate.departure_yaw, True,
                          rotate.clearance, rotate.next_gap)
        return PlanningReport(plan, search, plan.mode, diagnostics)

    hypothetical = search_rotate_then_approach(
        observation, robot_position, **kwargs)
    if hypothetical is not None:
        plan = MotionPlan("align first", (hypothetical.continuation.path_st,),
                          hypothetical.alignment_st,
                          hypothetical.departure_yaw, True,
                          hypothetical.continuation.min_observed_clearance,
                          hypothetical.continuation.candidate)
        diagnostics["alignment headings tested"] = hypothetical.headings_tested
        diagnostics["alignment headings with paths"] = hypothetical.headings_with_paths
        return PlanningReport(plan, search, plan.mode, diagnostics)

    intermediate = search_intermediate_alignment(
        observation, robot_position, **common, max_curvature=max_curvature)
    if intermediate is not None:
        plan = MotionPlan("intermediate alignment",
                          (intermediate.approach.path_st,
                           intermediate.continuation_st),
                          intermediate.alignment_st, intermediate.departure_yaw,
                          True, intermediate.clearance, intermediate.next_gap)
        return PlanningReport(plan, search, plan.mode, diagnostics)

    reposition_diagnostics = {}
    reposition = search_reposition_then_rotate(
        observation, robot_position, diagnostics=reposition_diagnostics, **common)
    diagnostics["reposition reasons"] = reposition_diagnostics
    if reposition is not None:
        plan = MotionPlan("lateral reposition",
                          (reposition.first_leg_st, reposition.second_leg_st),
                          reposition.rotation_st, reposition.departure_yaw,
                          True, reposition.observed_clearance,
                          reposition.next_gap)
        return PlanningReport(plan, search, plan.mode, diagnostics)
    return PlanningReport(None, search, mode, diagnostics)
