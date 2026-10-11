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


def assess_passability(observation: EdgeObservation, robot_position: Point, *,
                       robot_radius: float = 0.089,
                       safety_margin: float = 0.04) -> PassabilityResult:
    """Reserved for evidence-aware corridor and footprint assessment."""
    return PassabilityResult(Passability.UNKNOWN, "Passability assessment not implemented")


def choose_avoidance_target(observation: EdgeObservation, robot_position: Point,
                            nominal_target: Point, *, robot_radius: float = 0.089,
                            safety_margin: float = 0.04) -> TargetResult:
    """Reserved for trajectory-aware lookahead candidate assessment."""
    return TargetResult(None, "Trajectory-aware target selection not implemented", False)
