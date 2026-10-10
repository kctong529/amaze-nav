import numpy as np


def transform_points_2d(points, translation, quaternion):
    """
    Transform Nx2 points using a 3D ROS Transform.

    Args:
        points: Nx2 NumPy array in the source frame.
        translation: geometry_msgs/Vector3
        quaternion: geometry_msgs/Quaternion

    Returns:
        Nx2 points in the target frame.

    Assumes points are measured in the sensor's z=0 plane.
    """
    qx = quaternion.x
    qy = quaternion.y
    qz = quaternion.z
    qw = quaternion.w

    # Full quaternion rotation matrix, retaining XY components.
    r00 = 1 - 2 * (qy * qy + qz * qz)
    r01 = 2 * (qx * qy - qz * qw)

    r10 = 2 * (qx * qy + qz * qw)
    r11 = 1 - 2 * (qx * qx + qz * qz)

    rotation = np.array([
        [r00, r01],
        [r10, r11],
    ])

    translated = points @ rotation.T

    translated[:, 0] += translation.x
    translated[:, 1] += translation.y

    return translated


def points_to_edge_frame(points, start, end):
    """
    Transform world/odom-frame LiDAR points into an
    edge-aligned coordinate system.

    s: distance along the edge, starting at start.
    t: lateral distance from the edge centerline.
       Positive t is to the left of the edge direction.

    Returns:
        Nx2 array containing (s, t) coordinates.
    """
    points = np.asarray(points, dtype=float)
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)

    edge = end - start
    length = np.linalg.norm(edge)

    if length <= 0:
        raise ValueError("Edge endpoints must be distinct")

    direction = edge / length
    normal = np.array([-direction[1], direction[0]])

    relative = points - start

    s = relative @ direction
    t = relative @ normal

    return np.column_stack((s, t))


def extract_slice(edge_points, s0, thickness):
    """
    Select LiDAR points within a cross-sectional strip.

    s0: position along the edge in metres
    thickness: total strip thickness in metres
    """
    if thickness <= 0:
        raise ValueError("Slice thickness must be positive")

    edge_points = np.asarray(edge_points, dtype=float)

    mask = np.abs(edge_points[:, 0] - s0) <= thickness / 2

    return edge_points[mask]


def cluster_lateral_points(
    slice_points,
    max_gap=0.05,
    min_points=3,
):
    """
    Group nearby lateral LiDAR returns into candidate
    obstacle intervals.

    Parameters
    ----------
    slice_points : array-like, shape (N, 2)
        Points expressed as (s, t).
    max_gap : float
        Maximum lateral separation between neighboring
        points in the same cluster, in metres.
    min_points : int
        Minimum number of points for a retained cluster.

    Returns
    -------
    list[tuple[float, float]]
        Intervals (t_min, t_max), sorted from negative
        to positive t.
    """
    if max_gap <= 0:
        raise ValueError("max_gap must be positive")

    if min_points < 1:
        raise ValueError("min_points must be at least 1")

    points = np.asarray(slice_points, dtype=float)

    if points.size == 0:
        return []

    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError("Expected an Nx2 array")

    # Only the lateral coordinate matters for this step.
    lateral = np.sort(points[:, 1])

    clusters = []
    current = [lateral[0]]

    for t in lateral[1:]:
        if t - current[-1] <= max_gap:
            current.append(t)
        else:
            if len(current) >= min_points:
                clusters.append(
                    (float(current[0]), float(current[-1]))
                )

            current = [t]

    # Don't forget the final cluster.
    if len(current) >= min_points:
        clusters.append(
            (float(current[0]), float(current[-1]))
        )

    return clusters


def find_gaps(intervals):
    """
    Find candidate free intervals between consecutive
    observed obstacle intervals.

    This does not establish that the gaps are collision-free.
    """
    intervals = sorted(intervals, key=lambda pair: pair[0])

    gaps = []

    for (_, obstacle_right), (obstacle_left, _) in zip(
        intervals, intervals[1:]
    ):
        width = obstacle_left - obstacle_right

        if width <= 0:
            continue

        gaps.append({
            "left": float(obstacle_right),
            "right": float(obstacle_left),
            "width": float(width),
            "midpoint": float(
                (obstacle_right + obstacle_left) / 2
            ),
        })

    return gaps
