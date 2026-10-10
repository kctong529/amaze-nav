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
