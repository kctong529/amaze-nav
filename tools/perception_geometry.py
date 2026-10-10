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
