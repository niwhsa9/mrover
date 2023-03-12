import numpy as np
import collections
from geometry_msgs.msg import Vector3, Quaternion
from typing import Union


def _translation_to_numpy(translation: Vector3) -> np.ndarray:
    return np.array([translation.x, translation.y, translation.z])


def _rotation_to_numpy(rotation: Quaternion) -> np.ndarray:
    return np.array([rotation.x, rotation.y, rotation.z, rotation.w])


def normalized(v):
    norm = np.linalg.norm(v)
    return v / norm


def perpendicular_2d(v):
    if v.shape != (2,) and v.shape != (1, 2) and v.shape != (2, 1):
        raise Exception("vector must be 2D!!!")
    orig_shape = v.shape
    return np.reshape((np.array([-v.flatten()[1], v.flatten()[0]])), orig_shape)


def angle_to_rotate(v1, v2):
    """
    returns the angle you'd need to rotate normalized(v1) by in order to have a vector that is normalized(v2)
    """
    v1_u, v2_u = normalized(v1), normalized(v2)
    smallest_angle = np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
    # Determine the sign of our effort by seeing if we are to the left or to the right of the target
    # This is done by dotting rover_dir and target_dir rotated 90 degrees ccw
    perp_alignment = v2_u[0] * -v1_u[1] + v2_u[1] * v1_u[0]
    sign = np.sign(perp_alignment)
    if sign == 0.0:
        sign = 1
    return smallest_angle * sign


def numpify(msg: Union[Vector3, Quaternion]) -> np.ndarray:
    if msg.__class__ == Vector3:
        return _translation_to_numpy(msg)
    elif msg.__class__ == Quaternion:
        return _rotation_to_numpy(msg)
    else:
        raise Exception("type of msg must be either Vector3 or Quaternion!!!")
