"""
A small library of functions to convert Euler angles to rotation matrices and vice versa
"""

from typing import List, Union
import numpy as np
import math

from geometry_msgs.msg import Quaternion, QuaternionStamped
from tf_transformations import euler_matrix, euler_from_matrix

def transInv(T: np.ndarray) -> np.ndarray:
    """Inverts a homogeneous transformation matrix
    
    :param T: The homogeneous transformation matrix to invert
    :return: The inverted homogeneous transformation matrix
    """
    R, p = T[:3,:3], T[:3, 3]
    Rt = np.array(R).T
    return np.r_[np.c_[Rt, -np.dot(Rt, p)], [[0, 0, 0, 1]]]

def yawToRotationMatrix(yaw: float) -> np.ndarray:
    """Calculates 2D Rotation Matrix given a desired yaw angle
    
    :param yaw: A yaw rotation
    :return: A 2x2 rotation matrix rotated yaw radians
    """
    R_z = np.array([[math.cos(yaw),    -math.sin(yaw)],
                    [math.sin(yaw),    math.cos(yaw)],
                    ])
    return R_z

def poseToTransformationMatrix(pose: List[float]) -> np.ndarray:
    """Transform a Six Element Pose vector to a Transformation Matrix
    
    :param pose: A list of 6 floats in the format [x, y, z, roll, pitch, yaw]
    :return: A 4x4 homogeneous transformation matrix
    """
    mat = np.identity(4)
    mat[:3, :3] = eulerAnglesToRotationMatrix(pose[3:])
    mat[:3, 3] = pose[:3]
    return mat

def eulerAnglesToRotationMatrix(theta: List[float]) -> np.ndarray:
    """Calculates rotation matrix given euler angles in 'xyz' sequence
    
    :param theta: list of 3 euler angles
    :return: 3x3 rotation matrix equivalent to the given euler angles
    """
    return euler_matrix(theta[0], theta[1], theta[2], axes="sxyz")[:3, :3]

def rotationMatrixToEulerAngles(R) -> List[float]:
    """Calculates euler angles given rotation matrix in 'xyz' sequence

    :param R: 3x3 rotation matrix
    :return: list of three euler angles equivalent to the given rotation matrix
    """
    return list(euler_from_matrix(R, axes="sxyz"))

def quaternion_is_valid(
    quat: Union[Quaternion, QuaternionStamped],
    tol: float = 10e-3
) -> bool:
    """Tests if a quaternion is valid
    
    :param quat: Quaternion to check validity of
    :param tol: tolerance with which to check validity
    :return: `True` if quaternion is valid, `False` otherwise
    :raises: `TypeError` if quat is not a valid type
    """
    if isinstance(quat, Quaternion):
        return abs((quat.w * quat.w
            + quat.x * quat.x
            + quat.y * quat.y
            + quat.z * quat.z) - 1.0) < tol
    elif isinstance(quat, QuaternionStamped):
        return abs((quat.quaternion.w * quat.quaternion.w
            + quat.quaternion.x * quat.quaternion.x
            + quat.quaternion.y * quat.quaternion.y
            + quat.quaternion.z * quat.quaternion.z) - 1.0) < tol
    else:
        raise TypeError(
            ("quaternion_is_valid must be given a Quaternion or "
            "QuaternionStamped message. Was given %s." % type(quat)))
