import numpy as np
import math

from geometry_msgs.msg import Quaternion, QuaternionStamped
from tf.transformations import euler_matrix, euler_from_matrix

# Inverts a homogeneous transformation matrix
def transInv(T):
    R, p = T[:3,:3], T[:3, 3]
    Rt = np.array(R).T
    return np.r_[np.c_[Rt, -np.dot(Rt, p)], [[0, 0, 0, 1]]]

# Calculates 2D Rotation Matrix given a desired yaw angle
def yawToRotationMatrix(yaw):
    R_z = np.array([[math.cos(yaw),    -math.sin(yaw)],
                    [math.sin(yaw),    math.cos(yaw)],
                    ])
    return R_z

# Transform a Six Element Pose vector to a Transformation Matrix
def poseToTransformationMatrix(pose):
    mat = np.identity(4)
    mat[:3, :3] = eulerAnglesToRotationMatrix(pose[3:])
    mat[:3, 3] = pose[:3]
    return mat

def eulerAnglesToRotationMatrix(theta):
    """Calculates rotation matrix given euler angles in 'xyz' sequence
    
    :param theta: list of 3 euler angles
    :return: 3x3 rotation matrix equivalent to the given euler angles
    """
    return euler_matrix(theta[0], theta[1], theta[2], axes="sxyz")[:3, :3]

def rotationMatrixToEulerAngles(R):
    """Calculates euler angles given rotation matrix in 'xyz' sequence

    :param R: 3x3 rotation matrix
    :return: list of three euler angles equivalent to the given rotation matrix
    """
    return list(euler_from_matrix(R, axes="sxyz"))

def quaternion_is_valid(quat, tol=10e-3):
    """Tests if a quaternion is valid
    
    :param quat: Quaternion to check validity of
    :type quat: geometry_msgs.msg.Quaternion
    :param tol: tolerance with which to check validity
    :tpe tol: float
    :return: `True` if quaternion is valid, `False` otherwise
    :rtype: bool
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
