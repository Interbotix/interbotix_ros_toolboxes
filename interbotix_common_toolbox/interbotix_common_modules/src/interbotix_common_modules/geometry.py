import math
from geometry_msgs.msg import Quaternion

def quaternion_is_valid(quat, tol=10e-3):
    """Tests if a quaternion is valid
    
    :param quat: Quaternion to check validity of
    :type quat: geometry_msgs.msg.Quaternion
    :param tol: tolerance with which to check validity
    :return: `True` if quaternion is valid, `False` otherwise
    :rtype: bool
    """
    valid = abs((quat.w * quat.w
        + quat.x * quat.x
        + quat.y * quat.y
        + quat.z * quat.z) - 1.0) < tol

    return valid