import rospy
from xarm_msgs.srv import *
from interbotix_ux_modules.core import InterbotixRobotUXCore

### @brief Standalone Module to control an Universal Factory Gripper
### @param robot_model - Universal Factory Xarm model (ex. 'uxarm5' or 'uxarm6')
### @param robot_name - defaults to value given to 'robot_model'; this can be customized if controlling two of the same arms from one computer (like 'arm1/uxarm6' and 'arm2/uxarm6')
### @param mode - Universal Factory Xarm Mode; can be 0 (pose mode), 1 (servo mode), or 2 (teach mode)
### @param wait_for_finish - set to True to have the function wait until arm is finished moving before returning (only applicable in Mode 0); otherwise, set to False
### @param ee_offset - transform from the current end-effector frame to the desired end-effector frame [x, y, z, roll, pitch, yaw] (units are meters/radians)
### @param init_node - set to True if the InterbotixRobotUXCore class should initialize the ROS node - this is the most Pythonic approach; to incorporate a robot into an existing ROS node though, set to False
### @param joint_state_topic - desired joint_state topic name to subscribe to; note that the name is resolved relative to the 'robot_name' namespace
### @param pulse_vel - desired gripper speed [1 - 5000]
### @param pulse - desired initial gripper pulse from 0 (closed) to 850 (fully open)
### @param gripper_type - type of gripper being used; currently, only "gripper" for the standard gripper and 'None' for no gripper are supported
class InterbotixGripperUX(object):
    def __init__(self, robot_model, robot_name=None, mode=0, wait_for_finish=True, ee_offset=None, init_node=True, joint_state_topic="joint_states", pulse_vel=1500, pulse=850, gripper_type="gripper"):
        self.ux = InterbotixRobotUXCore(robot_model, robot_name, mode, wait_for_finish, ee_offset, init_node, joint_state_topic)
        if gripper_type is "gripper":
            self.gripper = InterbotixGripperUXInterface(self.ux, pulse_vel, pulse)

### @brief Definition of the Interbotix Gripper Module
### @param core - reference to the InterbotixRobotUXCore class containing the internal ROS plumbing that drives the Python API
### @param pulse_vel - desired gripper speed ranging from 1 (slowest) to 5000 (fastest)
### @pram pulse - desired initial gripper position ranging from 0 (closed) to 850 (open)
class InterbotixGripperUXInterface(object):

    def __init__(self, core, pulse_vel=1500, pulse=850):
        self.core = core
        rospy.wait_for_service("/" + self.core.robot_name + "/gripper_move")
        rospy.wait_for_service("/" + self.core.robot_name + "/gripper_config")
        rospy.wait_for_service("/" + self.core.robot_name + "/gripper_state")
        self.srv_gripper_move = rospy.ServiceProxy("/" + self.core.robot_name + "/gripper_move", GripperMove)
        self.srv_gripper_config = rospy.ServiceProxy("/" + self.core.robot_name + "/gripper_config", GripperConfig)
        self.srv_gripper_state = rospy.ServiceProxy("/" + self.core.robot_name + "/gripper_state", GripperState)
        self.config(pulse_vel)
        self.move(pulse)
        rospy.loginfo("Initializing InterbotixGripperUXInterface...")
        rospy.loginfo("\nGripper Pulse Vel: %s\nGripper Pulse: %d\n" % (pulse_vel, pulse))

    ### @brief Move gripper
    ### @param pulse - value from 0 (closed) - 850 (open)
    ### @param delay - number of seconds to wait before returning control to the user
    ### @return ret - error code (0 means all's good)
    def move(self, pulse, delay=1.0):
        resp = self.srv_gripper_move(pulse)
        if (resp.ret != 0): rospy.loginfo(resp.message)
        rospy.sleep(delay)
        return resp.ret

    ### @brief Configure gripper speed
    ### @param pulse_vel - value from 1 (slowest) - 5000 (fastest)
    ### @return ret - error code (0 means all's good)
    def config(self, pulse_vel):
        resp = self.srv_gripper_config(pulse_vel)
        rospy.loginfo(resp.message)
        return resp.ret

    ### @brief Get current gripper state
    ### @return state - gripper state [rad]
    def get_state(self):
        resp = self.srv_gripper_state()
        if resp.err_code != 0:
            rospy.loginfo("Error Num: %d" % resp.err_code)
        state = (850 - resp.curr_pos) / 1000.0
        return state

    ### @brief Opens the gripper
    ### @param delay - number of seconds to delay before returning control to the user
    ### @return ret - error code (0 means all's good)
    def open(self, delay=1.0):
        ret = self.move(850, delay)
        return ret

    ### @brief Closes the gripper
    ### @param delay - number of seconds to delay before returning control to the user
    ### @return ret - error code (0 means all's good)
    def close(self, delay=1.0):
        ret = self.move(0, delay)
        return ret
