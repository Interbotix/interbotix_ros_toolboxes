import rospy
from interbotix_xs_sdk.msg import JointSingleCommand
from interbotix_xs_modules.core import InterbotixRobotXSCore

### @brief Standalone Module to control an Interbotix Gripper using PWM or Current control
### @param robot_model - Interbotix Arm model (ex. 'wx200' or 'vx300s')
### @param gripper_name - name of the gripper joint as defined in the 'motor_config' yaml file; typically, this is 'gripper'
### @param robot_name - defaults to value given to 'robot_model'; this can be customized to best suit the user's needs
### @param gripper_pressure - fraction from 0 - 1 where '0' means the gripper operates at 'gripper_pressure_lower_limit' and '1' means the gripper operates at 'gripper_pressure_upper_limit'
### @param gripper_pressure_lower_limit - lowest 'effort' that should be applied to the gripper if gripper_pressure is set to 0; it should be high enough to open/close the gripper (~150 PWM or ~400 mA current)
### @param gripper_pressure_upper_limit - largest 'effort' that should be applied to the gripper if gripper_pressure is set to 1; it should be low enough that the motor doesn't 'overload' when gripping an object for a few seconds (~350 PWM or ~900 mA)
### @param init_node - set to True if the InterbotixRobotXSCore class should initialize the ROS node - this is the most Pythonic approach; to incorporate a robot into an existing ROS node though, set to False
### @details - note that this module doesn't really have any use case except in controlling just the gripper joint on an Interbotix Arm.
class InterbotixGripperXS(object):
    def __init__(self, robot_model, gripper_name, robot_name=None, gripper_pressure=0.5, gripper_pressure_lower_limit=150, gripper_pressure_upper_limit=350, init_node=True):
        self.dxl = InterbotixRobotXSCore(robot_model, robot_name, init_node)
        self.gripper = InterbotixGripperXSInterface(self.dxl, gripper_name, gripper_pressure, gripper_pressure_lower_limit, gripper_pressure_upper_limit)

### @brief Definition of the Interbotix Gripper Module
### @param core - reference to the InterbotixRobotXSCore class containing the internal ROS plumbing that drives the Python API
### @param gripper_name - name of the gripper joint as defined in the 'motor_config' yaml file; typically, this is 'gripper'
### @param gripper_pressure - fraction from 0 - 1 where '0' means the gripper operates at 'gripper_pressure_lower_limit' and '1' means the gripper operates at 'gripper_pressure_upper_limit'
### @param gripper_pressure_lower_limit - lowest 'effort' that should be applied to the gripper if gripper_pressure is set to 0; it should be high enough to open/close the gripper (~150 PWM or ~400 mA current)
### @param gripper_pressure_upper_limit - largest 'effort' that should be applied to the gripper if gripper_pressure is set to 1; it should be low enough that the motor doesn't 'overload' when gripping an object for a few seconds (~350 PWM or ~900 mA)
class InterbotixGripperXSInterface(object):

    def __init__(self, core, gripper_name, gripper_pressure=0.5, gripper_pressure_lower_limit=150, gripper_pressure_upper_limit=350):
        self.core = core
        gripper_info = self.core.srv_get_info("single", gripper_name)
        if (gripper_info.mode != "current" and gripper_info.mode != "pwm"):
            rospy.logerr("Please set the gripper's 'operating mode' to 'pwm' or 'current'.")
        self.gripper_moving = False
        self.gripper_command = JointSingleCommand(name="gripper")
        self.gripper_pressure_lower_limit = gripper_pressure_lower_limit
        self.gripper_pressure_upper_limit = gripper_pressure_upper_limit
        self.gripper_value = gripper_pressure_lower_limit + (gripper_pressure * (gripper_pressure_upper_limit - gripper_pressure_lower_limit))
        self.left_finger_index = self.core.js_index_map[gripper_info.joint_names[0]]
        self.left_finger_lower_limit = gripper_info.joint_lower_limits[0]
        self.left_finger_upper_limit = gripper_info.joint_upper_limits[0]
        tmr_gripper_state = rospy.Timer(rospy.Duration(0.02), self.gripper_state)
        print("Gripper Name: %s\nGripper Pressure: %d%%" % (gripper_name, gripper_pressure * 100))
        print("Initialized InterbotixGripperXSInterface!\n")

    ### @brief ROS Timer Callback function to stop the gripper moving past its limits when in PWM mode
    ### @param event [unused] - Timer event message
    def gripper_state(self, event):
        if (self.gripper_moving):
            with self.core.js_mutex:
                gripper_pos = self.core.joint_states.position[self.left_finger_index]
            if ((self.gripper_command.cmd > 0 and gripper_pos >= self.left_finger_upper_limit) or
                (self.gripper_command.cmd < 0 and gripper_pos <= self.left_finger_lower_limit)):
                self.gripper_command.cmd = 0
                self.core.pub_single.publish(self.gripper_command)
                self.gripper_moving = False

    ### @brief Helper function used to publish effort commands to the gripper (when in 'pwm' or 'current' mode)
    ### @param effort - effort command to send to the gripper motor
    ### @param delay - number of seconds to wait before returning control to the user
    def gripper_controller(self, effort, delay):
        self.gripper_command.cmd = effort
        with self.core.js_mutex:
            gripper_pos = self.core.joint_states.position[self.left_finger_index]
        if ((self.gripper_command.cmd > 0 and gripper_pos < self.left_finger_upper_limit) or
            (self.gripper_command.cmd < 0 and gripper_pos > self.left_finger_lower_limit)):
            self.core.pub_single.publish(self.gripper_command)
            self.gripper_moving = True
            rospy.sleep(delay)

    ### @brief Set the amount of pressure that the gripper should use when grasping an object (when in 'effort' control mode)
    ### @param pressure - a scaling factor from 0 to 1 where the pressure increases as the factor increases
    def set_pressure(self, pressure):
        self.gripper_value = self.gripper_pressure_lower_limit + pressure * \
        (self.gripper_pressure_upper_limit - self.gripper_pressure_lower_limit)

    ### @brief Opens the gripper (when in 'pwm' control mode)
    ### @param delay - number of seconds to delay before returning control to the user
    def open(self, delay=1.0):
        self.gripper_controller(self.gripper_value, delay)

    ### @brief Closes the gripper (when in 'pwm' control mode)
    ### @param delay - number of seconds to delay before returning control to the user
    def close(self, delay=1.0):
        self.gripper_controller(-self.gripper_value, delay)
