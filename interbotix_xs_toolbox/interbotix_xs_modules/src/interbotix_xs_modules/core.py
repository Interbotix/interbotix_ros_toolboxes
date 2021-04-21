import copy
import rospy
import threading
from interbotix_xs_sdk.msg import *
from interbotix_xs_sdk.srv import *
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

### @brief Class that interfaces with the xs_sdk node ROS interfaces
### @param robot_model - Interbotix Arm model (ex. 'wx200' or 'vx300s')
### @param robot_name - defaults to value given to 'robot_model'; this can be customized if controlling two of the same arms from one computer (like 'arm1/wx200' and 'arm2/wx200')
### @param init_node - set to True if the InterbotixRobotXSCore class should initialize the ROS node - this is the most Pythonic approach; to incorporate a robot into an existing ROS node though, set to False
### @param joint_state_topic - the specifc JointState topic output by the xs_sdk node
class InterbotixRobotXSCore(object):
    def __init__(self, robot_model, robot_name=None, init_node=True, joint_state_topic="joint_states"):
        self.joint_states = None
        self.js_mutex = threading.Lock()
        self.robot_name = robot_name
        if (self.robot_name is None):
            self.robot_name = robot_model
        if (init_node):
            rospy.init_node(self.robot_name + "_robot_manipulation")
        rospy.wait_for_service("/" + self.robot_name + "/set_operating_modes")
        rospy.wait_for_service("/" + self.robot_name + "/set_motor_pid_gains")
        rospy.wait_for_service("/" + self.robot_name + "/set_motor_registers")
        rospy.wait_for_service("/" + self.robot_name + "/get_motor_registers")
        rospy.wait_for_service("/" + self.robot_name + "/get_robot_info")
        rospy.wait_for_service("/" + self.robot_name + "/torque_enable")
        rospy.wait_for_service("/" + self.robot_name + "/reboot_motors")
        self.srv_set_op_modes = rospy.ServiceProxy("/" + self.robot_name + "/set_operating_modes", OperatingModes)
        self.srv_set_pids = rospy.ServiceProxy("/" + self.robot_name + "/set_motor_pid_gains", MotorGains)
        self.srv_set_reg = rospy.ServiceProxy("/" + self.robot_name + "/set_motor_registers", RegisterValues)
        self.srv_get_reg = rospy.ServiceProxy("/" + self.robot_name + "/get_motor_registers", RegisterValues)
        self.srv_get_info = rospy.ServiceProxy("/" + self.robot_name + "/get_robot_info", RobotInfo)
        self.srv_torque = rospy.ServiceProxy("/" + self.robot_name + "/torque_enable", TorqueEnable)
        self.srv_reboot = rospy.ServiceProxy("/" + self.robot_name + "/reboot_motors", Reboot)
        self.pub_group = rospy.Publisher("/" + self.robot_name + "/commands/joint_group", JointGroupCommand, queue_size=1)
        self.pub_single = rospy.Publisher("/" + self.robot_name + "/commands/joint_single", JointSingleCommand, queue_size=1)
        self.pub_traj = rospy.Publisher("/" + self.robot_name + "/commands/joint_trajectory", JointTrajectoryCommand, queue_size=1)
        self.sub_joint_states = rospy.Subscriber("/" + self.robot_name + "/" + joint_state_topic, JointState, self.joint_state_cb)
        while (self.joint_states == None and not rospy.is_shutdown()): pass
        self.js_index_map = dict(zip(self.joint_states.name, range(len(self.joint_states.name))))
        rospy.sleep(0.5)
        print("Robot Name: %s\nRobot Model: %s" % (self.robot_name, robot_model))
        print("Initialized InterbotixRobotXSCore!\n")

    ### @brief Set the operating mode for either a single motor or a group of motors
    ### @param cmd_type - can be "group" for a group of motors or "single" for a single motor
    ### @param name - group name if cmd_type is 'group' or the motor name if cmd_type is 'single'
    ### @param mode - desired operatinge mode like "position" or "velocity". See the OperatingModes Service description for all choices
    ### @param profile_type - can be "time" or "velocity". See the OperatingModes Service description for details
    ### @param profile_velocity - passthrough to the Profile_Velocity register. See the OperatingModes Service description for details
    ### @param profile_acceleration - passthrough to the Profile_Acceleration register. See the OperatingModes Service description for details
    def robot_set_operating_modes(self, cmd_type, name, mode, profile_type="velocity", profile_velocity=0, profile_acceleration=0):
        self.srv_set_op_modes(cmd_type, name, mode, profile_type, profile_velocity, profile_acceleration)

    ### @brief Set the internal PID gains for either a single motor or a group of motors
    ### @param cmd_type - can be "group" for a group of motors or "single" for a single motor
    ### @param name - group name if cmd_type is 'group' or the motor name if cmd_type is 'single'
    ### @param kp_pos - passthrough to the Position_P_Gain register. See the MotorGains Service description for details
    ### @param ki_pos - passthrough to the Position_I_Gain register. See the MotorGains Service description for details
    ### @param kd_pos - passthrough to the Position_D_Gain register. See the MotorGains Service description for details
    ### @param k1 - passthrough to the Feedforward_1st_Gain register. See the MotorGains Service description for details
    ### @param k2 - passthrough to the Feedforward_2nd_Gain register. See the MotorGains Service description for details
    ### @param kp_vel - passthrough to the Velocity_P_Gain register. See the MotorGains Service description for details
    ### @param ki_vel - passthrough to the Velocity_I_Gain register. See the MotorGains Service description for details
    def robot_set_motor_pid_gains(self, cmd_type, name, kp_pos, ki_pos=0, kd_pos=0, k1=0, k2=0, kp_vel=100, ki_vel=1920):
        self.srv_set_pids(cmd_type, name, kp_pos, ki_pos, kd_pos, k1, k2, kp_vel, ki_vel)

    ### @brief Set the desired register for either a single motor or a group of motors
    ### @param cmd_type - can be "group" for a group of motors or "single" for a single motor
    ### @param name - group name if cmd_type is 'group' or the motor name if cmd_type is 'single'
    ### @param reg - desired register name
    ### @param value - desired value for the above register
    def robot_set_motor_registers(self, cmd_type, name, reg, value):
        self.srv_set_reg(cmd_type, name, reg, value)

    ### @brief Get the desired register value from either a single motor or a group of motors
    ### @param cmd_type - can be "group" for a group of motors or "single" for a single motor
    ### @param name - group name if cmd_type is 'group' or the motor name if cmd_type is 'single'
    ### @param reg - desired register name
    ### @return response - list of register values
    def robot_get_motor_registers(self, cmd_type, name, reg):
        response = self.srv_get_reg(cmd_type=cmd_type, name=name, reg=reg)
        return response

    ### @brief Get information about the robot - mostly joint limit data
    ### @param cmd_type - can be "group" for a group of motors or "single" for a single motor
    ### @param name - group name if cmd_type is 'group' or the motor name if cmd_type is 'single'
    ### @return response - an object with the same structure as a RobotInfo Service description
    def robot_get_robot_info(self, cmd_type, name):
        response = self.srv_get_info(cmd_type, name)
        return response

    ### @brief Torque a single motor or a group of motors to be on or off
    ### @param cmd_type - can be "group" for a group of motors or "single" for a single motor
    ### @param name - group name if cmd_type is 'group' or the motor name if cmd_type is 'single'
    ### @param enable - True to torque on or False to torque off
    def robot_torque_enable(self, cmd_type, name, enable):
        self.srv_torque(cmd_type, name, enable)

    ### @brief Reboot a single motor or a group of motors if they are in an error state
    ### @param cmd_type - can be "group" for a group of motors or "single" for a single motor
    ### @param name - group name if cmd_type is 'group' or the motor name if cmd_type is 'single'
    ### @param enable - True to torque on or False to leave torqued off after rebooting
    ### @param smart_reboot - if 'cmd_type' is set to 'group', setting this to True will only reboot
    ###                       those motors that are in an error state (as opposed to all motors
    ###                       within the group regardless of if they are in an error state)
    def robot_reboot_motors(self, cmd_type, name, enable, smart_reboot=False):
        self.srv_reboot(cmd_type, name, enable, smart_reboot)

    ### @brief Command a group of motors (refer to the JointGroupCommand Message description for more info)
    ### @param group_name - the group name of the motors to command
    ### @param commands - desired list of commands
    def robot_write_commands(self, group_name, commands):
        msg = JointGroupCommand(group_name, commands)
        self.pub_group.publish(msg)

    ### @brief Command a single motor (refer to the JointSingleCommand Message description for more info)
    ### @param joint_name - the name of the motor to command
    ### @param command - desired command
    def robot_write_joint_command(self, joint_name, command):
        msg = JointSingleCommand(joint_name, command);
        self.pub_single.publish(msg)

    ### @brief Command a trajectory of positions or velocities to a single motor or a group of motors
    ### @param cmd_type - can be "group" for a group of motors or "single" for a single motor
    ### @param name - group name if cmd_type is 'group' or the motor name if cmd_type is 'single'
    ### @param type - "position" if the trajectory is a list of positions [rad]; otherwise "velocity" if the
    ###               trajectory is a list of velocities [rad/s]
    ### @param raw_traj - list of dictionaries where each dictionary is made up of a float / list of float pairs.
    ###                   the 'key' is the desired time [sec] from start that the 'value' (list of floats) should be executed.
    ### @details - an example input trajectory for a pan/tilt mechansim could look like...
    ###            [{0, [1,1]},
    ###             {1.5, [-1,0.75]},
    ###             {2.3, [0,0]}]
    def robot_write_trajectory(self, cmd_type, name, type, raw_traj):
        traj = JointTrajectory()
        for point in raw_traj:
            for key, value in point.items():
                traj_point = JointTrajectoryPoint()
                if (type == "position"):
                    traj_point.positions = value
                elif (type == "velocity"):
                    traj_point.velocities = value
                traj_point.time_from_start = rospy.Duration.from_sec(key)
                traj.points.append(traj_point)
        msg = JointTrajectoryCommand(cmd_type, name, traj)
        self.pub_traj.publish(msg)

    ### @brief Get the current joint states (position, velocity, effort) of all Dynamixel motors
    ### @return joint_states - JointState ROS message. Refer to online documenation to see its structure
    def robot_get_joint_states(self):
        joint_states = None
        with self.js_mutex:
            joint_states = copy.deepcopy(self.joint_states)
        return joint_states

    ### @brief Get a single joint state for the specified Dynamixel motor
    ### @param name - desired motor name for which to get the joint state
    ### @return joint_info - dictionary with 3 keys: "position", "velocity", and "effort".
    ###                      Units are rad, rad/s, and mA
    def robot_get_single_joint_state(self, name):
        joint_states = None
        with self.js_mutex:
            joint_states = copy.deepcopy(self.joint_states)
        joint_index = joint_states.name.index(name)
        joint_info = {}
        joint_info["position"] = joint_states.position[joint_index]
        joint_info["velocity"] = joint_states.velocity[joint_index]
        joint_info["effort"] = joint_states.effort[joint_index]
        return joint_info

    ### @brief ROS Subscriber Callback function to get the latest JointState message
    def joint_state_cb(self, msg):
        with self.js_mutex:
            self.joint_states = msg
