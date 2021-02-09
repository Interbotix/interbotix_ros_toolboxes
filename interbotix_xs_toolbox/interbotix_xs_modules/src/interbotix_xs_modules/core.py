import copy
import rospy
import threading
from interbotix_xs_sdk.msg import *
from interbotix_xs_sdk.srv import *
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

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

    def robot_set_operating_modes(self, cmd_type, name, mode, profile_type="velocity", profile_velocity=0, profile_acceleration=0):
        self.srv_set_op_modes(cmd_type, name, mode, profile_type, profile_velocity, profile_acceleration)

    def robot_set_motor_pid_gains(self, cmd_type, name, kp_pos, ki_pos=0, kd_pos=0, k1=0, k2=0, kp_vel=100, ki_vel=1920):
        self.srv_set_pids(cmd_type, name, kp_pos, ki_pos, kd_pos, k1, k2, kp_vel, ki_vel)

    def robot_set_motor_registers(self, cmd_type, name, reg, value):
        self.srv_set_reg(cmd_type, name, reg, value)

    def robot_get_motor_registers(self, cmd_type, name, reg):
        response = self.srv_get_reg(cmd_type=cmd_type, name=name, reg=reg)
        return response

    def robot_get_robot_info(self, cmd_type, name):
        response = self.srv_get_info(cmd_type, name)
        return response

    def robot_torque_enable(self, cmd_type, name, enable):
        self.srv_torque(cmd_type, name, enable)

    def robot_reboot_motors(self, cmd_type, name, enable, smart_reboot=False):
        self.srv_reboot(cmd_type, name, enable, smart_reboot)

    def robot_write_commands(self, group_name, commands):
        msg = JointGroupCommand(group_name, commands)
        self.pub_group.publish(msg)

    def robot_write_joint_command(self, joint_name, command):
        msg = JointSingleCommand(joint_name, command);
        self.pub_single.publish(msg)

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

    def robot_get_joint_states(self):
        joint_states = None
        with self.js_mutex:
            joint_states = copy.deepcopy(self.joint_states)
        return joint_states

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

    def joint_state_cb(self, msg):
        with self.js_mutex:
            self.joint_states = msg
