import copy
import rospy
import threading
from xarm_msgs.msg import *
from xarm_msgs.srv import *
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


### Note that this module uses the Xarm built-in approach to perform Inverse Kinematics instead of the Modern Robotics one
### To use the Modern Robotics Inverse Kinematics solver, use the InterbotixManipulatorUX class instead

### @brief Standalone Module to control a Universal Factory Xarm
### @param robot_model - Universal Factory Xarm model (ex. 'uxarm5' or 'uxarm6')
### @param robot_name - defaults to value given to 'robot_model'; this can be customized if controlling two of the same arms from one computer (like 'arm1/uxarm6' and 'arm2/uxarm6')
### @param mode - Universal Factory Xarm Mode; can be 0 (pose mode), 1 (servo mode), or 2 (teach mode)
### @param wait_for_finish - set to True to have the function wait until arm is finished moving before returning (only applicable in Mode 0); otherwise, set to False
### @param ee_offset - transform from the current end-effector frame to the desired end-effector frame [x, y, z, roll, pitch, yaw] (units are meters/radians)
### @param init_node - set to True if the InterbotixRobotUXCore class should initialize the ROS node - this is the most Pythonic approach; to incorporate a robot into an existing ROS node though, set to False
### @param joint_state_topic - desired joint_state topic name to subscribe to; note that the name is resolved relative to the 'robot_name' namespace
class InterbotixRobotUXCore(object):
    def __init__(self, robot_model, robot_name=None, mode=0, wait_for_finish=True, ee_offset=None, init_node=True, joint_state_topic="joint_states"):
        self.joint_states = None
        self.xarm_states = None
        self.ee_offset = ee_offset
        self.xs_mutex = threading.Lock()
        self.js_mutex = threading.Lock()
        self.robot_model = robot_model
        self.robot_name = robot_name
        if (self.robot_name is None):
            self.robot_name = robot_model
        if (init_node):
            rospy.init_node(self.robot_name + "_robot_manipulation")
        self.dof = rospy.get_param("/" + self.robot_name + "/DOF")
        self.joint_names = rospy.get_param("/" + self.robot_name + "/joint_names")
        rospy.set_param("/" + self.robot_name + "/wait_for_finish", wait_for_finish)
        rospy.wait_for_service("/" + self.robot_name + "/motion_ctrl")
        rospy.wait_for_service("/" + self.robot_name + "/get_err")
        rospy.wait_for_service("/" + self.robot_name + "/clear_err")
        rospy.wait_for_service("/" + self.robot_name + "/set_mode")
        rospy.wait_for_service("/" + self.robot_name + "/set_state")
        rospy.wait_for_service("/" + self.robot_name + "/set_load")
        rospy.wait_for_service("/" + self.robot_name + "/set_tcp_offset")
        rospy.wait_for_service("/" + self.robot_name + "/go_home")
        rospy.wait_for_service("/" + self.robot_name + "/move_line")
        rospy.wait_for_service("/" + self.robot_name + "/move_lineb")
        rospy.wait_for_service("/" + self.robot_name + "/move_joint")
        rospy.wait_for_service("/" + self.robot_name + "/move_servoj")
        rospy.wait_for_service("/" + self.robot_name + "/move_servo_cart")
        self.srv_motion_ctrl = rospy.ServiceProxy("/" + self.robot_name + "/motion_ctrl", SetAxis)
        self.srv_get_err = rospy.ServiceProxy("/" + self.robot_name + "/get_err", GetErr)
        self.srv_clear_err = rospy.ServiceProxy("/" + self.robot_name + "/clear_err", ClearErr)
        self.srv_set_mode = rospy.ServiceProxy("/" + self.robot_name + "/set_mode", SetInt16)
        self.srv_set_state = rospy.ServiceProxy("/" + self.robot_name + "/set_state", SetInt16)
        self.srv_set_load = rospy.ServiceProxy("/" + self.robot_name + "/set_load", SetLoad)
        self.srv_set_tcp = rospy.ServiceProxy("/" + self.robot_name + "/set_tcp_offset", TCPOffset)
        self.srv_go_home = rospy.ServiceProxy("/" + self.robot_name + "/go_home", Move)
        self.srv_move_line = rospy.ServiceProxy("/" + self.robot_name + "/move_line", Move)
        self.srv_move_lineb = rospy.ServiceProxy("/" + self.robot_name + "/move_lineb", Move)
        self.srv_move_joint = rospy.ServiceProxy("/" + self.robot_name + "/move_joint", Move)
        self.srv_move_servoj = rospy.ServiceProxy("/" + self.robot_name + "/move_servoj", Move)
        self.srv_move_servo_cart = rospy.ServiceProxy("/" + self.robot_name + "/move_servo_cart", Move)
        self.sub_joint_states = rospy.Subscriber("/" + self.robot_name + "/" + joint_state_topic, JointState, self.joint_state_cb)
        self.sub_xarm_states = rospy.Subscriber("/" + self.robot_name + "/xarm_states", RobotMsg, self.xarm_state_cb)
        rospy.loginfo("Initializing InterbotixRobotUXCore...")
        rospy.loginfo("\nRobot Name: %s\nRobot Model: %s\n" % (self.robot_name, robot_model))
        while (self.joint_states == None and self.xarm_states == None and not rospy.is_shutdown()): pass
        self.js_index_map = dict(zip(self.joint_states.name, range(len(self.joint_states.name))))
        self.mode = mode
        rospy.sleep(1)
        self.robot_motion_enable(8, True)
        if self.ee_offset is not None:
            ee_off = self.ee_offset[:]
            ee_off[0] *= 1000
            ee_off[1] *= 1000
            ee_off[2] *= 1000
            self.robot_set_tcp_offset(ee_off)
        self.robot_smart_mode_reset(self.mode)

    ### @brief Enable/Disable the specified joint
    ### @param id - joint to enable/disable (1-8)
    ### @param enable - whether the joint should be enabled or disabled
    ### @return ret - error code (0 means all good)
    ### @details - the number at the end of each joint name corresponds to its ID; '8' is a special
    ###            case that enables all motors on the robot
    def robot_motion_enable(self, id=8, enable=True):
        resp = self.srv_motion_ctrl(id, enable)
        rospy.loginfo(resp.message)
        return resp.ret

    ### @brief Get Error
    ### @return ret - error code (0 means all good)
    def robot_get_error(self):
        resp = self.srv_get_err()
        rospy.loginfo(resp.message)
        return resp.err

    ### @brief Clear Error
    ### @return ret - error code (0 means all good)
    def robot_clear_error(self):
        resp = self.srv_clear_err()
        rospy.loginfo(resp.message)
        return resp.ret

    ### @brief Set the Operating Mode
    ### @param mode - mode to set (0 = pose, 1 = servo, 2 = teach)
    ### @return ret - error code (0 means all good)
    def robot_set_mode(self, mode=0):
        resp = self.srv_set_mode(mode)
        if (resp.ret != 0):
            self.mode = 0
        else:
            self.mode = mode
        rospy.loginfo(resp.message)
        return resp.ret

    ### @brief Set the robot state
    ### @param state - state to which to set the robot (should normally be 0)
    ### @return ret - error code (0 means all good)
    def robot_set_state(self, state=0):
        resp = self.srv_set_state(state)
        rospy.loginfo(resp.message)
        return resp.ret

    ### @brief Set the centroid of the load seen at the end-effector w.r.t. the end-effector frame
    ### @param mass - mass [kg] of the load
    ### @param xc - 'X' component of the centroid [mm]
    ### @param yc - 'Y' component of the centroid [mm]
    ### @param zc - 'Z' component of the centroid [mm]
    ### @return ret - error code (0 means all good)
    def robot_set_load(self, mass, xc=0, yc=0, zc=0):
        resp = self.srv_set_load(mass, xc, yc, zc)
        rospy.loginfo(resp.message)
        return resp.ret

    ### @brief Set TCP offset w.r.t. the end-effector frame
    ### @param ee_offset - [x, y, z, roll, pitch, yaw] (mm / rad)
    ### @return ret - error code (0 means all good)
    def robot_set_tcp_offset(self, ee_offset):
        tcp_offset = TCPOffsetRequest()
        tcp_offset.x = ee_offset[0]
        tcp_offset.y = ee_offset[1]
        tcp_offset.z = ee_offset[2]
        tcp_offset.roll = ee_offset[3]
        tcp_offset.pitch = ee_offset[4]
        tcp_offset.yaw = ee_offset[5]
        resp = self.srv_set_tcp(tcp_offset)
        self.robot_set_state(0)
        rospy.loginfo(resp.message)
        return resp.ret

    ### @brief Go Home (used in Mode 0)
    ### @param vel - desired joint velocity [rad/s]
    ### @param accel - desired joint acceleration [rad/s^2]
    ### @return ret - error code (0 means all good)
    def robot_go_home(self, vel=0.35, accel=7):
        resp = self.srv_go_home(mvvelo=vel, mvacc=accel)
        rospy.loginfo(resp.message)
        return resp.ret

    ### @brief Move end-effector on a line path (used in Mode 0)
    ### @param pose - [x, y, z, roll, pitch, yaw] (mm / rad)
    ### @param vel - desired end-effector speed [mm/s]
    ### @param accel - desired end-effector acceleration [mm/s^2]
    ### @return ret - error code (0 means all good)
    def robot_move_line(self, pose, vel=200, accel=2000):
        resp = self.srv_move_line(mvvelo=vel, mvacc=accel, mvtime=0, pose=pose)
        if resp.ret != 0: rospy.loginfo(resp.message)
        return resp.ret

    ### @brief Move end-effector on a sequence of line paths that blend together (used in Mode 0)
    ### @param num_points - number of points for the end-effector to reach
    ### @param pose_list - a list of desired [x, y, z, roll , pitch, yaw] (mm / rad) commands
    ### @param vel - desired end-effector speed [mm/s]
    ### @param accel - desired end-effector acceleration [mm/s^2]
    ### @param radii - radius of curve when connecting two lines in a sequence together [mm]
    ### @return ret - error code (0 means all good)
    def robot_move_lineb(self, num_points, pose_list, vel=200, accel=2000, radii=0):
        move_request = MoveRequest(mvvelo=vel, mvacc=accel, mvtime=0, mvradii=radii)
        for point in range(num_points):
            move_request.pose = pose_list[point]
            resp = self.srv_move_lineb(move_request)
            if (resp.ret != 0):
                rospy.loginfo(resp.message)
                return resp.ret

    ### @brief Move Joint (used in Mode 0)
    ### @param cmd - list of desired joint positions [rad]
    ### @param vel - desired joint velocity [rad/s]
    ### @param accel - desired joint acceleration [rad/s^2]
    ### @return ret - error code (0 means all good)
    def robot_move_joint(self, cmd, vel=1.0, accel=5.0):
        resp = self.srv_move_joint(mvvelo=vel, mvacc=accel, mvtime=0, pose=cmd)
        if resp.ret != 0: rospy.loginfo(resp.message)
        return resp.ret

    ### @brief Move ServoJ - moves joints extremely fast (used in Mode 1)
    ### @param cmd - list of desired joint positions [rad]
    ### @return ret - error code (0 means all good)
    ### @details - make sure to send joint positions very near to the
    ###            current ones or the behavior might be unpredictable
    def robot_move_servoj(self, cmd):
        resp = self.srv_move_servoj(mvvelo=0, mvacc=0, mvtime=0, pose=cmd)
        if resp.ret != 0: rospy.loginfo(resp.message)
        return resp.ret

    ### @brief Move Servo Cart - moves joints extremely fast (used in Mode 1)
    ### @param cmd - desired pose [x, y, z, roll, pitch, yaw] (mm / rad)
    ### @return ret - error code (0 means all good)
    ### @details - make sure to send a pose very near to the current pose
    ###            or the behavior might be unpredictable
    def robot_move_servo_cart(self, cmd):
        resp = self.srv_move_servo_cart(mvvelo=0, mvacc=0, mvtime=0, pose=cmd)
        if resp.ret != 0: rospy.loginfo(resp.message)
        return resp.ret

    ### @brief Smart mode reset - checks for errors while setting the mode and loops until it's successful
    ### @param mode - desired mode to set
    def robot_smart_mode_reset(self, mode=0):
        with self.xs_mutex:
            current_mode = self.xarm_states.mode
        while (mode != current_mode):
            ret = self.robot_set_mode(mode)
            while (ret != 0):
                self.robot_clear_error()
                ret = self.robot_set_mode(mode)
            self.robot_set_state(0)
            rospy.sleep(0.5)
            with self.xs_mutex:
                current_mode = self.xarm_states.mode

    ### @brief Get info on the specified joint
    ### @param name - joint name for which to get info
    ### @return joint_info - the joint state (position [rad], velocity, [rad/s], and effort)
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

    ### @brief Get info on all joints
    ### @return joint_states - info on all the joints
    def robot_get_joint_states(self):
        joint_states = None
        with self.js_mutex:
            joint_states = copy.deepcopy(self.joint_states)
        return joint_states

    ### @brief ROS Subscriber Callback function get the current joint states
    ### @param msg - ROS JointState message
    def joint_state_cb(self, msg):
        with self.js_mutex:
            self.joint_states = msg

    ### @brief ROS Subscriber Callback function to get the current xarm states
    ### @param msg - ROS RobotMsg message
    def xarm_state_cb(self, msg):
        with self.xs_mutex:
            self.xarm_states = msg
