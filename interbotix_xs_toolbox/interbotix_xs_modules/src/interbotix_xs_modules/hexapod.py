import copy
import math
import rospy
import tf2_ros
import numpy as np
from urdf_parser_py.urdf import URDF
from interbotix_xs_sdk.msg import *
from geometry_msgs.msg import PoseStamped, Point
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped, Quaternion
from interbotix_xs_modules.core import InterbotixRobotXSCore
from interbotix_common_modules import angle_manipulation as ang
from interbotix_rpi_modules.neopixels import InterbotixRpiPixelInterface


### Notes

# Transform Names
#     - self.T_sf: odometry transform from the 'odom' frame to the hexapod's 'base_footprint' frame; only the X, Y, and Yaw values are specified; it is updated when the hexapod moves in the world
#     - self.T_fb: transform from the 'base_footprint' frame to the hexapod's 'base_link' frame; all values can be specified; it is updated when the hexapod moves in place
#     - self.T_bc: a dictionary that holds all six static transforms from the hexapod's 'base_link' frame to every leg's initial "coxa_link" frame; it is never updated after initialization
#     - R_cfcm: rotation matrix specifying the transform from a leg's initial (or fixed) 'coxa_link' frame to the present (or moving) 'coxa_link' frame; it is updated whenever the hexapod moves
#     - p_femur: point specifying the transform from a leg's 'femur_link' frame to its 'foot_link' frame; it is updated whenever the hexapod moves
#     - p_cm: point specifying the transform from a leg's present (or moving) 'coxa_link' frame to its 'foot_link' frame; it is updated whenever the hexapod moves
#     - p_cf: point specifying the transform from a leg's initial (or fixed) 'coxa_link' frame to its 'foot_link' frame; it is updated whenever the hexapod moves
#     - p_b: point specifying the transform from the hexapod's 'base_link' frame to a specified leg's 'foot_link' frame; it is updated whenever the hexapod moves
#     - p_f: point specifying the transform from the hexapod's 'base_footprint' frame to a specified leg's 'foot_link' frame; it is updated when the hexapod moves in the world

# Gaits
# The code supports the tripod gait and the modified versions of the ripple and wave gaits that are discussed at https://hexyrobot.wordpress.com/2015/11/20/common-walking-gaits-for-hexapods/.
# The implementation of these gaits were all custom developed using two sinusoid wave functions. The first sin function was used to determine what the x, y, and yaw values for each leg should be while the
# second sin function was used to determine what the z value for each leg should be. A 'period' represents one complete wave cycle of the second sin function signifying the time it takes for a given
# leg to start and finish one 'swing' phase. In the tripod gait, a 'period' also represents the time it takes for a leg to start and finish one 'stance' phase. So, the number of periods needed for each gait cycle are:
#     - Tripod: 2 periods -> one for the 'swing' phase and one for the 'stance' phase
#     - Ripple: 3 periods -> one for the 'swing' phase and two for the 'stance' phase
#     - Wave: 6 periods -> one for the 'swing' phase and five for the 'stance' phase
# This means that the tripod gait is 3 times faster than the wave gait and 1.5 times faster than the ripple gait. However, the inverse holds true for stability.

### @brief Standalone Module to control an Interbotix Hexapod
### @param robot_model - Interbotix Hexapod model (ex. 'mark4')
### @param robot_name - defaults to value given to 'robot_model'; this can be customized to best suit the user's needs
### @param position_p_gain - passthrough to the Position_P_Gain register on all hexapod servos - sets the desired Proportional gain
### @param init_node - set to True if the InterbotixRobotXSCore class should initialize the ROS node - this is the most Pythonic approach; to incorporate a robot into an existing ROS node though, set to False
class InterbotixHexapodXS(object):
    def __init__(self, robot_model, robot_name=None, position_p_gain=800, init_node=True):
        self.dxl = InterbotixRobotXSCore(robot_model, robot_name, init_node)
        self.hex = InterbotixHexapodXSInterface(self.dxl, position_p_gain)
        self.pixels = InterbotixRpiPixelInterface(self.dxl.robot_name)

### @brief Definition of the Interbotix Hexapod Module
### @param core - reference to the InterbotixRobotXSCore class containing the internal ROS plumbing that drives the Python API
class InterbotixHexapodXSInterface(object):
    def __init__(self, core, position_p_gain):
        self.core = core                                                        # Reference to the InterbotixRobotXSCore object
        self.position_p_gain = position_p_gain                                  # Desired Proportional gain for all servos
        self.inc_prev = 0                                                       # Latest increment during the gait cycle
        self.period_cntr = 0                                                    # Used to count a period (self.num_steps/2.0) during the wave or ripple gait cycles
        self.num_steps = 20.0                                                   # Number of steps in one wave of the first sinusoid cycle
        self.step_cntr = 1                                                      # Counts the number of steps during a single gait cycle
        self.gait_types = ["tripod", "ripple", "wave"]                          # Three supported gaits that can be selected
        self.gait_factors = {"tripod" : 2.0, "ripple" : 3.0, "wave" : 6.0}      # Gait factors that modify the first sinusoid function mentioned above based on the selected gait
        self.wave_legs = ["right_front", "left_front", "right_middle", "left_middle", "right_back", "left_back"]                                            # Leg 'Queue' when doing the wave gait; after every period, the first element is taken out and appended to the end of the list
        self.wave_incs = {leg:0 for leg in self.wave_legs}                                                                                                  # Dictionary to keep track of where each leg's foot is during the wave gait
        self.ripple_legs = {"first" : ["left_middle", "right_front"], "second" : ["left_back", "right_middle"], "third" : ["left_front", "right_back"]}     # Dictionary to keep track of which two legs move together during the ripple gait
        self.ripple_leg_pairs = ["first", "second", "third"]                                                                                                # Leg pair 'Queue' when doing the ripple gait; after every period, the first element is taken out and appended to the end of the list
        self.ripple_incs = {pair:0 for pair in self.ripple_leg_pairs}                                                                                       # Dictionary to keep track of where each leg pair's feet are during the ripple gait
        self.leg_list = ["left_back", "left_middle", "left_front", "right_front", "right_middle", "right_back"]                                             # List of all legs in the hexapod
        self.leg_time_map = {leg: {"move" : 0, "accel" : 0} for leg in self.leg_list}                                                                       # Keeps track of the moving & accel times for each joint group
        self.leg_time_map["all"] = {"move" : 0, "accel" : 0}
        self.leg_mode_on = False                                                # Boolean dictating whether or no 'individual leg control' is on or not
        self.foot_points = {}                                                   # Dictionary that contains the current feet positions for each leg
        self.home_foot_points = {}                                              # Dictionary that contains the 'home' feet positions for each leg before starting a gait cycle
        self.sleep_foot_points = {}                                             # Dictionary that contains the 'sleep' feet positions for each leg
        self.home_height = 0                                                    # The 'z' component of self.T_fb specifying the height of the 'base_link' frame relative to the 'base_footprint' frame
        self.sleep_height = 0                                                   # The 'z' component of self.T_fb specifying the height of the 'base_link' frame relative to the 'base_footprint' frame when sleeping
        self.bottom_height = 0                                                  # Height difference between the 'base_link' frame and the 'base_bottom_link' frame
        self.T_sf = np.identity(4)                                              # Odometry transform specifying the 'base_footprint' frame relative to the 'odom' frame
        self.T_fb = np.identity(4)                                              # Body transform specifying the 'base_link' frame relative to the 'base_footprint' frame
        self.T_bc = {}                                                          # Dictionary containing the static transforms of all six 'coxa_link' frames relative to the 'base_link' frame
        self.coxa_length = None                                                 # Length [meters] of the coxa_link
        self.femur_length = None                                                # Length [meters] of the femur_link
        self.tibia_length = None                                                # Length [meters] of the tibia_link
        self.femur_offset_angle = None                                          # Offset angle [rad] that makes the tibia_link frame coincident with a line shooting out of the coxa_link frame that's parallel to the ground
        self.tibia_offset_angle = None                                          # Offset angle [rad] that makes the foot_link frame coincident with a line shooting out of the coxa_link frame that's parallel to the ground
        self.get_urdf_info()
        self.pose = PoseStamped()                                               # ROS PoseStamped message to publish self.T_sf to its own topic
        self.t_sf = TransformStamped()                                          # ROS Transform that holds self.T_sf and is published to the /tf topic
        self.t_fb = TransformStamped()                                          # ROS Transform that holds self.T_fb and is published to the /tf topic
        self.br = tf2_ros.TransformBroadcaster()
        self.initialize_transforms()
        self.info = self.core.srv_get_info("group", "all")
        self.info_index_map = dict(zip(self.info.joint_names, range(len(self.info.joint_names))))           # Map joint names to their positions in the upper/lower and sleep position arrays
        self.hexapod_command = JointGroupCommand(name="all", cmd=[0] * self.info.num_joints)                # ROS Message to command all 18 joints in the hexapod simultaneously
        self.initialize_start_pose()
        self.pub_pose = rospy.Publisher("/" + self.core.robot_name + "/pose", PoseStamped, queue_size=1)    # ROS Publisher to publish self.T_sf as a PoseStamped message
        tmr_transforms = rospy.Timer(rospy.Duration(0.04), self.publish_states)                             # ROS Timer to publish transforms to the /tf and /odom topics at a fixed rate
        print("Initialized InterbotixHexapodXSInterface!\n")

    ### @brief Parses the URDF and populates the appropiate variables with link information
    def get_urdf_info(self):
        full_rd_name = '/' + self.core.robot_name + '/robot_description'
        while rospy.has_param(full_rd_name) != True: pass
        robot_description = URDF.from_parameter_server(key=full_rd_name)

        for leg in self.leg_list:
            joint_object = next((joint for joint in robot_description.joints if joint.name == (leg + "_coxa")), None)
            T_bc = np.identity(4)
            T_bc[:3,3] = joint_object.origin.xyz
            T_bc[:3,:3] = ang.eulerAnglesToRotationMatrix(joint_object.origin.rpy)
            self.T_bc[leg] = T_bc

        femur_joint = next((joint for joint in robot_description.joints if joint.name == "left_front_femur"))
        self.coxa_length = femur_joint.origin.xyz[0]

        tibia_joint = next((joint for joint in robot_description.joints if joint.name == "left_front_tibia"))
        femur_x = tibia_joint.origin.xyz[0]
        femur_z = tibia_joint.origin.xyz[2]
        self.femur_offset_angle = abs(math.atan2(femur_z, femur_x))
        self.femur_length = math.sqrt(femur_x**2 + femur_z**2)

        foot_joint = next((joint for joint in robot_description.joints if joint.name == "left_front_foot"))
        tibia_x = foot_joint.origin.xyz[0]
        tibia_z = foot_joint.origin.xyz[2]
        self.tibia_offset_angle = abs(math.atan2(tibia_z, tibia_x)) - self.femur_offset_angle
        self.tibia_length = math.sqrt(tibia_x**2 + tibia_z**2)

        bottom_joint = next((joint for joint in robot_description.joints if joint.name == "base_bottom"))
        self.bottom_height = abs(bottom_joint.origin.xyz[2])
        self.home_height = self.bottom_height + 0.05

    ### @brief Intializes the static components of the ROS transforms
    def initialize_transforms(self):
        self.pose.header.frame_id = self.core.robot_name + "/odom"
        self.pose.pose.orientation.w = 1.0
        self.t_sf.header.frame_id = self.core.robot_name + "/odom"
        self.t_sf.child_frame_id = self.core.robot_name + "/base_footprint"
        self.t_sf.transform.rotation.w = 1.0
        self.t_fb.header.frame_id = self.core.robot_name + "/base_footprint"
        self.t_fb.child_frame_id = self.core.robot_name + "/base_link"
        self.t_fb.transform.rotation.w = 1.0

    ### @brief Uses forward-kinematics to find the initial foot position for each leg relative to the 'base_footprint' frame
    def initialize_start_pose(self):
        self.T_fb[2,3] = self.bottom_height
        for leg in self.leg_list:
            theta_1 = self.info.joint_sleep_positions[self.info_index_map[leg + "_coxa"]]
            theta_2 = self.info.joint_sleep_positions[self.info_index_map[leg + "_femur"]]
            theta_3 = self.info.joint_sleep_positions[self.info_index_map[leg + "_tibia"]]
            self.sleep_foot_points[leg] = self.solve_fk([theta_1, theta_2, theta_3], leg)
            self.sleep_height = self.bottom_height - self.sleep_foot_points[leg][2]
            self.sleep_foot_points[leg][2] = 0
        self.home_foot_points = copy.deepcopy(self.sleep_foot_points)
        self.foot_points = copy.deepcopy(self.home_foot_points)
        self.core.srv_set_reg("group", "all", "Position_P_Gain", self.position_p_gain)
        self.reset_hexapod("home")
        self.move_in_world()

    ### @brief Performs forward-kinematics to get the specified leg's foot position relative to the 'base_footprint' frame
    ### @param theta - list specifying the desired coxa, femur, and tibia joint values
    ### @param leg - name of the leg to perform forward-kinematics on
    ### @return p_f - 3-element list specifying the foot point relative to the 'base_footprint' frame
    def solve_fk(self, theta, leg):
        x = self.femur_length * math.cos(theta[1] + self.femur_offset_angle) + self.tibia_length * math.cos(theta[1] + self.femur_offset_angle + theta[2] + self.tibia_offset_angle)
        z = -self.femur_length * math.sin(theta[1] + self.femur_offset_angle) - self.tibia_length * math.sin(theta[1] + self.femur_offset_angle + theta[2] + self.tibia_offset_angle)

        R_cfcm = np.identity(3)
        R_cfcm[:2,:2] = ang.yawToRotationMatrix(theta[0])

        p_femur = [x, 0, z]
        p_cm = np.add(p_femur, [self.coxa_length, 0, 0])
        p_cf = np.dot(R_cfcm, p_cm)
        p_b = np.dot(self.T_bc[leg], np.r_[p_cf, 1])
        p_f = np.dot(self.T_fb, p_b)

        return [p_f[0], p_f[1], p_f[2]]

    ### @brief Performs inverse-kinematics to get the desired joint angles to move a leg's foot to the right position
    ### @param p_f - 3-element list specifying the desired leg's foot position relative to the 'base_footprint' frame
    ### @param leg - name of the leg to perform inverse-kinematics on
    ### @param mod_value - relative distance value by which to tighten or widen the hexapod stance [m]
    ### @return <list, bool> - 3-element list and boolean specifying the required joint angles and if the function was successful respectively
    def solve_ik(self, p_f, leg, mod_value=0):
        p_b = np.dot(ang.transInv(self.T_fb), np.r_[p_f, 1])
        p_cf = np.dot(ang.transInv(self.T_bc[leg]), p_b)
        theta_1 = math.atan2(p_cf[1], p_cf[0])

        R_cfcm = np.identity(3)
        R_cfcm[:2,:2] = ang.yawToRotationMatrix(theta_1)

        p_cm = np.dot(R_cfcm.T, p_cf[:3])
        p_cm[0] += mod_value
        p_femur = np.subtract(p_cm, [self.coxa_length, 0, 0])
        try:
            theta_3 = math.acos((p_femur[0]**2 + p_femur[2]**2 - self.femur_length**2 - self.tibia_length**2) / (2 * self.femur_length * self.tibia_length))
            theta_2 = -(math.atan2(p_femur[2], p_femur[0]) + math.atan2((self.tibia_length * math.sin(theta_3)) , (self.femur_length + self.tibia_length * math.cos(theta_3))))
            return [theta_1, theta_2 - self.femur_offset_angle, theta_3 - self.tibia_offset_angle], True
        except ValueError:
            return [0, 0, 0], False

    ### @brief Adjusts the hexapod's stance to be wider or narrower
    ### @param mod_value - relative distance value by which to tighten or widen the hexapod stance [m]
    def modify_stance(self, mod_value):
        new_foot_points = {}
        for leg, point in self.foot_points.items():
            theta_list, success = self.solve_ik(point, leg, mod_value)
            if success:
                new_foot_points[leg] = self.solve_fk(theta_list, leg)
            else:
                return False
        self.foot_points = new_foot_points
        self.move_in_world()
        return True

    ### @brief Resets the hexapod to its 'home' or 'sleep' pose
    ### @param pose_type - desired pose
    def reset_hexapod(self, pose_type="home"):
        rospy.loginfo("Going to %s pose..." % pose_type)
        self.T_fb = np.identity(4)
        self.T_fb[2,3] = self.home_height
        self.move_in_place()
        if (self.foot_points != self.home_foot_points):
            self.foot_points = copy.deepcopy(self.home_foot_points)
            self.move_in_world()
        if pose_type == "sleep":
            if (self.foot_points != self.sleep_foot_points):
                self.foot_points = copy.deepcopy(self.sleep_foot_points)
                self.move_in_world()
            self.T_fb[2,3] = self.sleep_height
            self.move_in_place()
        self.set_trajectory_time("all", 0.150, 0.075)

    ### @brief Update the ROS transform signifying self.T_sf
    ### @param moving_time - time [sec] it takes for each motor to move a step
    ### @details - Message is future dated by 'moving_time' milliseconds since that's
    ###            the amount of time it takes for the motors to move
    def update_tsf_transform(self, moving_time):
        self.t_sf.transform.translation.x = self.T_sf[0,3]
        self.t_sf.transform.translation.y = self.T_sf[1,3]
        rpy = ang.rotationMatrixToEulerAngles(self.T_sf[:3,:3])
        q = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
        self.t_sf.transform.rotation = Quaternion(q[0], q[1], q[2], q[3])
        self.t_sf.header.stamp = rospy.Time.now() + rospy.Duration(moving_time)
        self.pose.pose.position = Point(self.T_sf[0,3], self.T_sf[1,3], 0)
        self.pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        self.pose.header.stamp = rospy.Time.now() + rospy.Duration(moving_time)

    ### @brief Update the ROS transform signifying self.T_fb
    ### @details - Message is future dated by 'moving_time' since that's the
    ###            amount of time it takes for the motors to move
    def update_tfb_transform(self, moving_time):
        self.t_fb.transform.translation.x = self.T_fb[0,3]
        self.t_fb.transform.translation.y = self.T_fb[1,3]
        self.t_fb.transform.translation.z = self.T_fb[2,3]
        rpy = ang.rotationMatrixToEulerAngles(self.T_fb[:3,:3])
        q = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
        self.t_fb.transform.rotation = Quaternion(q[0], q[1], q[2], q[3])
        self.t_fb.header.stamp = rospy.Time.now() + rospy.Duration(moving_time)

    ### @brief Updates the ROS message containing the joint commands with new values
    ### @param point - 3-element list specifying the desired foot position (relative to the 'base_footprint' frame) for a given leg
    ### @param leg - name of the leg to be commanded
    ### @return <bool> - True if function completed successfully; False otherwise
    def update_joint_command(self, point, leg):
        theta, success = self.solve_ik(point, leg)
        if not success: return False
        theta_names = [leg + "_coxa", leg + "_femur", leg + "_tibia"]
        for x in range(len(theta_names)):
            if not (self.info.joint_lower_limits[self.info_index_map[theta_names[x]]] <= theta[x] <= self.info.joint_upper_limits[self.info_index_map[theta_names[x]]]):
                return False
        self.hexapod_command.cmd[self.info_index_map[leg + "_coxa"]] = theta[0]
        self.hexapod_command.cmd[self.info_index_map[leg + "_femur"]] = theta[1]
        self.hexapod_command.cmd[self.info_index_map[leg + "_tibia"]] = theta[2]
        return True

    ### @brief ROS Timer callback function that continously publishes transforms
    ### @param event - unused ROS Timer event message
    ### @details - if a transform is not being updated, then wait until the time stamp is no longer in the future
    ###            before publishing it with the current ROS time (prevents jumps back in time)
    def publish_states(self, event):
        time = rospy.Time.now()
        if self.t_sf.header.stamp < time:
            self.t_sf.header.stamp = time
            self.pose.header.stamp = time
        if self.t_fb.header.stamp < time:
            self.t_fb.header.stamp = time
        self.br.sendTransform(self.t_sf)
        self.br.sendTransform(self.t_fb)
        self.pub_pose.publish(self.pose)

    ### @brief Helper function to command the 'Profile_Velocity' and 'Profile_Acceleration' motor registers
    ### @param group - name of the leg to control (or 'all' for all legs)
    ### @param moving_time - time in seconds that each motor should move
    ### @param accel_time - time in seconds that each motor should accelerate
    def set_trajectory_time(self, group, moving_time=1.0, accel_time=0.3):
        if (group == "all" and self.leg_mode_on):
            self.core.srv_set_reg("group", "all", "Profile_Velocity", int(moving_time * 1000))
            self.core.srv_set_reg("group", "all", "Profile_Acceleration", int(accel_time * 1000))
            for leg in self.leg_list:
                self.leg_time_map[leg] = {"move" : moving_time, "accel" : accel_time}
            self.leg_time_map["all"] = {"move" : moving_time, "accel" : accel_time}
            self.leg_mode_on = False
        else:
            times = self.leg_time_map[group]
            if (moving_time != times["move"]):
                times["move"] = moving_time
                self.core.srv_set_reg("group", group, "Profile_Velocity", int(moving_time * 1000))
            if (accel_time != times["accel"]):
                times["accel"] = accel_time
                self.core.srv_set_reg("group", group, "Profile_Acceleration", int(accel_time * 1000))

    ### @brief Moves the selected leg's foot position relative to its current foot position
    ### @param leg - name of the leg to move
    ### @param p_f_inc - desired relative point to add to the current foot_point
    ### @param moving_time - time [sec] that each joint should spend moving
    ### @param accel_time - time [sec] that each joint should spend accelerating
    ### @param blocking - True if the function should wait 'moving_time' seconds before returning
    def move_leg(self, leg, p_f_inc=[0, 0, 0], moving_time=0.15, accel_time=0.075, blocking=True):
        self.leg_mode_on = True
        self.set_trajectory_time(leg, moving_time, accel_time)
        point = list(self.foot_points[leg])
        target_point = np.add(point, p_f_inc)
        theta, success = self.solve_ik(target_point, leg)
        if not success: return False
        theta_names = [leg + "_coxa", leg + "_femur", leg + "_tibia"]
        for x in range(len(theta_names)):
            if not (self.info.joint_lower_limits[self.info_index_map[theta_names[x]]] <= theta[x] <= self.info.joint_upper_limits[self.info_index_map[theta_names[x]]]):
                return False
        command = JointGroupCommand(name=leg, cmd=theta)
        self.core.pub_group.publish(command)
        self.foot_points[leg] = list(target_point)
        if blocking:
            rospy.sleep(moving_time)

    ### @brief Move the hexapod 'base_link' frame in place
    ### @param x - desired 'x' component of self.T_fb
    ### @param y - desired 'y' component of self.T_fb
    ### @param z - desired 'z' component of self.T_fb
    ### @param roll - desired 'roll' component of self.T_fb
    ### @param pitch - desired 'pitch' component of self.T_fb
    ### @param yaw - desired 'yaw' component of self.T_fb
    ### @param moving_time - time [sec] that each joint should spend moving
    ### @param accel_time - time [sec] that each joint should spend accelerating
    ### @param blocking - True if the function should wait 'moving_time' seconds before returning; False otherwise
    ### @return <bool> - True if function completed successfully; False otherwise
    def move_in_place(self, x=None, y=None, z=None, roll=None, pitch=None, yaw=None, moving_time=1.0, accel_time=0.3, blocking=True):
        self.set_trajectory_time("all", moving_time, accel_time)
        T_fb = np.array(self.T_fb)
        if x is not None: self.T_fb[0,3] = x
        if y is not None: self.T_fb[1,3] = y
        if z is not None: self.T_fb[2,3] = z
        rpy = ang.rotationMatrixToEulerAngles(self.T_fb[:3,:3])
        if roll is not None: rpy[0] = roll
        if pitch is not None: rpy[1] = pitch
        if yaw is not None: rpy[2] = yaw
        self.T_fb[:3,:3] = ang.eulerAnglesToRotationMatrix(rpy)
        for leg, point in self.foot_points.items():
            success = self.update_joint_command(point, leg)
            if not success:
                self.T_fb = T_fb
                return False
        self.core.pub_group.publish(self.hexapod_command)
        self.update_tfb_transform(moving_time)
        if blocking: rospy.sleep(moving_time)
        return True

    ### @brief Move the hexapod 'base_footprint' frame relative to the 'odom' frame
    ### @param x_stride - desired positive/negative distance to cover in a gait cycle relative to the base_footprint's X-axis
    ### @param y_stride - desired positive/negative distance to cover in a gait cycle relative to the base_footprint's Y-axis
    ### @param yaw_stride - desired positive/negative distance to cover in a gait cycle around the base_footprint's Z-axis
    ### @param max_foot_height - max height [meters] that a leg's foot will be lifted during the 'swing' phase
    ### @param num_steps - number of steps to complete one wave in the first sinusoid function
    ### @param gait_type - desired gait to use
    ### @param mp - time [sec] that each joint should spend moving per step
    ### @param ap - time [sec] that each joint should spend accelerating per step
    ### @param num_cycles - number of gait cycles to complete before exiting
    ### @param cycle_freq - frequency at which the gait cycle should run; defaults to 'num_steps'
    ### @return <bool> - True if function completed successfully; False otherwise
    def move_in_world(self, x_stride=0, y_stride=0, yaw_stride=0, max_foot_height=0.04, num_steps=20.0, gait_type="tripod", mp=0.150, ap=0.075, num_cycles=1, cycle_freq=None):
        self.set_trajectory_time("all", mp, ap)
        self.num_steps = num_steps
        num_steps_in_cycle = self.num_steps * self.gait_factors[gait_type]/2.0
        if cycle_freq is None: cycle_freq = num_steps
        rate = rospy.Rate(cycle_freq)
        for cycle in range(num_cycles):
            self.step_cntr = 1
            self.inc_prev = 0
            while (self.step_cntr <= num_steps_in_cycle and not rospy.is_shutdown()):
                inc = 1/self.gait_factors[gait_type] * 0.5*(1 + math.sin(2*np.pi*(self.step_cntr/self.num_steps) - np.pi/2))
                foot_height = max_foot_height * 0.5*(1 + math.sin(4*np.pi*(self.step_cntr/self.num_steps) - np.pi/2))

                success = False
                if gait_type == "tripod":
                    success = self.tripod_gait(x_stride, y_stride, yaw_stride, inc, foot_height)
                elif gait_type == "ripple":
                    success = self.ripple_gait(x_stride, y_stride, yaw_stride, inc, foot_height)
                elif gait_type == "wave":
                    success = self.wave_gait(x_stride, y_stride, yaw_stride, inc, foot_height)
                if not success:
                    self.reset_hexapod()
                    return False

                aug_inc = abs(inc - self.inc_prev)
                temp_point = [aug_inc * x_stride, aug_inc * y_stride, 0]
                world_point = np.dot(self.T_sf[:3,:3], temp_point)
                self.T_sf[:3,3] += world_point
                rpy = ang.rotationMatrixToEulerAngles(self.T_sf[:3,:3])
                rpy[2] += aug_inc * yaw_stride
                self.T_sf[:3,:3] = ang.eulerAnglesToRotationMatrix(rpy)
                self.core.pub_group.publish(self.hexapod_command)
                self.update_tsf_transform(mp)

                self.inc_prev = inc
                self.step_cntr += 1.0
                rate.sleep()
        return True

    ### @brief Makes the hexapod walk using a tripod gait
    ### @param x_stride - desired positive/negative distance to cover in a gait cycle relative to the base_footprint's X-axis
    ### @param y_stride - desired positive/negative distance to cover in a gait cycle relative to the base_footprint's Y-axis
    ### @param yaw_stride - desired positive/negative distance to cover in a gait cycle around the base_footprint's Z-axis
    ### @param inc - output from the 'first' sinusoidal function (as described in the Notes above) that describes the desired 'x' and 'y' position for a given leg's foot
    ### @param foot_height - output from the 'second' sinusoidal function (as described in the Notes above) that describes the desired 'z' position for a given leg's foot
    ### @return <bool> - True if function completed successfully; False otherwise
    def tripod_gait(self, x_stride, y_stride, yaw_stride, inc, foot_height):
        x_inc = inc * x_stride
        y_inc = inc * y_stride
        yaw_inc = inc * yaw_stride

        for leg in self.leg_list:
            new_point = []
            T_osc = np.identity(4)
            if (leg == "right_front" or leg == "right_back" or leg == "left_middle"):
                T_osc[:3,:3] = ang.eulerAnglesToRotationMatrix([0, 0, -yaw_inc])
                p_f = [-x_inc, -y_inc, 0 if self.step_cntr < self.num_steps/2.0 else foot_height]
            else:
                T_osc[:3,:3] = ang.eulerAnglesToRotationMatrix([0, 0, yaw_inc])
                p_f = [x_inc, y_inc, 0 if self.step_cntr > self.num_steps/2.0 else foot_height]
            T_osc[:3,3] = p_f
            new_point = np.dot(T_osc, np.r_[self.foot_points[leg], 1])
            success = self.update_joint_command(new_point[:3], leg)
            if not success: return False
        return True

    ### @brief Makes the hexapod walk using a ripple gait
    ### @param x_stride - desired positive/negative distance to cover in a gait cycle relative to the base_footprint's X-axis
    ### @param y_stride - desired positive/negative distance to cover in a gait cycle relative to the base_footprint's Y-axis
    ### @param yaw_stride - desired positive/negative distance to cover in a gait cycle around the base_footprint's Z-axis
    ### @param inc - output from the 'first' sinusoidal function (as described in the Notes above) that describes the desired 'x' and 'y' position for a given leg's foot
    ### @param foot_height - output from the 'second' sinusoidal function (as described in the Notes above) that describes the desired 'z' position for a given leg's foot
    ### @return <bool> - True if function completed successfully; False otherwise
    def ripple_gait(self, x_stride, y_stride, yaw_stride, inc, foot_height):
        for pair in self.ripple_leg_pairs:
            z_inc = 0
            new_point = []
            T_osc = np.identity(4)
            if pair != self.ripple_leg_pairs[0]:
                self.ripple_incs[pair] -= abs(inc - self.inc_prev)
            else:
                self.ripple_incs[pair] += abs(inc - self.inc_prev) * 2.0
                z_inc = foot_height
            x_inc = self.ripple_incs[pair] * x_stride
            y_inc = self.ripple_incs[pair] * y_stride
            yaw_inc = self.ripple_incs[pair] * yaw_stride
            p_f = [x_inc, y_inc, z_inc]
            T_osc[:3,3] = p_f
            T_osc[:3,:3] = ang.eulerAnglesToRotationMatrix([0, 0, yaw_inc])
            for leg in self.ripple_legs[pair]:
                new_point = np.dot(T_osc, np.r_[self.foot_points[leg], 1])
                success = self.update_joint_command(new_point[:3], leg)
                if not success:
                    self.ripple_leg_pairs = ["first", "second", "third"]
                    self.ripple_incs = {p:0 for p in self.ripple_leg_pairs}
                    self.period_cntr = 0
                    return False
        self.period_cntr += 1.0
        if (self.period_cntr == self.num_steps/2.0):
            old_pair = self.ripple_leg_pairs.pop(0)
            self.ripple_leg_pairs.append(old_pair)
            self.period_cntr = 0
        return True

    ### @brief Makes the hexapod walk using a wave gait
    ### @param x_stride - desired positive/negative distance to cover in a gait cycle relative to the base_footprint's X-axis
    ### @param y_stride - desired positive/negative distance to cover in a gait cycle relative to the base_footprint's Y-axis
    ### @param yaw_stride - desired positive/negative distance to cover in a gait cycle around the base_footprint's Z-axis
    ### @param inc - output from the 'first' sinusoidal function (as described in the Notes above) that describes the desired 'x' and 'y' position for a given leg's foot
    ### @param foot_height - output from the 'second' sinusoidal function (as described in the Notes above) that describes the desired 'z' position for a given leg's foot
    ### @return <bool> - True if function completed successfully; False otherwise
    def wave_gait(self, x_stride, y_stride, yaw_stride, inc, foot_height):
        for leg in self.wave_legs:
            z_inc = 0
            new_point = []
            T_osc = np.identity(4)
            if leg != self.wave_legs[0]:
                self.wave_incs[leg] -= abs(inc - self.inc_prev)
            else:
                self.wave_incs[leg] += abs(inc - self.inc_prev) * 5.0
                z_inc = foot_height
            x_inc = self.wave_incs[leg] * x_stride
            y_inc = self.wave_incs[leg] * y_stride
            yaw_inc = self.wave_incs[leg] * yaw_stride
            p_f = [x_inc, y_inc, z_inc]
            T_osc[:3,3] = p_f
            T_osc[:3,:3] = ang.eulerAnglesToRotationMatrix([0, 0, yaw_inc])
            new_point = np.dot(T_osc, np.r_[self.foot_points[leg], 1])
            success = self.update_joint_command(new_point[:3], leg)
            if not success:
                self.wave_legs = ["right_front", "left_front", "right_middle", "left_middle", "right_back", "left_back"]
                self.wave_incs = {l:0 for l in self.wave_legs}
                self.period_cntr = 0
                return False
        self.period_cntr += 1.0
        if (self.period_cntr == self.num_steps/2.0):
            old_leg = self.wave_legs.pop(0)
            self.wave_legs.append(old_leg)
            self.period_cntr = 0
        return True

    ### @brief Move the hexapod 'base_footprint' frame relative to the 'odom' frame in uneven terrain (utilizes the tripod gait)
    ### @param x_stride - desired positive/negative distance to cover in a gait cycle relative to the base_footprint's X-axis
    ### @param y_stride - desired positive/negative distance to cover in a gait cycle relative to the base_footprint's Y-axis
    ### @param yaw_stride - desired positive/negative distance to cover in a gait cycle around the base_footprint's Z-axis
    ### @param max_foot_height - max height [meters] that a leg's foot will be lifted during the 'swing' phase
    ### @param leg_up_time - time in seconds that it should take for a foot to move up to 'max_foot_height'
    ### @param num_swing_steps - number of iterations to complete planar motion w.r.t. the XY plane
    ### @param mp - time [sec] that each joint should spend moving per step in num_swing_steps
    ### @param ap - time [sec] that each joint should spend moving per step in num_swing_steps
    ### @param leg_down_inc - length [meters] that a leg moves down every iteration
    ### @param threshold - the femur motor current [mA] above 0 that is considered a 'ground touch' for the leg
    ### @param reset_foot_points - set to True to reset 'self.foot_points' to 'self.home_foot_points' before moving the hexapod
    ### @param reset_height - if resetting foot points, this sets the 'z' value of self.T_fb
    ### @param num_cycles - number of gait cycles to complete before exiting
    ### @param cycle_freq - frequency at which the gait cycle should run
    ### @return <bool> - True if function completed successfully; False otherwise
    def move_in_world_rough(self, x_stride=0, y_stride=0, yaw_stride=0, max_foot_height=0.02, leg_up_time=0.5, num_swing_steps=10.0, mp=0.150, ap=0.075, leg_down_inc=0.001, threshold=70, reset_foot_points=False, reset_height=0.12, num_cycles=1, cycle_freq=20.0):

        if reset_foot_points:
            self.foot_points = copy.deepcopy(self.home_foot_points)
            self.move_in_place(z=reset_height)

        # setup 'tripod' gait variables
        first_set = ["left_front", "left_back", "right_middle"]
        second_set = ["right_front", "right_back", "left_middle"]
        sets = [first_set, second_set]
        rate = rospy.Rate(cycle_freq)
        for x in range(num_cycles):
            for set in sets:
                # Move all legs in a set up to 'max_foot_height'
                for leg in set:
                    new_point = np.r_[self.foot_points[leg][:2], max_foot_height]
                    success = self.update_joint_command(new_point, leg)
                    if not success: return False
                    self.foot_points[leg][2] = max_foot_height
                self.set_trajectory_time("all", leg_up_time, leg_up_time/2.0)
                self.core.pub_group.publish(self.hexapod_command)
                time_start = rospy.get_time()

                # Move all legs in the XY plane according to the 'stride' parameters
                self.set_trajectory_time("all", mp, ap)
                time_diff = leg_up_time - (rospy.get_time() - time_start)
                if time_diff > 0: rospy.sleep(time_diff)
                inc_prev = 0
                for step in range(1, int(num_swing_steps) + 1):
                    inc = 0.25*(1 + math.sin(np.pi*(step/num_swing_steps) - np.pi/2))
                    x_inc = inc * x_stride
                    y_inc = inc * y_stride
                    yaw_inc = inc * yaw_stride
                    for leg, point in self.foot_points.items():
                        T_osc = np.identity(4)
                        if leg not in set:
                            T_osc[:3,:3] = ang.eulerAnglesToRotationMatrix([0, 0, -yaw_inc])
                            p_f = [-x_inc, -y_inc, 0]
                        else:
                            T_osc[:3,:3] = ang.eulerAnglesToRotationMatrix([0, 0, yaw_inc])
                            p_f = [x_inc, y_inc, 0]
                        T_osc[:3,3] = p_f
                        new_point = np.dot(T_osc, np.r_[self.foot_points[leg], 1])
                        success = self.update_joint_command(new_point[:3], leg)
                        if not success: return False
                        if step == num_swing_steps:
                            self.foot_points[leg] = list(new_point[:3])
                    aug_inc = abs(inc - inc_prev)
                    temp_point = [aug_inc * x_stride, aug_inc * y_stride, 0]
                    world_point = np.dot(self.T_sf[:3,:3], temp_point)
                    self.T_sf[:3,3] += world_point
                    rpy = ang.rotationMatrixToEulerAngles(self.T_sf[:3,:3])
                    rpy[2] += aug_inc * yaw_stride
                    self.T_sf[:3,:3] = ang.eulerAnglesToRotationMatrix(rpy)
                    self.core.pub_group.publish(self.hexapod_command)
                    self.update_tsf_transform(mp)
                    inc_prev = inc
                    rate.sleep()

                # Move all legs in a set down until 'ground touch' is achieved
                all_feet_grounded = False
                current_foot_height = max_foot_height
                time_start = rospy.get_time()
                while not all_feet_grounded:
                    all_feet_grounded = True
                    current_foot_height -= leg_down_inc
                    for leg in set:
                        if (rospy.get_time() <= time_start + 0.6):
                            new_point = np.r_[self.foot_points[leg][:2], current_foot_height]
                            success = self.update_joint_command(new_point, leg)
                            if not success: continue
                            self.foot_points[leg][2] = current_foot_height
                            all_feet_grounded = False
                        elif (rospy.get_time() > time_start + 0.6):
                            joint_effort = self.core.joint_states.effort[self.core.js_index_map[leg + "_femur"]]
                            if (joint_effort < threshold):
                                new_point = np.r_[self.foot_points[leg][:2], current_foot_height]
                                success = self.update_joint_command(new_point, leg)
                                if not success: continue
                                self.foot_points[leg][2] = current_foot_height
                                all_feet_grounded = False
                    self.core.pub_group.publish(self.hexapod_command)
                    rate.sleep()
        return True

    ### @brief Get current odometry
    ### @return <list> - 3-element-list containing the 2D pose of the hexapod (as [x, y, theta])
    ### @details - odometry is from <robot_name>/odom to <robot_name>/base_footprint
    def get_odometry(self):
        rpy = ang.rotationMatrixToEulerAngles(self.T_sf[:3,:3])
        return [self.T_sf[0,3], self.T_sf[1,3], rpy[2]]

    ### @brief Get current body bose (i.e. self.T_fb)
    ### @return <4x4 matrix> - copy of self.T_fb
    def get_body_pose(self):
        return np.array(self.T_fb)

    ### @brief Get all six hexapod initial foot points (feet location before beginning a gait cycle)
    ### @param foot_points - dictionary containing all six hexapod foot points
    ### @details - each foot point is the [x, y, z] location of the 'foot_link' frame relative to the 'base_footprint' frame;
    ###            the keys to the dictionary are the 'leg' names in 'self.leg_list'
    def get_foot_points(self):
        foot_points = copy.deepcopy(self.foot_points)
        return foot_points

    ### @brief Set one or all initial hexapod foot points (foot/feet location before beginning a gait cycle)
    ### @param leg - one of the legs defined in 'self.leg_list' or 'all' if passing in a dictionary containg all desired foot points
    ### @param foot_points - a list containing the [x, y, z] location of the 'foot_link' frame for the given leg relative to the 'base_footprint' frame OR
    ###                      a dictionary containing each leg's location of the 'foot_link' frame relative to the 'base_footprint' frame (keys are the leg names defined in 'self.leg_list')
    def set_foot_points(self, leg, foot_points):
        if leg == "all":
            self.foot_points = copy.deepcopy(foot_points)
        else:
            self.foot_points[leg] = copy.deepcopy(foot_points)

    ### @brief Set one or all initial hexapod home foot points (foot/feet location before beginning a gait cycle)
    ### @param leg - one of the legs defined in 'self.leg_list' or 'all' if passing in a dictionary containg all desired foot points
    ### @param foot_points - a list containing the [x, y, z] location of the 'foot_link' frame for the given leg relative to the 'base_footprint' frame OR
    ###                      a dictionary containing each leg's location of the 'foot_link' frame relative to the 'base_footprint' frame (keys are the leg names defined in 'self.leg_list')
    def set_home_foot_points(self, leg, foot_points):
        if leg == "all":
            self.home_foot_points = copy.deepcopy(foot_points)
        else:
            self.home_foot_points[leg] = copy.deepcopy(foot_points)

    def set_home_height(self, height):
        self.home_height = height
