import sys
import rospy
import actionlib
from std_msgs.msg import Empty

# handles imports of different base message types
kobuki_msgs_found = False
create_msgs_found = False
try:
    from kobuki_msgs.msg import Sound, AutoDockingAction, AutoDockingGoal
    kobuki_msgs_found = True
except ImportError:
    pass
try:
    from irobot_create_msgs.msg import AudioNote, AudioNoteVector
    create_msgs_found = True
except ImportError:
    pass
if not (kobuki_msgs_found or create_msgs_found):
    rospy.logerr("Could not find the messages required to operate the base. Quitting...")
    sys.exit(1)

from geometry_msgs.msg import Twist, Vector3, PoseStamped, Quaternion
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from interbotix_xs_modules.core import InterbotixRobotXSCore
from interbotix_xs_modules.arm import InterbotixArmXSInterface
from interbotix_xs_modules.turret import InterbotixTurretXSInterface
from interbotix_xs_modules.gripper import InterbotixGripperXSInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface
from interbotix_perception_modules.armtag import InterbotixArmTagInterface


### @brief Standalone Module to control an Interbotix LoCoBot with a Kobuki base
### @param robot_model - Interbotix LoCoBot model (ex. 'locobot_px100' or 'locobot_base')
### @param arm_model - Interbotix Manipulator model (ex. 'mobile_px100' or 'mobile_wx200'); if set
###        to `None`, does not add an arm, gripper, or apriltag component to the robot
### @param arm_group_name - joint group name that contains the 'arm' joints as defined in the
###        'motor_config' yaml file
### @param gripper_name - name of the gripper joint as defined in the 'motor_config' yaml file;
###        typically, this is 'gripper'
### @param turret_group_name - joint group name that contains the 'turret' joints as defined in the
###        'motor_config' yaml file; typically, this is 'camera'
### @param robot_name - defaults to the value given to 'robot_model' if unspecified; this can be
###        customized if controlling two or more locobots from one computer (like 'locobot1' and
###        'locobot2')
### @param init_node - set to `True` if the InterbotixRobotXSCore class should initialize the ROS
###        node - this is the most Pythonic approach; to incorporate a robot into an existing ROS
###        node though, set to `False`
### @param dxl_joint_states - name of the joint states topic that contains just the states of the
###        dynamixel servos
### @param kobuki_joint_states - name of the joints states topic that contains the states of the
###        Kobuki's two wheels
### @param use_move_base_action - whether or not Move-Base's Action Server should be used instead
###        of the Topic interface; set to `True` to make the 'move_to_pose' function block until
###        the robot reaches its goal pose
class InterbotixLocobotKobukiXS(object):
    def __init__(
        self,
        robot_model,
        arm_model=None,
        arm_group_name="arm",
        gripper_name="gripper",
        turret_group_name="camera",
        robot_name="locobot",
        init_node=True,
        dxl_joint_states="dynamixel/joint_states",
        kobuki_joint_states="mobile_base/joint_states",
        use_move_base_action=False
    ):
        # Create DYNAMIXEL core interface
        self.dxl = InterbotixRobotXSCore(
            robot_model=robot_model,
            robot_name=robot_name,
            init_node=init_node,
            joint_state_topic=dxl_joint_states
        )
        # Create turret interface for the camera
        self.camera = InterbotixTurretXSInterface(
            core=self.dxl,
            group_name=turret_group_name
        )
        # Create Kobuki Interface if using base
        if rospy.has_param("/" + robot_name + "/use_base") and rospy.get_param("/" + robot_name + "/use_base"):
            self.base = InterbotixKobukiInterface(
                robot_name=robot_name,
                kobuki_joint_states=kobuki_joint_states,
                use_move_base_action=use_move_base_action
            )
        # Create PointCloud Interface if using perception
        if rospy.has_param("/" + robot_name + "/use_perception") and rospy.get_param("/" + robot_name + "/use_perception"):
            self.pcl = InterbotixPointCloudInterface(
                robot_name=robot_name + "/pc_filter",
                init_node=False
            )
        # Create Arm and Gripper interfaces if LoCoBot has an arm (if arm_model was specified)
        if arm_model is not None:
            self.arm = InterbotixArmXSInterface(
                core=self.dxl,
                robot_model=arm_model,
                group_name=arm_group_name
            )
            self.gripper = InterbotixGripperXSInterface(self.dxl, gripper_name)
            # Create ArmTag interface if using armtag
            if rospy.has_param("/" + robot_name + "/use_armtag") and rospy.get_param("/" + robot_name + "/use_armtag"):
                self.armtag = InterbotixArmTagInterface(
                    armtag_ns=robot_name + "/armtag",
                    apriltag_ns=robot_name + "/apriltag",
                    init_node=False
                )


### @brief Definition of the Interbotix Kobuki Module
### @param robot_name - namespace of the Kobuki node (a.k.a the name of the Interbotix LoCoBot)
### @param kobuki_joint_states - name of the joints states topic that contains the states of the
###        Kobuki's two wheels
### @param use_move_base_action - whether or not Move-Base's Action Server should be used instead
###        of the Topic interface; set to `True` to make the 'move_to_pose' function block until
###        the robot reaches its goal pose
class InterbotixKobukiInterface(object):
    def __init__(
        self,
        robot_name,
        kobuki_joint_states,
        use_move_base_action
    ):
        self.robot_name = robot_name
        self.odom = None
        self.wheel_states = None
        self.use_move_base_action = use_move_base_action
        if (self.use_move_base_action):
            self.mb_client = actionlib.SimpleActionClient(
                "/" + self.robot_name + "/move_base",
                MoveBaseAction
            )
            self.mb_client.wait_for_server()
        # ROS Publisher to command twists to the Kobuki base
        self.pub_base_command = rospy.Publisher(
            "/" + self.robot_name + "/mobile_base/commands/velocity",
            Twist,
            queue_size=1
        )
        # ROS Publisher to reset the base odometry
        self.pub_base_reset = rospy.Publisher(
            "/" + self.robot_name + "/mobile_base/commands/reset_odometry",
            Empty,
            queue_size=1
        )
        self.pub_base_sound = rospy.Publisher(
            "/" + self.robot_name + "/mobile_base/commands/sound",
            Sound,
            queue_size=1
        )
        self.pub_base_pose = rospy.Publisher(
            "/" + self.robot_name + "/move_base_simple/goal",
            PoseStamped,
            queue_size=1
        )
        self.sub_base_odom = rospy.Subscriber(
            "/" + self.robot_name + "/mobile_base/odom",
            Odometry,
            self.base_odom_cb
        )
        self.sub_wheel_states = rospy.Subscriber(
            "/" + self.robot_name + "/" + kobuki_joint_states,
            JointState,
            self.wheel_states_cb
        )
        rospy.sleep(0.5)
        print("Initialized InterbotixKobukiInterface!\n")

    ### @brief Move the base for a given amount of time
    ### @param x - desired speed [m/s] in the 'x' direction (forward/backward)
    ### @param yaw - desired angular speed [rad/s] around the 'z' axis
    ### @param duration - desired time [sec] that the robot should follow the specified speeds
    def move(self, x=0, yaw=0, duration=1.0):
        time_start = rospy.get_time()
        r = rospy.Rate(10)
        while (rospy.get_time() < (time_start + duration)):
            self.pub_base_command.publish(Twist(linear=Vector3(x=x), angular=Vector3(z=yaw)))
            r.sleep()
        self.pub_base_command.publish(Twist())

    ### @brief Move the base to a given pose in a Map (Nav Stack must be enabled!)
    ### @param x - desired 'x' position [m] w.r.t. the map frame that the robot should achieve
    ### @param y - desired 'y' position [m] w.r.t. the map frame that the robot should achieve
    ### @param yaw - desired yaw [rad] w.r.t. the map frame that the robot should achieve
    ### @param wait - whether the function should wait until the base reaches its goal pose before
    ###        returning
    ### @return <bool> - whether the robot successfully reached its goal pose (only applies if
    ###        'wait' is `True`)
    ### @details - note that if 'wait' is `False`, the function will always return `True`.
    def move_to_pose(self, x, y, yaw, wait=False):
        target_pose = PoseStamped()
        target_pose.header.frame_id = "map"
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        quat = quaternion_from_euler(0, 0, yaw)
        target_pose.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
        if (wait and self.use_move_base_action):
            goal = MoveBaseGoal(target_pose)
            self.mb_client.send_goal(goal)
            self.mb_client.wait_for_result()
            if not (self.mb_client.get_state() == actionlib.GoalStatus.SUCCEEDED):
                rospy.logerr("Did not successfully reach goal...")
                return False
        else:
            self.pub_base_pose.publish(target_pose)
        self.pub_base_sound.publish(Sound.CLEANINGEND)
        return True

    ### @brief Commands a Twist message to the base
    ### @param x - desired speed [m/s] in the 'x' direction (forward/backward)
    ### @param yaw - desired angular speed [rad/s] around the 'z' axis
    ### @details - This method can be called repeatedly to move the robot if using a controller
    def command_velocity(self, x=0, yaw=0):
        self.pub_base_command.publish(Twist(linear=Vector3(x=x), angular=Vector3(z=yaw)))

    ### @brief ROS Callback function to update the odometry of the robot
    ### @param msg - ROS Odometry message from the Kobuki
    def base_odom_cb(self, msg):
        self.odom = msg.pose.pose

    ### @brief ROS Callback function get get the wheel joint states
    ### @param msg - ROS JointState message from Kobuki
    def wheel_states_cb(self, msg):
        self.wheel_states = msg

    ### @brief Call action to automatically dock the base to charging station dock
    ### @details - must be near enough to dock to see IR signals (~1 meter in front)
    def auto_dock(self):
        rospy.loginfo("Attempting to autonomously dock to charging station.\n")
        try:
            # set docking action client
            ad_client = actionlib.SimpleActionClient(
                "/" + self.robot_name + "/dock_drive_action",
                AutoDockingAction
            )

            # connect to Action Server
            rospy.loginfo("Wating for auto_dock Action Server...")
            while not ad_client.wait_for_server(timeout=rospy.Duration(secs=5)):
                if rospy.is_shutdown(): return False
            rospy.loginfo("Found auto_dock Action Server")

            # set docking goal
            ad_goal = AutoDockingGoal()
            ad_client.send_goal(ad_goal)
            rospy.on_shutdown(ad_client.cancel_goal)

            rospy.loginfo("Attemping to dock...")
            # run action and wait 120 seconds for result
            ad_client.wait_for_result(rospy.Duration(secs=120))
            if ad_client.get_result():
                rospy.loginfo("Docking Successful.")
            else:
                rospy.loginfo("Docking Unsuccessful.")

        # catch interrupts
        except rospy.ROSInterruptException():
            rospy.logerr("Docking interrupted by user.")
            ad_client.cancel_goal()
        except Exception:
            rospy.logerr("Docking interrupted unexpectedly.")
            ad_client.cancel_goal()

    ### Get the 2D pose of the robot w.r.t. the robot 'odom' frame
    ### @return pose - list containing the [x, y, yaw] of the robot w.r.t. the odom frame
    def get_odom(self):
        quat = (
            self.odom.orientation.x,
            self.odom.orientation.y,
            self.odom.orientation.z,
            self.odom.orientation.w
        )
        return [
            self.odom.position.x,
            self.odom.position.y,
            euler_from_quaternion(quat)[2]
        ]

    ### Get the current wheel positions
    ### @return <list> - 2 element list containing the wheel positions [rad]
    def get_wheel_states(self):
        return list(self.wheel_states.position)

    ### Reset odometry to zero
    def reset_odom(self):
        self.pub_base_reset.publish(Empty())
        self.pub_base_sound.publish(Sound(value=Sound.CLEANINGEND))


### @brief Standalone Module to control an Interbotix LoCoBot with a Create3 base
### @param robot_model - Interbotix LoCoBot model (ex. 'locobot_px100' or 'locobot_base')
### @param arm_model - Interbotix Manipulator model (ex. 'mobile_px100' or 'mobile_wx200'); if set
#          to `None`, does not add an arm, gripper, or apriltag component to the robot
### @param arm_group_name - joint group name that contains the 'arm' joints as defined in the
###        'motor_config' yaml file
### @param gripper_name - name of the gripper joint as defined in the 'motor_config' yaml file;
###        typically, this is 'gripper'
### @param turret_group_name - joint group name that contains the 'turret' joints as defined in the
###        'motor_config' yaml file; typically, this is 'camera'
### @param robot_name - defaults to the value given to 'robot_model' if unspecified; this can be
###        customized if controlling two or more locobots from one computer (like 'locobot1' and
###        'locobot2')
### @param init_node - set to `True` if the InterbotixRobotXSCore class should initialize the ROS
###        node - this is the most Pythonic approach; to incorporate a robot into an existing ROS
###        node though, set to `False`
### @param dxl_joint_states - name of the joint states topic that contains just the states of the
###        dynamixel servos
class InterbotixLocobotCreate3XS(object):
    def __init__(
        self,
        robot_model,
        arm_model=None,
        arm_group_name="arm",
        gripper_name="gripper",
        turret_group_name="camera",
        robot_name="locobot",
        init_node=True,
        dxl_joint_states="dynamixel/joint_states",
    ):
        self.dxl = InterbotixRobotXSCore(robot_model, robot_name, init_node, dxl_joint_states)
        self.camera = InterbotixTurretXSInterface(self.dxl, turret_group_name)
        if rospy.has_param("/" + robot_name + "/use_base") and rospy.get_param("/" + robot_name + "/use_base") == True:
            self.base = InterbotixCreate3Interface(robot_name)
        if rospy.has_param("/" + robot_name + "/use_perception") and rospy.get_param("/" + robot_name + "/use_perception") == True:
            self.pcl = InterbotixPointCloudInterface(robot_name + "/pc_filter", False)
        if arm_model is not None:
            self.arm = InterbotixArmXSInterface(self.dxl, arm_model, arm_group_name)
            self.gripper = InterbotixGripperXSInterface(self.dxl, gripper_name)
            if rospy.has_param("/" + robot_name + "/use_armtag") and rospy.get_param("/" + robot_name + "/use_armtag") == True:
                self.armtag = InterbotixArmTagInterface(robot_name + "/armtag", robot_name + "/apriltag", False)


### @brief Definition of the Interbotix Create3 Module
### @param robot_name - namespace of the Create3 (a.k.a the name of the Interbotix LoCoBot)
class InterbotixCreate3Interface(object):
    def __init__(self, robot_name):
        self.robot_name = robot_name
        self.odom = None
        self.wheel_status = None
        # ROS Publisher to command twists to the base
        self.pub_base_command = rospy.Publisher("/" + self.robot_name + "/cmd_vel", Twist, queue_size=1)
        # ROS Publisher to command audios to the base
        self.pub_base_sound = rospy.Publisher("/" + self.robot_name + "/cmd_audio", AudioNoteVector, queue_size=1)
        # ROS Subscriber to process odometry of the base
        self.sub_base_odom = rospy.Subscriber("/" + self.robot_name + "/odom", Odometry, self.base_odom_cb)
        rospy.sleep(0.5)
        print("Initialized InterbotixCreate3Interface!\n")

    ### @brief Move the base for a given amount of time
    ### @param x - desired speed [m/s] in the 'x' direction (forward/backward)
    ### @param yaw - desired angular speed [rad/s] around the 'z' axis
    ### @param duration - desired time [sec] that the robot should follow the specified speeds
    def move(self, x=0, yaw=0, duration=1.0):
        time_start = rospy.get_time()
        r = rospy.Rate(10)
        # Publish Twist at 10 Hz for duration
        while (rospy.get_time() < (time_start + duration)):
            self.pub_base_command.publish(Twist(linear=Vector3(x=x), angular=Vector3(z=yaw)))
            r.sleep()
        # After the duration has passed, publish a zero Twist
        self.pub_base_command.publish(Twist())

    ### @brief Commands a Twist message to the base
    ### @param x - desired speed [m/s] in the 'x' direction (forward/backward)
    ### @param yaw - desired angular speed [rad/s] around the 'z' axis
    ### @details - This method can be called repeatedly to move the robot if using a controller
    def command_velocity(self, x=0, yaw=0):
        self.pub_base_command.publish(Twist(linear=Vector3(x=x), angular=Vector3(z=yaw)))

    ### @brief Command the Create3 base to play a frequency for a given duration
    ### @param frequency - frequency of the note in Hz
    ### @param duration - duration the note should be played for in seconds
    def command_audio(self, frequency, duration):
        self.pub_base_sound.publish(AudioNote(frequency=frequency, max_runtime=duration))

    ### @brief Command the Create3 base to play the given AudioNoteVector
    ### @param audio_vector - AudioNoteVector to be commanded
    def command_audio_vector(self, audio_vector):
        self.pub_base_sound.publish(audio_vector)

    ### @brief ROS Callback function to update the odometry of the robot
    ### @param msg - ROS Odometry message from the base
    def base_odom_cb(self, msg):
        self.odom = msg.pose.pose

    ### Get the 2D pose of the robot w.r.t. the robot 'odom' frame
    ### @return pose - list containing the [x, y, yaw] of the robot w.r.t. the odom frame
    def get_odom(self):
        quat = (
            self.odom.orientation.x,
            self.odom.orientation.y,
            self.odom.orientation.z,
            self.odom.orientation.w
        )
        return [self.odom.position.x, self.odom.position.y, euler_from_quaternion(quat)[2]]


# Alias Kobuki LoCoBot as 'default' LoCoBot
InterbotixLocobotXS = InterbotixLocobotKobukiXS
