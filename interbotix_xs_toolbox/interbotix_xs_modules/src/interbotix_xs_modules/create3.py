import rospy
import actionlib
from geometry_msgs.msg import Twist, Vector3, Pose, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from irobot_create_msgs.msg import AudioNote, AudioNoteVector, WheelTicks

SOUND_END = AudioNoteVector(
    notes=[
        AudioNote(frequency=261, max_runtime=rospy.Duration(secs=0.15)),
        AudioNote(frequency=294, max_runtime=rospy.Duration(secs=0.15)),
        AudioNote(frequency=330, max_runtime=rospy.Duration(secs=0.15)),
    ],
    append=True
)


### @brief Definition of the Interbotix Create 3 Module
### @param robot_name - namespace of the Create3 nodes (a.k.a the name of the Interbotix LoCoBot)
### @param base_joint_states - name of the joints states topic that contains the states of the
###        base's two wheels
### @param use_move_base_action - whether or not Move-Base's Action Server should be used instead
###        of the Topic interface; set to `True` to make the 'move_to_pose' function block until
###        the robot reaches its goal pose
class InterbotixCreate3Interface(object):
    def __init__(
        self,
        robot_name,
        base_joint_states="/wheel_ticks",
        use_move_base_action=False,
    ):
        self.robot_name = robot_name
        self.odom = Pose()
        self.wheel_states = WheelTicks()
        self.use_move_base_action = use_move_base_action
        if (self.use_move_base_action):
            self.mb_client = actionlib.SimpleActionClient(
                ns="/" + self.robot_name + "/move_base",
                ActionSpec=MoveBaseAction,
            )
            self.mb_client.wait_for_server()
        # ROS Publisher to command twists to the base
        self.pub_base_command = rospy.Publisher(
            name="/mobile_base/cmd_vel",
            data_class=Twist,
            queue_size=1,
        )
        # ROS Publisher to command audios to the base
        self.pub_base_sound = rospy.Publisher(
            name="/mobile_base/cmd_audio",
            data_class=AudioNoteVector,
            queue_size=1,
        )
        self.pub_base_pose = rospy.Publisher(
            name="/" + self.robot_name + "/move_base_simple/goal",
            data_class=PoseStamped,
            queue_size=1
        )
        # ROS Subscriber to process odometry of the base
        self.sub_base_odom = rospy.Subscriber(
            name="/mobile_base/odom",
            data_class=Odometry,
            callback=self.base_odom_cb,
        )
        self.sub_wheel_states = rospy.Subscriber(
            name=base_joint_states,
            data_class=WheelTicks,
            callback=self.wheel_states_cb,
        )
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
        self.command_audio_vector(SOUND_END)
        return True

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
        self.pub_base_sound.publish(
            AudioNoteVector(
                notes=[AudioNote(frequency=frequency, max_runtime=rospy.Duration(secs=duration))]
            )
        )

    ### @brief Command the Create3 base to play the given AudioNoteVector
    ### @param audio_vector - AudioNoteVector to be commanded
    def command_audio_vector(self, audio_vector):
        self.pub_base_sound.publish(audio_vector)

    ### @brief ROS Callback function to update the odometry of the robot
    ### @param msg - ROS Odometry message from the base
    def base_odom_cb(self, msg):
        self.odom = msg.pose.pose

    ### @brief ROS Callback function get get the wheel joint states
    ### @param msg - WheelTicks message from base
    def wheel_states_cb(self, msg):
        self.wheel_states = msg

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

    ### Get the current wheel positions
    ### @return <list> - 2 element list containing the wheel positions [rad]
    def get_wheel_states(self):
        return list([self.wheel_states.ticks_left, self.wheel_states.ticks_right])
