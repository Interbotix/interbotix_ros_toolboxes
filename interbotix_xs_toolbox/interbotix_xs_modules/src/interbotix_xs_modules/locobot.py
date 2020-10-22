import rospy
from std_msgs.msg import Empty
from kobuki_msgs.msg import Sound
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from interbotix_xs_modules.core import InterbotixRobotXSCore
from interbotix_xs_modules.arm import InterbotixArmXSInterface
from interbotix_xs_modules.turret import InterbotixTurretXSInterface
from interbotix_xs_modules.gripper import InterbotixGripperXSInterface


class InterbotixLocobotXS(object):
    def __init__(self, robot_model, arm_model=None, arm_group_name="arm", gripper_name="gripper", turret_group_name="camera", robot_name=None, init_node=True, dxl_joint_states="dynamixel/joint_states", kobuki_joint_states="mobile_base/joint_states"):
        self.dxl = InterbotixRobotXSCore(robot_model, robot_name, init_node, dxl_joint_states)
        self.camera = InterbotixTurretXSInterface(self.dxl, turret_group_name)
        self.base = InterbotixKobukiInterface(self.dxl.robot_name, kobuki_joint_states)
        if arm_model is not None:
            self.arm = InterbotixArmXSInterface(self.dxl, arm_model, arm_group_name)
            self.gripper = InterbotixGripperXSInterface(self.dxl, gripper_name)

class InterbotixKobukiInterface(object):
    def __init__(self, robot_name, kobuki_joint_states):
        self.robot_name = robot_name
        self.odom = None
        self.wheel_states = None
        self.pub_base_command = rospy.Publisher("/" + self.robot_name + "/mobile_base/commands/velocity", Twist, queue_size=1)                     # ROS Publisher to command twists to the Kobuki base
        self.pub_base_reset = rospy.Publisher("/" + self.robot_name + "/mobile_base/commands/reset_odometry", Empty, queue_size=1)                 # ROS Publisher to reset the base odometry
        self.pub_base_sound = rospy.Publisher("/" + self.robot_name + "/mobile_base/commands/sound", Sound, queue_size=1)
        self.sub_base_odom = rospy.Subscriber("/" + self.robot_name + "/mobile_base/odom", Odometry, self.base_odom_cb)
        self.sub_wheel_states = rospy.Subscriber("/" + self.robot_name + "/" + kobuki_joint_states, JointState, self.wheel_states_cb)

    def move(self, x=0, yaw=0, duration=1.0):
        time_start = rospy.get_time()
        r = rospy.Rate(10)
        while (rospy.get_time() < (time_start + duration)):
            self.pub_base_command.publish(Twist(linear=Vector3(x=x), angular=Vector3(z=yaw)))
            r.sleep()
        self.pub_base_command.publish(Twist())

    def command_velocity(self, x=0, yaw=0):
        self.pub_base_command.publish(Twist(linear=Vector3(x=x), angular=Vector3(z=yaw)))

    def base_odom_cb(self, msg):
        self.odom = msg.pose.pose

    def wheel_states_cb(self, msg):
        self.wheel_states = msg

    def get_odom(self):
        quat = (self.odom.orientation.x, self.odom.orientation.y, self.odom.orientation.z, self.odom.orientation.w)
        euler = euler_from_quaternion(quat)
        pose = [pose.position.x, pose.position.y, euler[2]]
        return pose

    def get_wheel_states(self):
        return list(self.wheel_states.position)

    def reset_odom(self):
        self.pub_base_reset.publish(Empty())
        self.pub_base_sound.publish(Sound(value=Sound.CLEANINGEND))
