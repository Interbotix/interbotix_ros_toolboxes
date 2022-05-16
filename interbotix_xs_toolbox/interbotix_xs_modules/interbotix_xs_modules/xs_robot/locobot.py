# Copyright 2022 Trossen Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from time import time
from typing import List

from geometry_msgs.msg import PoseStamped, Quaternion, Twist, Vector3
from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface
from interbotix_xs_modules.xs_robot.arm import InterbotixArmXSInterface
from interbotix_xs_modules.xs_robot.core import InterbotixRobotXSCore
from interbotix_xs_modules.xs_robot.gripper import InterbotixGripperXSInterface
from interbotix_xs_modules.xs_robot.turret import InterbotixTurretXSInterface
from kobuki_ros_interfaces.msg import AutoDockingAction, AutoDockingGoal, Sound
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from tf_transformations import euler_from_quaternion, quaternion_from_euler

raise NotImplementedError('The locobot module is not yet compatible with ROS2')


class InterbotixLocobotXS:
    """Standalone Module to control an Interbotix Locobot."""

    def __init__(
        self,
        robot_model: str,
        arm_model: str = None,
        arm_group_name: str = 'arm',
        gripper_name: str = 'gripper',
        turret_group_name: str = 'camera',
        robot_name: str = 'locobot',
        dxl_joint_states: str = 'dynamixel/joint_states',
        kobuki_joint_states: str = 'mobile_base/joint_states',
        use_move_base_action: bool = False,
    ):
        """
        Construct the Standalone InterbotixLocobotXS object.

        :param robot_model: Interbotix Locobot model (ex. 'locobot_px100' or 'locobot_base')
        :param arm_group_name: (optional) joint group name that contains the 'arm' joints as
            defined in the 'motor_config' yaml file
        :param gripper_name: (optional) name of the gripper joint as defined in the 'motor_config'
            yaml file; typically, this is 'gripper'
        :param turret_group_name: (optional) joint group name that contains the 'turret' joints as
            defined in the 'motor_config' yaml file; typically, this is 'camera'
        :param robot_name: (optional) defaults to the value given to 'robot_model' if unspecified;
            this can be customized if controlling two or more locobots from one computer (like
            'locobot1' and 'locobot2')
        :param dxl_joint_states: (optional) name of the joint states topic that contains just the
            states of the dynamixel servos
        :param kobuki_joint_states: (optional) name of the joints states topic that contains the
            states of the Kobuki's two wheels
        :param use_move_base_action: (optional) whether or not Move-Base's Action Server should be
            used instead of the Topic interface; set to `True` to make the 'move_to_pose' function
            block until the robot reaches its goal pose
        """
        self.core = InterbotixRobotXSCore(
            robot_model=robot_model,
            robot_name=robot_name,
            joint_states_topic=dxl_joint_states
        )
        self.camera = InterbotixTurretXSInterface(
            core=self.core,
            group_name=turret_group_name
        )
        self.core.declare_parameter(f'/{robot_name}/use_base')
        self.core.declare_parameter(f'/{robot_name}/use_perception')
        self.core.declare_parameter(f'/{robot_name}/use_armtag')
        use_base = self.core.get_parameter(
            f'/{robot_name}/use_base').get_parameter_value().bool_value
        use_perception = self.core.get_parameter(
            f'/{robot_name}/use_perception').get_parameter_value().bool_value
        use_armtag = self.core.get_parameter(
            f'/{robot_name}/use_armtag').get_parameter_value().bool_value
        if use_base:
            self.base = InterbotixKobukiInterface(
                core=self.core,
                robot_name=robot_name,
                kobuki_joint_states=kobuki_joint_states,
                use_mode_base_actions=use_move_base_action,
            )
        if use_perception:
            self.pcl = InterbotixPointCloudInterface(filter_ns=f'{robot_name}/pc_filter')
        if arm_model is not None:
            self.arm = InterbotixArmXSInterface(
                core=self.core,
                robot_model=arm_model,
                group_name=arm_group_name
            )
            self.gripper = InterbotixGripperXSInterface(core=self.core, gripper_name=gripper_name)
            if use_armtag:
                self.armtag = InterbotixArmTagInterface(
                    armtag_ns=f'{robot_name}/armtag',
                    apriltag_ns=f'{robot_name}/apriltag',
                    init_node=False
                )


class InterbotixKobukiInterface:
    """Definition of the Interbotix Kobuki Module."""

    def __init__(
        self,
        core: InterbotixRobotXSCore,
        robot_name: str,
        kobuki_joint_states: str,
        use_move_base_action: bool,
    ):
        """
        Construct the InterbotixKobukiInterface object.

        :param core: reference to the InterbotixRobotXSCore class containing the
            internal ROS plumbing that drives the Python API
        :param robot_name: namespace of the Kobuki node (a.k.a the name of the Interbotix LoCoBot)
        :param kobuki_joint_states: name of the joints states topic that contains the states of the
            Kobuki's two wheels
        :param use_move_base_action: whether or not Move-Base's Action Server should be used
            instead of the Topic interface; set to `True` to make the 'move_to_pose' function block
            until the robot reaches its goal pose
        """
        self.core = core
        self.robot_name = robot_name
        self.odom = None
        self.wheel_states = None
        self.use_move_base_action = use_move_base_action
        if (self.use_move_base_action):
            self.mb_client = ActionClient(
                node=self,
                action_type=MoveBaseAction,
                action_name=f'/{self.robot_name}/move_base',
            )
            self.mb_client.wait_for_server()
        # ROS Publisher to command twists to the Kobuki base
        self.pub_base_command = self.core.create_publisher(
            Twist,
            f'/{self.robot_name}/mobile_base/commands/velocity',
            1
        )
        # ROS Publisher to reset the base odometry
        self.pub_base_reset = self.core.create_publisher(
            Empty,
            f'/{self.robot_name}/mobile_base/commands/reset_odometry',
            1
        )
        self.pub_base_sound = self.core.create_publisher(
            Sound,
            f'/{self.robot_name}/mobile_base/commands/sound',
            1
        )
        self.pub_base_pose = self.core.create_publisher(
            PoseStamped,
            f'/{self.robot_name}/move_base_simple/goal',
            1
        )
        self.sub_base_odom = self.core.create_subscription(
            Odometry,
            f'/{self.robot_name}/mobile_base/odom',
            self.base_odom_cb
        )
        self.sub_wheel_states = self.core.create_subscription(
            JointState,
            f'/{self.robot_name}/{kobuki_joint_states}',
            self.wheel_states_cb
        )
        time.sleep(0.5)
        print('Initialized InterbotixKobukiInterface!\n')

    def move(
        self,
        x: float = 0,
        yaw: float = 0,
        duration: float = 1.0
    ) -> None:
        """
        Move the base for a given amount of time.

        :param x: (optional) desired speed [m/s] in the 'x' direction (forward/backward)
        :param yaw: (optional) desired angular speed [rad/s] around the 'z' axis
        :param duration: (optional) desired time [sec] that the robot should follow the specified
            speeds
        """
        time_start = self.core.get_clock().now()
        r = self.core.create_rate(frequency=10)
        while (self.core.get_clock().now() < (time_start + duration)):
            self.pub_base_command.publish(Twist(linear=Vector3(x=x), angular=Vector3(z=yaw)))
            r.sleep()
        self.pub_base_command.publish(Twist())

    def move_to_pose(
        self,
        x: float,
        y: float,
        yaw: float,
        wait: bool = False
    ) -> bool:
        """
        Move the base to a given pose in a Map (Nav Stack must be enabled!).

        :param x: desired 'x' position [m] w.r.t. the map frame that the robot should achieve
        :param y: desired 'y' position [m] w.r.t. the map frame that the robot should achieve
        :param yaw: desired yaw [rad] w.r.t. the map frame that the robot should achieve
        :param wait: whether the function should wait until the base reaches its goal pose before
            returning
        :return:  whether the robot successfully reached its goal pose (only applies if 'wait' is
             `True`)
        :details - note that if 'wait' is `False`, the function will always return `True`.
        """
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'map'
        target_pose.header.stamp = self.core.get_clock().now().to_msg()
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        quat = quaternion_from_euler(0, 0, yaw)
        target_pose.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
        if (wait and self.use_move_base_action):
            goal = MoveBaseGoal(target_pose)
            self.mb_client.send_goal(goal)
            self.mb_client.wait_for_result()
            if not (self.mb_client.succeed()):
                self.core.get_logger().error('Did not successfully reach goal.')
                return False
        else:
            self.pub_base_pose.publish(target_pose)
        self.pub_base_sound.publish(Sound.CLEANINGEND)
        return True

    def command_velocity(self, x: float = 0, yaw: float = 0) -> None:
        """
        Command a twist (velocity) message to move the robot.

        :param x: (optional) desired speed [m/s] in the 'x' direction (forward/backward)
        :param yaw: (optional) desired angular speed [rad/s] around the 'z' axis
        """
        self.pub_base_command.publish(Twist(linear=Vector3(x=x), angular=Vector3(z=yaw)))

    def base_odom_cb(self, msg: Odometry) -> None:
        """
        Update the odometry of the robot using a ROS callback function.

        :param msg: ROS Odometry message from the Kobuki
        """
        self.odom = msg.pose.pose

    def wheel_states_cb(self, msg: JointState):
        """
        Get the wheel joint states using a ROS callback function.

        :param msg: ROS JointState message from Kobuki
        """
        self.wheel_states = msg

    def auto_dock(self) -> bool:
        """
        Call action to automatically dock the base to charging station dock.

        :return: `True` if docked successfully, `False` otherwise
        :details: must be near enough to dock to see IR signals (~1 meter in front)
        """
        self.core.get_logger().info('Attempting to autonomously dock to charging station.')

        # set docking action client
        ad_client = ActionClient(
            AutoDockingAction,
            f'/{self.robot_name}/dock_drive_action',
        )

        # connect to Action Server
        self.core.get_logger().info('Wating for auto_dock Action Server...')
        while not ad_client.wait_for_server(timeout=Duration(seconds=5)):
            if not rclpy.ok():
                return False
        self.core.get_logger().info('Found auto_dock Action Server')

        # set docking goal
        ad_goal = AutoDockingGoal()
        ad_client.send_goal(ad_goal)
        # rospy.on_shutdown(ad_client.cancel_goal)

        self.core.get_logger().info('Attemping to dock...')
        # run action and wait 120 seconds for result
        ad_client.wait_for_result(Duration(seconds=120))

        if ad_client.get_result():
            self.core.get_logger().info('Docking Successful.')
            return True
        else:
            self.core.get_logger().warning('Docking Unsuccessful.')
            return False

    def get_odom(self) -> List[float]:
        """
        Get the 2D pose of the robot w.r.t. the robot 'odom' frame.

        :return: list containing the [x, y, yaw] of the robot w.r.t. the odom frame
        """
        euler = euler_from_quaternion(
            (
                self.odom.orientation.x,
                self.odom.orientation.y,
                self.odom.orientation.z,
                self.odom.orientation.w
            )
        )
        return [self.odom.position.x, self.odom.position.y, euler[2]]

    def get_wheel_states(self) -> List[float]:
        """
        Get the current wheel positions.

        :return: 2 element list containing the wheel positions [rad]
        """
        return list(self.wheel_states.position)

    def reset_odom(self) -> None:
        """Reset odometry to zero."""
        self.pub_base_reset.publish(Empty())
        self.pub_base_sound.publish(Sound(value=Sound.CLEANINGEND))
