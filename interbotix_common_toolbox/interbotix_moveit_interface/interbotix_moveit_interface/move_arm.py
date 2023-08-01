#!/usr/bin/env python3

# Copyright 2023 Trossen Robotics
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

"""
Contains classes used to control Interbotix X-Series Arms.

These classes can be used to control an X-Series standalone arm using Python.
"""

import math
from threading import Thread
import time
from typing import Any, List, Tuple, Union

from builtin_interfaces.msg import Duration
# import interbotix_common_modules.angle_manipulation as ang
# from interbotix_xs_modules.xs_robot import mr_descriptions as mrd
# from interbotix_xs_modules.xs_robot.core import InterbotixRobotXSCore
# from interbotix_xs_modules.xs_robot.gripper import InterbotixGripperXSInterface
# from interbotix_xs_msgs.msg import (
#     JointGroupCommand,
#     JointSingleCommand,
#     JointTrajectoryCommand,
# )
# from interbotix_xs_msgs.srv import RegisterValues, RobotInfo
# import modern_robotics as mr
import numpy as np
import rclpy
from rclpy.constants import S_TO_NS
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import LoggingSeverity
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# generic ros libraries
from rclpy.logging import get_logger

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)
from geometry_msgs.msg import PoseStamped, Pose


REV: float = 2 * np.pi
"""One complete revolution of a joint (2*pi)"""


class InterbotixManipulatorXS:
    """Standalone Module to control an Interbotix Arm and Gripper."""

    def __init__(
        self,
        robot_model: str,
        group_name: str = 'interbotix_arm',
        gripper_name: str = 'interbotix_gripper',
        robot_name: float = None,
        moving_time: float = 2.0,
        accel_time: float = 0.3,
        gripper_pressure: float = 0.5,
        gripper_pressure_lower_limit: int = 150,
        gripper_pressure_upper_limit: int = 350,
        topic_joint_states: str = 'joint_states',
        logging_level: LoggingSeverity = LoggingSeverity.INFO,
        moveit_node_name: str = 'moveit_py',
        start_on_init: bool = True,
        args=None,
    ) -> None:
        """
        Construct the Standalone Interbotix Manipulator module.

        :param robot_model: Interbotix Arm model (ex. 'wx200' or 'vx300s')
        :param group_name: (optional) joint group name that contains the 'arm' joints as defined in
            the 'motor_config' yaml file; typically, this is 'arm'
        :param gripper_name: (optional) name of the gripper joint as defined in the 'motor_config'
            yaml file; typically, this is 'gripper'
        :param robot_name: (optional) defaults to value given to `robot_model`; this can be
            customized if controlling two of the same arms from one computer (like 'arm1/wx200' and
            'arm2/wx200')
        :param moving_time: (optional) time [s] it should take for all joints in the arm to
            complete one move
        :param accel_time: (optional) time [s] it should take for all joints in the arm to
            accelerate/decelerate to/from max speed
        :param use_gripper: (optional) `True` if the gripper module should be initialized;
            otherwise, it won't be.
        :param gripper_pressure: (optional) fraction from 0 - 1 where '0' means the gripper
            operates at `gripper_pressure_lower_limit` and '1' means the gripper operates at
            `gripper_pressure_upper_limit`
        :param gripper_pressure_lower_limit: (optional) lowest 'effort' that should be applied to
            the gripper if `gripper_pressure` is set to 0; it should be high enough to open/close
            the gripper (~150 PWM or ~400 mA current)
        :param gripper_pressure_upper_limit: (optional) largest 'effort' that should be applied to
            the gripper if `gripper_pressure` is set to 1; it should be low enough that the motor
            doesn't 'overload' when gripping an object for a few seconds (~350 PWM or ~900 mA)
        :param topic_joint_states: (optional) the specific JointState topic output by the xs_sdk
            node
        :param logging_level: (optional) rclpy logging severity level. Can be DEBUG, INFO, WARN,
            ERROR, or FATAL. defaults to INFO
        :param node_name: (optional) name to give to the core started by this class, defaults to
            'robot_manipulation'
        :param start_on_init: (optional) set to `True` to start running the spin thread after the
            object is built; set to `False` if intending to sub-class this. If set to `False`,
            either call the `start()` method later on, or add the core to an executor in another
            thread.
        """
        # self.core = InterbotixRobotXSCore(
        #     robot_model=robot_model,
        #     robot_name=robot_name,
        #     topic_joint_states=topic_joint_states,
        #     logging_level=logging_level,
        #     node_name=node_name,
        #     args=args
        # )
        rclpy.init()

        self.arm = InterbotixArmXSInterface(
            robot_model=robot_model,
            group_name=group_name,
            moveit_node_name = moveit_node_name,
            moving_time=moving_time,
            accel_time=accel_time,

        )
        if gripper_name is not None:
            self.gripper = InterbotixGripperXSInterface(
                robot_model=robot_model,
                gripper_name=gripper_name,
                moveit_node_name = moveit_node_name,
                gripper_pressure=gripper_pressure,
                gripper_pressure_lower_limit=gripper_pressure_lower_limit,
                gripper_pressure_upper_limit=gripper_pressure_upper_limit,
            )

        # if start_on_init:
        #     self.start()

    # def start(self) -> None:
    #     """Start a background thread that builds and spins an executor."""
    #     self._execution_thread = Thread(target=self.run)
    #     self._execution_thread.start()

    # def run(self) -> None:
    #     """Thread target."""
    #     self.ex = MultiThreadedExecutor()
    #     self.ex.add_node(self.core)
    #     self.ex.spin()

    def shutdown(self) -> None:
        """Destroy the node and shut down all threads and processes."""
        # self.core.destroy_node()
        rclpy.shutdown()
        # self._execution_thread.join()
        # time.sleep(0.5)


class InterbotixArmXSInterface:
    """Definition of the Interbotix Arm component."""

    def __init__(
        self,
        robot_model: str,
        group_name: str,
        moveit_node_name: str = "moveit_py",
        moving_time: float = 2.0,
        accel_time: float = 0.3,
    ) -> None:
        """
        Construct the InterbotixArmXSInterface object.

        :param core: reference to the InterbotixRobotXSCore class containing the internal ROS
            plumbing that drives the Python API
        :param robot_model: Interbotix Arm model (ex. 'wx200' or 'vx300s')
        :param group_name: joint group name that contains the 'arm' joints as defined in the
            'motor_config' yaml file; typically, this is 'arm'
        :param moving_time: (optional) time [s] it should take for all joints in the arm to
            complete one move
        :param accel_time: (optional) time [s] it should take for all joints in the arm to
            accelerate/decelerate to/from max speed
        """
        self.logger = get_logger("moveit_py.pose_goal")

        # instantiate moveit_py instance and a planning component for the panda_arm
        self.robot = MoveItPy(node_name=moveit_node_name)
        self.planning_component = self.robot.get_planning_component(group_name)
        self.logger.info("MoveItPy instance created")

    def plan_and_execute(
        self,
        single_plan_parameters=None,
        multi_plan_parameters=None,
        sleep_time=0.0,
    ):
        """Helper function to plan and execute a motion."""
        # plan to goal
        self.logger.info("Planning trajectory")
        if multi_plan_parameters is not None:
            plan_result = self.planning_component.plan(
                multi_plan_parameters=multi_plan_parameters
            )
        elif single_plan_parameters is not None:
            plan_result = self.planning_component.plan(
                single_plan_parameters=single_plan_parameters
            )
        else:
            plan_result = self.planning_component.plan()

        # execute the plan
        if plan_result:
            self.logger.info("Executing plan")
            robot_trajectory = plan_result.trajectory
            self.robot.execute(robot_trajectory, controllers=[])
        else:
            self.logger.error("Planning failed")

        time.sleep(sleep_time)


    def go_to_home_pose(self):
        """Move robot to home configuartion"""
        self.planning_component.set_start_state_to_current_state()

        # set pose goal using predefined state
        self.planning_component.set_goal_state(configuration_name="home")

        # plan to goal
        self.plan_and_execute(sleep_time=3.0)

    def go_to_sleep_pose(self):
        """Move robot to home configuartion"""
        self.planning_component.set_start_state_to_current_state()

        # set pose goal using predefined state
        self.planning_component.set_goal_state(configuration_name="sleep")

        # plan to goal
        self.plan_and_execute(sleep_time=3.0)

    def go_to_pose(self):
        planning_scene_monitor = self.robot.get_planning_scene_monitor()
        with planning_scene_monitor.read_write() as scene:
            # instantiate a RobotState instance using the current robot model
            robot_state = scene.current_state
            robot_state.update()
            original_joint_positions = robot_state.get_joint_group_positions("interbotix_arm")
        # robot_state.get_joint_group_positions("interbotix_arm")
        # robot_model = self.robot.get_robot_model()
        # robot_state = RobotState(robot_model)
        # robot_state.update()


            robot_state.update()
            # link_pose = robot_state.get_global_link_transform("dx400/link_5")
            print("See initial joints:",original_joint_positions)


            # pose_goal.position.x = 0.223
            # pose_goal.position.y = -0.0056
            # pose_goal.position.z = 0.1192
            # pose_goal.orientation.x = 0.74
            # pose_goal.orientation.y = -0.642
            # pose_goal.orientation.z = 0.181
            # pose_goal.orientation.w = -0.087
    ##################################################
            pose_goal = Pose()
            pose_goal.position.x = -0.091
            pose_goal.position.y = 0.028
            pose_goal.position.z = 0.40
            pose_goal.orientation.x = -0.183
            pose_goal.orientation.y = -0.136
            pose_goal.orientation.z = -0.943
            pose_goal.orientation.w = -0.238
            # pose_goal.position.x = -0.0541
            # pose_goal.position.y = 0.07905
            # pose_goal.position.z = 0.27134
            # pose_goal.orientation.x = -0.352
            # pose_goal.orientation.y = -0.377
            # pose_goal.orientation.z = 0.303
            # pose_goal.orientation.w = 0.8012
            # robot_state.update()

            # randomize the robot state
            robot_state.set_from_ik(joint_model_group_name="interbotix_arm",
                geometry_pose=pose_goal,
                tip_name="dx400/link_5",
                timeout=2.0)

            robot_state.update()
            print("see joints:",robot_state.joint_positions)

            # set plan start state to current state
            self.planning_component.set_start_state_to_current_state()
            # robot_state.update()
            # print("Look", robot_state.joint_positions)

            # robot_state.update(force=True)

            # set goal state to the initialized robot state
            self.logger.info("Goal to go_to_pose")
            self.planning_component.set_goal_state(robot_state=robot_state)
            self.logger.info("Reached EE Pose")
            robot_state.update()
            # plan to goal
            self.plan_and_execute(sleep_time=3.0)

            # # set pose goal with PoseStamped message
            # self.planning_component.set_goal_state(pose_stamped_msg=pose_goal, pose_link="dx400/gripper_link")



    def shutdown(self) -> None:
        """Destroy the node and shut down all threads and processes."""
        rclpy.shutdown()


class InterbotixGripperXSInterface:
    """Definition of the Interbotix Gripper component."""

    def __init__(
        self,
        robot_model: str,
        gripper_name: str,
        moveit_node_name: str = "moveit_py",
        gripper_pressure: float = 0.5,
        gripper_pressure_lower_limit: int = 150,
        gripper_pressure_upper_limit: int = 350,
    ) -> None:
        """
        Construct the Interbotix Gripper Module.

        :param core: reference to the InterbotixRobotXSCore class containing the internal ROS
            plumbing that drives the Python API
        :param gripper_name: name of the gripper joint as defined in the 'motor_config' yaml file;
            typically, this is 'gripper'
        :param gripper_pressure: (optional) fraction from 0 - 1 where '0' means the gripper
            operates at 'gripper_pressure_lower_limit' and '1' means the gripper operates at
            'gripper_pressure_upper_limit'
        :param gripper_pressure_lower_limit: (optional) lowest 'effort' that should be applied to
            the gripper if gripper_pressure is set to 0; it should be high enough to open/close the
            gripper (~150 PWM or ~400 mA current)
        :param gripper_pressure_upper_limit: (optional) largest 'effort' that should be applied to
            the gripper if gripper_pressure is set to 1; it should be low enough that the motor
            doesn't 'overload' when gripping an object for a few seconds (~350 PWM or ~900 mA)
        """
        # rclpy.init()
        self.logger = get_logger("moveit_py.gripper")

        # instantiate moveit_py instance and a planning component for the panda_arm
        self.robot = MoveItPy(node_name=moveit_node_name)
        self.planning_component = self.robot.get_planning_component(gripper_name)
        self.logger.info("MoveItPy instance created")

    def plan_and_execute(
        self,
        single_plan_parameters=None,
        multi_plan_parameters=None,
        sleep_time=0.0,
    ):
        """Helper function to plan and execute a motion."""
        # plan to goal
        self.logger.info("Planning trajectory")
        if multi_plan_parameters is not None:
            plan_result = self.planning_component.plan(
                multi_plan_parameters=multi_plan_parameters
            )
        elif single_plan_parameters is not None:
            plan_result = self.planning_component.plan(
                single_plan_parameters=single_plan_parameters
            )
        else:
            plan_result = self.planning_component.plan()

        # execute the plan
        if plan_result:
            self.logger.info("Executing plan")
            robot_trajectory = plan_result.trajectory
            self.robot.execute(robot_trajectory, controllers=[])
        else:
            self.logger.error("Planning failed")

        time.sleep(sleep_time)

    def gripper_open(self):
        """Move robot to home configuartion"""
        self.planning_component.set_start_state_to_current_state()

        # set pose goal using predefined state
        self.planning_component.set_goal_state(configuration_name="open")

        # plan to goal
        self.plan_and_execute(sleep_time=3.0)

    def gripper_close(self):
        """Move robot to home configuartion"""
        self.planning_component.set_start_state_to_current_state()

        # set pose goal using predefined state
        self.planning_component.set_goal_state(configuration_name="close")

    #     # plan to goal
        self.plan_and_execute(sleep_time=3.0)


    def shutdown(self) -> None:
        """Destroy the node and shut down all threads and processes."""
        rclpy.shutdown()

    # def _publish_commands(
    #     self,
    #     positions: List[float],
    #     moving_time: float = None,
    #     accel_time: float = None,
    #     blocking: bool = True,
    # ) -> None:
    #     """
    #     Publish joint positions and block if necessary.

    #     :param positions: desired joint positions
    #     :param moving_time: (optional) duration in seconds that the robot should move
    #     :param accel_time: (optional) duration in seconds that that robot should spend
    #         accelerating/decelerating (must be less than or equal to half the moving_time)
    #     :param blocking: (optional) whether the function should wait to return control to the user
    #         until the robot finishes moving
    #     """
    #     self.core.get_logger().debug(f'Publishing {positions=}')
    #     self.set_trajectory_time(moving_time, accel_time)
    #     self.joint_commands = list(positions)
    #     joint_commands = JointGroupCommand(
    #         name=self.group_name, cmd=self.joint_commands
    #     )
    #     self.core.pub_group.publish(joint_commands)
    #     if blocking:
    #         time.sleep(
    #             self.moving_time
    #         )  # TODO: once released, use rclpy.clock().sleep_for()
    #     self._update_Tsb()

    # def set_trajectory_time(
    #     self,
    #     moving_time: float = None,
    #     accel_time: float = None
    # ) -> None:
    #     """
    #     Command the 'Profile_Velocity' and 'Profile_Acceleration' motor registers.

    #     :param moving_time: (optional) duration in seconds that the robot should move
    #     :param accel_time: (optional) duration in seconds that that robot should spend
    #         accelerating/decelerating (must be less than or equal to half the moving_time)
    #     """
    #     self.core.get_logger().debug(
    #         f'Updating timing params: {moving_time=}, {accel_time=}'
    #     )
    #     if moving_time is not None and moving_time != self.moving_time:
    #         self.moving_time = moving_time
    #         future_moving_time = self.core.srv_set_reg.call_async(
    #             RegisterValues.Request(
    #                 cmd_type='group',
    #                 name=self.group_name,
    #                 reg='Profile_Velocity',
    #                 value=int(moving_time * 1000),
    #             )
    #         )
    #         self.core.executor.spin_once_until_future_complete(
    #             future=future_moving_time,
    #             timeout_sec=0.1
    #         )

    #     if accel_time is not None and accel_time != self.accel_time:
    #         self.accel_time = accel_time
    #         future_accel_time = self.core.srv_set_reg.call_async(
    #             RegisterValues.Request(
    #                 cmd_type='group',
    #                 name=self.group_name,
    #                 reg='Profile_Acceleration',
    #                 value=int(accel_time * 1000),
    #             )
    #         )
    #         self.core.executor.spin_once_until_future_complete(
    #             future=future_accel_time,
    #             timeout_sec=0.1
    #         )

    # def _check_joint_limits(self, positions: List[float]) -> bool:
    #     """
    #     Ensure the desired arm group's joint positions are within their limits.

    #     :param positions: the positions [rad] to check
    #     :return: `True` if all positions are within limits; `False` otherwise
    #     """
    #     self.core.get_logger().debug(f'Checking joint limits for {positions=}')
    #     theta_list = [int(elem * 1000) / 1000.0 for elem in positions]
    #     speed_list = [
    #         abs(goal - current) / float(self.moving_time)
    #         for goal, current in zip(theta_list, self.joint_commands)
    #     ]
    #     # check position and velocity limits
    #     for x in range(self.group_info.num_joints):
    #         if not (
    #             self.group_info.joint_lower_limits[x]
    #             <= theta_list[x]
    #             <= self.group_info.joint_upper_limits[x]
    #         ):
    #             return False
    #         if speed_list[x] > self.group_info.joint_velocity_limits[x]:
    #             return False
    #     return True

    # def _check_single_joint_limit(self, joint_name: str, position: float) -> bool:
    #     """
    #     Ensure a desired position for a given joint is within its limits.

    #     :param joint_name: desired joint name
    #     :param position: desired joint position [rad]
    #     :return: `True` if within limits; `False` otherwise
    #     """
    #     self.core.get_logger().debug(
    #         f'Checking joint {joint_name} limits for {position=}'
    #     )
    #     theta = int(position * 1000) / 1000.0
    #     speed = abs(
    #         theta - self.joint_commands[self.info_index_map[joint_name]]
    #     ) / float(self.moving_time)
    #     ll = self.group_info.joint_lower_limits[self.info_index_map[joint_name]]
    #     ul = self.group_info.joint_upper_limits[self.info_index_map[joint_name]]
    #     vl = self.group_info.joint_velocity_limits[self.info_index_map[joint_name]]
    #     if not (ll <= theta <= ul):
    #         return False
    #     if speed > vl:
    #         return False
    #     return True

    # def set_joint_positions(
    #     self,
    #     joint_positions: List[float],
    #     moving_time: float = None,
    #     accel_time: float = None,
    #     blocking: bool = True,
    # ) -> bool:
    #     """
    #     Command positions to the arm joints.

    #     :param joint_positions: desired joint positions [rad]
    #     :param moving_time: (optional) duration in seconds that the robot should move
    #     :param accel_time: (optional) duration in seconds that that robot should spend
    #         accelerating/decelerating (must be less than or equal to half the moving_time)
    #     :param blocking: (optional) whether the function should wait to return control to the user
    #         until the robot finishes moving
    #     :return: `True` if position was commanded; `False` if it wasn't due to being outside limits
    #     """
    #     self.core.get_logger().debug(f'setting {joint_positions=}')
    #     if self._check_joint_limits(joint_positions):
    #         self._publish_commands(joint_positions, moving_time, accel_time, blocking)
    #         return True
    #     else:
    #         return False

    # def go_to_home_pose(
    #     self,
    #     moving_time: float = None,
    #     accel_time: float = None,
    #     blocking: bool = True
    # ) -> None:
    #     """
    #     Command the arm to go to its Home pose.

    #     :param moving_time: (optional) duration in seconds that the robot should move
    #     :param accel_time: (optional) duration in seconds that that robot should spend
    #         accelerating/decelerating (must be less than or equal to half the moving_time)
    #     :param blocking: (optional) whether the function should wait to return control to the user
    #         until the robot finishes moving
    #     """
    #     self.core.get_logger().debug('Going to home pose')
    #     self._publish_commands(
    #         positions=[0] * self.group_info.num_joints,
    #         moving_time=moving_time,
    #         accel_time=accel_time,
    #         blocking=blocking
    #     )

    # def go_to_sleep_pose(
    #     self,
    #     moving_time: float = None,
    #     accel_time: float = None,
    #     blocking: bool = True
    # ) -> None:
    #     """
    #     Command the arm to go to its Sleep pose.

    #     :param moving_time: (optional) duration in seconds that the robot should move
    #     :param accel_time: (optional) duration in seconds that that robot should spend
    #         accelerating/decelerating (must be less than or equal to half the moving_time)
    #     :param blocking: (optional) whether the function should wait to return control to the user
    #         until the robot finishes moving
    #     """
    #     self.core.get_logger().debug('Going to sleep pose')
    #     self._publish_commands(
    #         positions=self.group_info.joint_sleep_positions,
    #         moving_time=moving_time,
    #         accel_time=accel_time,
    #         blocking=blocking
    #     )

    # def set_single_joint_position(
    #     self,
    #     joint_name: str,
    #     position: float,
    #     moving_time: float = None,
    #     accel_time: float = None,
    #     blocking: bool = True,
    # ) -> bool:
    #     """
    #     Command a single joint to a desired position.

    #     :param joint_name: name of the joint to control
    #     :param position: desired position [rad]
    #     :param moving_time: (optional) duration in seconds that the robot should move
    #     :param accel_time: (optional) duration in seconds that that robot should spend
    #         accelerating/decelerating (must be less than or equal to half the moving_time)
    #     :param blocking: (optional) whether the function should wait to return control to the user
    #           until the robot finishes moving
    #     :return: `True` if single joint was set; `False` otherwise
    #     :details: Note that if a moving_time or accel_time is specified, the changes affect ALL the
    #         arm joints, not just the specified one
    #     """
    #     self.core.get_logger().debug(
    #         f'Setting joint {joint_name} to position={position}'
    #     )
    #     if not self._check_single_joint_limit(joint_name, position):
    #         return False
    #     self.set_trajectory_time(moving_time, accel_time)
    #     self.joint_commands[self.core.js_index_map[joint_name]] = position
    #     single_command = JointSingleCommand(name=joint_name, cmd=position)
    #     self.core.pub_single.publish(single_command)
    #     if blocking:
    #         time.sleep(self.moving_time)
    #     self._update_Tsb()
    #     return True

    # def set_ee_pose_matrix(
    #     self,
    #     T_sd: np.ndarray,
    #     custom_guess: List[float] = None,
    #     execute: bool = True,
    #     moving_time: float = None,
    #     accel_time: float = None,
    #     blocking: bool = True,
    # ) -> Tuple[Union[np.ndarray, Any, List[float]], bool]:
    #     """
    #     Command a desired end effector pose.

    #     :param T_sd: 4x4 Transformation Matrix representing the transform from the
    #         /<robot_name>/base_link frame to the /<robot_name>/ee_gripper_link frame
    #     :param custom_guess: (optional) list of joint positions with which to seed the IK solver
    #     :param execute: (optional) if `True`, this moves the physical robot after planning;
    #         otherwise, only planning is done
    #     :param moving_time: (optional) duration in seconds that the robot should move
    #     :param accel_time: (optional) duration in seconds that that robot should spend
    #         accelerating/decelerating (must be less than or equal to half the moving_time)
    #     :param blocking: (optional) whether the function should wait to return control to the user
    #         until the robot finishes moving
    #     :return: joint values needed to get the end effector to the desired pose
    #     :return: `True` if a valid solution was found; `False` otherwise
    #     """
    #     self.core.get_logger().debug(f'Setting ee_pose to matrix=\n{T_sd}')
    #     if custom_guess is None:
    #         initial_guesses = self.initial_guesses
    #     else:
    #         initial_guesses = [custom_guess]

    #     for guess in initial_guesses:
    #         theta_list, success = mr.IKinSpace(
    #             Slist=self.robot_des.Slist,
    #             M=self.robot_des.M,
    #             T=T_sd,
    #             thetalist0=guess,
    #             eomg=0.001,
    #             ev=0.001,
    #         )
    #         solution_found = True

    #         # Check to make sure a solution was found and that no joint limits were violated
    #         if success:
    #             theta_list = self._wrap_theta_list(theta_list)
    #             solution_found = self._check_joint_limits(theta_list)
    #         else:
    #             solution_found = False

    #         if solution_found:
    #             if execute:
    #                 self._publish_commands(
    #                     theta_list, moving_time, accel_time, blocking
    #                 )
    #                 self.T_sb = T_sd
    #             return theta_list, True

    #     self.core.get_logger().warn('No valid pose could be found. Will not execute')
    #     return theta_list, False

    # def set_ee_pose_components(
    #     self,
    #     x: float = 0,
    #     y: float = 0,
    #     z: float = 0,
    #     roll: float = 0,
    #     pitch: float = 0,
    #     yaw: float = None,
    #     custom_guess: List[float] = None,
    #     execute: bool = True,
    #     moving_time: float = None,
    #     accel_time: float = None,
    #     blocking: bool = True,
    # ) -> Tuple[Union[np.ndarray, Any, List[float]], bool]:
    #     """
    #     Command a desired end effector pose w.r.t. the Space frame.

    #     :param x: (optional) linear position along the X-axis of the Space frame [m]
    #     :param y: (optional) linear position along the Y-axis of the Space frame [m]
    #     :param z: (optional) linear position along the Z-axis of the Space frame [m]
    #     :param roll: (optional) angular position around the X-axis of the Space frame [rad]
    #     :param pitch: (optional) angular position around the Y-axis of the Space frame [rad]
    #     :param yaw: (optional) angular position around the Z-axis of the Space frame [rad]
    #     :param custom_guess: (optional) list of joint positions with which to seed the IK solver
    #     :param execute: (optional) if `True`, this moves the physical robot after planning;
    #         otherwise, only planning is done
    #     :param moving_time: (optional) duration in seconds that the robot should move
    #     :param accel_time: (optional) duration in seconds that that robot should spend
    #         accelerating/decelerating (must be less than or equal to half the moving_time)
    #     :param blocking: (optional) whether the function should wait to return control to the user
    #         until the robot finishes moving
    #     :return: joint values needed to get the end effector to the desired pose
    #     :return: True if a valid solution was found; False otherwise
    #     :details: Do not set 'yaw' if using an arm with fewer than 6dof
    #     """
    #     if self.group_info.num_joints < 6 or (
    #         self.group_info.num_joints >= 6 and yaw is None
    #     ):
    #         yaw = math.atan2(y, x)
    #     self.core.get_logger().debug(
    #         (
    #             f'Setting ee_pose components=\n'
    #             f'\t{x=}\n'
    #             f'\t{y=}\n'
    #             f'\t{z=}\n'
    #             f'\t{roll=}\n'
    #             f'\t{pitch=}\n'
    #             f'\t{yaw=}'
    #         )
    #     )
    #     T_sd = np.identity(4)
    #     T_sd[:3, :3] = ang.euler_angles_to_rotation_matrix([roll, pitch, yaw])
    #     T_sd[:3, 3] = [x, y, z]
    #     return self.set_ee_pose_matrix(
    #         T_sd, custom_guess, execute, moving_time, accel_time, blocking
    #     )

    # def set_ee_cartesian_trajectory(
    #     self,
    #     x: float = 0,
    #     y: float = 0,
    #     z: float = 0,
    #     roll: float = 0,
    #     pitch: float = 0,
    #     yaw: float = 0,
    #     moving_time: float = None,
    #     wp_moving_time: float = 0.2,
    #     wp_accel_time: float = 0.1,
    #     wp_period: float = 0.05,
    # ) -> bool:
    #     """
    #     Command a linear displacement to the end effector.

    #     :param x: (optional) linear displacement along the X-axis w.r.t. `T_sy` [m]
    #     :param y: (optional) linear displacement along the Y-axis w.r.t. `T_sy` [m]
    #     :param z: (optional) linear displacement along the Z-axis w.r.t. `T_sy` [m]
    #     :param roll: (optional) angular displacement around the X-axis w.r.t. `T_sy` [rad]
    #     :param pitch: (optional) angular displacement around the Y-axis w.r.t. `T_sy` [rad]
    #     :param yaw: (optional) angular displacement around the Z-axis w.r.t. `T_sy` [rad]
    #     :param moving_time: (optional) duration in seconds that the robot should move
    #     :param wp_moving_time: (optional) duration in seconds that each waypoint in the trajectory
    #         should move
    #     :param wp_accel_time: (optional) duration in seconds that each waypoint in the trajectory
    #         should be accelerating/decelerating (must be equal to or less than half of
    #         `wp_moving_time`)
    #     :param wp_period: (optional) duration in seconds between each waypoint
    #     :return: `True` if a trajectory was successfully planned and executed; otherwise `False`
    #     :details: `T_sy` is a 4x4 transformation matrix representing the pose of a virtual frame
    #         w.r.t. /<robot_name>/base_link. This virtual frame has the exact same `x`, `y`, `z`,
    #         `roll`, and `pitch` of /<robot_name>/base_link but contains the `yaw` of the end
    #         effector frame (/<robot_name>/ee_gripper_link).
    #     :details: Note that `y` and `yaw` must equal 0 if using arms with less than 6dof.
    #     """
    #     self.core.get_logger().debug(
    #         (
    #             f'Setting ee trajectory to components=\n'
    #             f'\tx={x}\n'
    #             f'\ty={y}\n'
    #             f'\tz={z}\n'
    #             f'\troll={roll}\n'
    #             f'\tpitch={pitch}\n'
    #             f'\tyaw={yaw}'
    #         )
    #     )
    #     if self.group_info.num_joints < 6 and (y != 0 or yaw != 0):
    #         self.core.get_logger().warn(
    #             (
    #                 "Please leave the 'y' and 'yaw' fields at '0' when working with arms that have"
    #                 ' fewer than 6dof.'
    #             )
    #         )
    #         return False
    #     rpy = ang.rotation_matrix_to_euler_angles(self.T_sb[:3, :3])
    #     T_sy = np.identity(4)
    #     T_sy[:3, :3] = ang.euler_angles_to_rotation_matrix([0.0, 0.0, rpy[2]])
    #     T_yb = np.dot(mr.TransInv(T_sy), self.T_sb)
    #     rpy[2] = 0.0
    #     if moving_time is None:
    #         moving_time = self.moving_time
    #     accel_time = self.accel_time
    #     N = int(moving_time / wp_period)
    #     inc = 1.0 / float(N)
    #     joint_traj = JointTrajectory()
    #     joint_positions = [float(cmd) for cmd in self.joint_commands]
    #     for i in range(N + 1):
    #         joint_traj_point = JointTrajectoryPoint()
    #         joint_traj_point.positions = tuple(joint_positions)
    #         joint_traj_point.time_from_start = Duration(
    #             nanosec=int(i * wp_period * S_TO_NS)
    #         )
    #         joint_traj.points.append(joint_traj_point)
    #         if i == N:
    #             break
    #         T_yb[:3, 3] += [inc * x, inc * y, inc * z]
    #         rpy[0] += inc * roll
    #         rpy[1] += inc * pitch
    #         rpy[2] += inc * yaw
    #         T_yb[:3, :3] = ang.euler_angles_to_rotation_matrix(rpy)
    #         T_sd = np.dot(T_sy, T_yb)
    #         theta_list, success = self.set_ee_pose_matrix(
    #             T_sd, joint_positions, False, blocking=False
    #         )
    #         if success:
    #             joint_positions = theta_list
    #         else:
    #             self.core.get_logger().warn(
    #                 (
    #                     f'{(i / float(N) * 100):.2f}% of trajectory successfully planned. '
    #                     'Trajectory will not be executed.'
    #                 )
    #             )
    #             break

    #     if success:
    #         self.set_trajectory_time(wp_moving_time, wp_accel_time)
    #         joint_traj.joint_names = self.group_info.joint_names
    #         current_positions = []
    #         with self.core.js_mutex:
    #             for name in joint_traj.joint_names:
    #                 current_positions.append(
    #                     self.core.joint_states.position[self.core.js_index_map[name]]
    #                 )
    #         joint_traj.points[0].positions = current_positions
    #         joint_traj.header.stamp = self.core.get_clock().now().to_msg()
    #         self.core.pub_traj.publish(
    #             JointTrajectoryCommand(
    #                 cmd_type='group', name=self.group_name, traj=joint_traj
    #             )
    #         )
    #         time.sleep(moving_time + wp_moving_time)
    #         self.T_sb = T_sd
    #         self.joint_commands = joint_positions
    #         self.set_trajectory_time(moving_time, accel_time)

    #     return success

    # def get_joint_commands(self) -> List[float]:
    #     """
    #     Get the latest commanded joint positions.

    #     :return: list of latest commanded joint positions [rad]
    #     """
    #     self.core.get_logger().debug('Getting latest joint commands')
    #     return list(self.joint_commands)

    # def get_single_joint_command(self, joint_name: str) -> float:
    #     """
    #     Get the latest commanded position for a given joint.

    #     :param joint_name: joint for which to get the position
    #     :return: desired position [rad]
    #     """
    #     self.core.get_logger().debug(f'Getting latest command for joint {joint_name}')
    #     return self.joint_commands[self.info_index_map[joint_name]]

    # def get_ee_pose_command(self) -> np.ndarray:
    #     """
    #     Get the latest commanded end effector pose w.r.t the Space frame.

    #     :return <4x4 matrix> - Transformation matrix
    #     """
    #     self.core.get_logger().debug('Getting latest ee pose command')
    #     return np.array(self.T_sb)

    # def get_ee_pose(self) -> np.ndarray:
    #     """
    #     Get the actual end effector pose w.r.t the Space frame.

    #     :return: Transformation matrix
    #     """
    #     self.core.get_logger().debug('Getting actual end effector pose')
    #     joint_states = [
    #         self.core.joint_states.position[self.core.js_index_map[name]]
    #         for name in self.group_info.joint_names
    #     ]
    #     return mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, joint_states)

    # def capture_joint_positions(self) -> None:
    #     """
    #     Reset self.joint_commands to be the actual positions seen by the encoders.

    #     :details: should be used whenever joints are torqued off, right after torquing them on
    #         again
    #     """
    #     self.core.get_logger().debug('Capturing joint positions')
    #     self.joint_commands = []
    #     for name in self.group_info.joint_names:
    #         self.joint_commands.append(
    #             self.core.joint_states.position[self.core.js_index_map[name]]
    #         )
    #     self._update_Tsb()

    # def _update_Tsb(self) -> None:
    #     """Update transform between the space and body frame from the current joint commands."""
    #     self.core.get_logger().debug('Updating T_sb')
    #     self.T_sb = mr.FKinSpace(
    #         self.robot_des.M, self.robot_des.Slist, self.joint_commands
    #     )

    # def _wrap_theta_list(self, theta_list: List[np.ndarray]) -> List[np.ndarray]:
    #     """
    #     Wrap an array of joint commands to [-pi, pi) and between the joint limits.

    #     :param theta_list: array of floats to wrap
    #     :return: array of floats wrapped between [-pi, pi)
    #     """
    #     theta_list = (theta_list + np.pi) % REV - np.pi
    #     for x in range(len(theta_list)):
    #         if round(theta_list[x], 3) < round(self.group_info.joint_lower_limits[x], 3):
    #             theta_list[x] += REV
    #         elif round(theta_list[x], 3) > round(self.group_info.joint_upper_limits[x], 3):
    #             theta_list[x] -= REV
    #     return theta_list
