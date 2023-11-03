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

import time

from geometry_msgs.msg import Pose

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy)

import numpy as np

# generic ros libraries
import rclpy
from rclpy.logging import get_logger
from rclpy.logging import LoggingSeverity


class InterbotixManipulatorXS:
    """Standalone Module to control an Interbotix Arm and Gripper."""

    def __init__(
        self,
        robot_model: str,
        group_name: str = 'interbotix_arm',
        gripper_name: str = 'interbotix_gripper',
        moving_time: float = 2.0,
        accel_time: float = 0.3,
        gripper_pressure: float = 0.5,
        gripper_pressure_lower_limit: int = 150,
        gripper_pressure_upper_limit: int = 350,
        topic_joint_states: str = 'joint_states',
        logging_level: LoggingSeverity = LoggingSeverity.INFO,
        moveit_node_name: str = 'moveit_py',
        args=None,
    ) -> None:
        """
        Construct the Standalone Interbotix Manipulator module.

        :param robot_model: Interbotix Arm/Cobot model (ex. 'wx200' or 'dx400')
        :param group_name: (optional) joint group name that contains the 'arm' joints as defined in
            the 'motor_config' yaml file; typically, this is 'interbotix_arm'
        :param gripper_name: (optional) name of the gripper joint as defined in the 'motor_config'
            yaml file; typically, this is 'interbotix_gripper'
        :param moving_time: (optional) time [s] it should take for all joints in the arm to
            complete one move
        :param accel_time: (optional) time [s] it should take for all joints in the arm to
            accelerate/decelerate to/from max speed
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
        :param moveit_node_name: (optional) name to give to the core started by this class,
        defaults to 'moveit_py'
        """
        rclpy.init()

        self.arm = InterbotixArmXSInterface(
            robot_model=robot_model,
            group_name=group_name,
            moveit_node_name=moveit_node_name,
            moving_time=moving_time,
            accel_time=accel_time,

        )
        if gripper_name is not None:
            self.gripper = InterbotixGripperXSInterface(
                robot_model=robot_model,
                gripper_name=gripper_name,
                moveit_node_name=moveit_node_name,
                gripper_pressure=gripper_pressure,
                gripper_pressure_lower_limit=gripper_pressure_lower_limit,
                gripper_pressure_upper_limit=gripper_pressure_upper_limit,
            )

    def shutdown(self) -> None:
        """Destroy the node and shut down all threads and processes."""
        rclpy.shutdown()


class InterbotixArmXSInterface:
    """Definition of the Interbotix Arm component."""

    def __init__(
        self,
        robot_model: str,
        group_name: str,
        moveit_node_name: str = 'moveit_py',
        moving_time: float = 2.0,
        accel_time: float = 0.3,
    ) -> None:
        """
        Construct the InterbotixArmXSInterface object.

        :param core: reference to the InterbotixRobotXSCore class containing the internal ROS
            plumbing that drives the Python API
        :param robot_model: Interbotix Arm model (ex. 'wx200' or 'dx400')
        :param group_name: joint group name that contains the 'interbotix_arm' joints as
        defined in the 'motor_config' yaml file; typically, this is 'arm'
        :param moving_time: (optional) time [s] it should take for all joints in the arm to
            complete one move
        :param accel_time: (optional) time [s] it should take for all joints in the arm to
            accelerate/decelerate to/from max speed
        """
        self.logger = get_logger('moveit_py.pose_goal')

        # instantiate moveit_py instance and a planning component for the panda_arm
        self.robot = MoveItPy(node_name=moveit_node_name)
        self.group_name = group_name
        self.planning_component = self.robot.get_planning_component(self.group_name)
        self.logger.info('MoveItPy instance created')

    def plan_and_execute(
        self,
        single_plan_parameters=None,
        multi_plan_parameters=None,
        sleep_time=0.0,
    ):
        """Plan and execute a motion."""
        # plan to goal
        self.logger.info('Planning trajectory')
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
            self.logger.info('Executing plan')
            robot_trajectory = plan_result.trajectory
            self.robot.execute(robot_trajectory, controllers=[])
        else:
            self.logger.error('Planning failed')

        time.sleep(sleep_time)

    def go_to_home_pose(self):
        """Move robot to home configuartion."""
        self.planning_component.set_start_state_to_current_state()

        # set pose goal using predefined state
        self.planning_component.set_goal_state(configuration_name='home')

        # plan to goal
        self.plan_and_execute(sleep_time=3.0)
        self.logger.info('Reached Home Pose')

    def go_to_sleep_pose(self):
        """Move robot to home configuartion."""
        self.planning_component.set_start_state_to_current_state()

        # set pose goal using predefined state
        self.planning_component.set_goal_state(configuration_name='sleep')

        # plan to goal
        self.plan_and_execute(sleep_time=3.0)
        self.logger.info('Reached Sleep Pose')

    def go_to_ee_pose(self,
                      pose_goal: Pose
                      ):
        """Move robot to the EE pose."""
        planning_scene_monitor = self.robot.get_planning_scene_monitor()
        with planning_scene_monitor.read_write() as scene:

            # instantiate a RobotState instance using the current planning scene of robot
            robot_state = scene.current_state
            robot_state.get_joint_group_positions(self.group_name)
            robot_state.update()
            # find the goal RobotState for given ee pose
            result = robot_state.set_from_ik(joint_model_group_name=self.group_name,
                                             geometry_pose=pose_goal,
                                             tip_name='dx400/ee_gripper_link',
                                             timeout=5.0)

            # Check the result of the IK solver
            if not result:
                self.logger.error('IK solution was not found!')
                return
            else:
                self.logger.info('IK solution found!')

            robot_state.update()

            # set plan start state to current state
            self.planning_component.set_start_state_to_current_state()

            # set goal state to the initialized robot state
            self.planning_component.set_goal_state(robot_state=robot_state)
            robot_state.update()

        self.plan_and_execute(sleep_time=3.0)
        self.logger.info('Reached EE Pose')

    def go_to_joint_positions(self,
                              joint_group_positions: np.ndarray):
        """Move robot to the input joint positions."""
        robot_model = self.robot.get_robot_model()
        robot_state = RobotState(robot_model)
        robot_state.update()
        robot_state.set_joint_group_positions(
            joint_model_group_name=self.group_name,
            position_values=joint_group_positions
        )
        robot_state.update()

        # set plan start state to current state
        self.planning_component.set_start_state_to_current_state()

        # set goal state to the initialized robot state
        self.planning_component.set_goal_state(robot_state=robot_state)
        self.plan_and_execute(sleep_time=3.0)
        self.logger.info('Reached joint Positions')

    def shutdown(self) -> None:
        """Destroy the node and shut down all threads and processes."""
        rclpy.shutdown()


class InterbotixGripperXSInterface:
    """Definition of the Interbotix Gripper component."""

    def __init__(
        self,
        robot_model: str,
        gripper_name: str,
        moveit_node_name: str = 'moveit_py',
        gripper_pressure: float = 0.5,
        gripper_pressure_lower_limit: int = 150,
        gripper_pressure_upper_limit: int = 350,
    ) -> None:
        """
        Construct the Interbotix Gripper Module.

        :param core: reference to the InterbotixRobotXSCore class containing the internal ROS
            plumbing that drives the Python API
        :param gripper_name: name of the gripper joint as defined in the 'motor_config' yaml file;
            typically, this is 'interbotix_gripper'
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
        self.logger = get_logger('moveit_py.gripper')

        # instantiate moveit_py instance and a planning component for the panda_arm
        self.robot = MoveItPy(node_name=moveit_node_name)
        self.planning_component = self.robot.get_planning_component(gripper_name)
        self.logger.info('MoveItPy instance created')

    def plan_and_execute(
        self,
        single_plan_parameters=None,
        multi_plan_parameters=None,
        sleep_time=0.0,
    ):
        """Plan and execute a motion."""
        # plan to goal
        self.logger.info('Planning trajectory')
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
            self.logger.info('Executing plan')
            robot_trajectory = plan_result.trajectory
            self.robot.execute(robot_trajectory, controllers=[])
        else:
            self.logger.error('Planning failed')

        time.sleep(sleep_time)

    def gripper_open(self):
        """Move robot to home configuartion."""
        self.planning_component.set_start_state_to_current_state()

        # set pose goal using predefined state
        self.planning_component.set_goal_state(configuration_name='open')

        # plan to goal
        self.plan_and_execute(sleep_time=3.0)

    def gripper_close(self):
        """Move robot to home configuartion."""
        self.planning_component.set_start_state_to_current_state()

        # set pose goal using predefined state
        self.planning_component.set_goal_state(configuration_name='close')

        # plan to goal
        self.plan_and_execute(sleep_time=3.0)

    def shutdown(self) -> None:
        """Destroy the node and shut down all threads and processes."""
        rclpy.shutdown()
