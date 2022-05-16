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

"""
Contains the `InterbotixGripperXS` and `InterbotixGripperXSInterface` classes.

These two classes can be used to control an X-Series standalone gripper using Python.
"""

from threading import Thread
import time

from interbotix_xs_modules.xs_robot.core import InterbotixRobotXSCore
from interbotix_xs_msgs.msg import JointSingleCommand
from interbotix_xs_msgs.srv import RobotInfo
import rclpy
from rclpy.logging import LoggingSeverity


class InterbotixGripperXS:
    """Standalone Module to control an Interbotix Gripper using PWM or Current control."""

    def __init__(
        self,
        robot_model: str,
        gripper_name: str,
        robot_name: str = None,
        gripper_pressure: float = 0.5,
        gripper_pressure_lower_limit: int = 150,
        gripper_pressure_upper_limit: int = 350,
        joint_state_topic: str = 'joint_states',
        logging_level: LoggingSeverity = LoggingSeverity.INFO,
        node_name: str = 'robot_manipulation',
    ) -> None:
        """
        Construct the Standalone Interbotix Gripper Module.

        :param robot_model: Interbotix Arm model (ex. 'wx200' or 'vx300s')
        :param gripper_name: name of the gripper joint as defined in the 'motor_config'
            yaml file; typically, this is 'gripper'
        :param robot_name: defaults to value given to 'robot_model'; this can be
            customized to best suit the user's needs
        :param gripper_pressure: fraction from 0 - 1 where '0' means the gripper
            operates at 'gripper_pressure_lower_limit' and '1' means the gripper
            operates at 'gripper_pressure_upper_limit'
        :param gripper_pressure_lower_limit: lowest 'effort' that should be applied to
            the gripper if gripper_pressure is set to 0; it should be high enough to
            open/close the gripper (~150 PWM or ~400 mA current)
        :param gripper_pressure_upper_limit: largest 'effort' that should be applied to
            the gripper if gripper_pressure is set to 1; it should be low enough that
            the motor doesn't 'overload' when gripping an object for a few seconds (~350
            PWM or ~900 mA)
        :param joint_state_topic: (optional) the specifc JointState topic output by the
            xs_sdk node
        :logging_level: (optional) rclpy logging severtity level. Can be DEBUG, INFO,
            WARN, ERROR, or FATAL. defaults to INFO
        :node_name: (optional) name to give to the core started by this class, defaults
            to 'robot_manipulation'
        :details: note that this module doesn't really have any use case except in
            controlling just the gripper joint on an Interbotix Arm.
        """
        self.core = InterbotixRobotXSCore(
            robot_model,
            robot_name,
            joint_state_topic=joint_state_topic,
            logging_level=logging_level,
            node_name=node_name,
        )
        self.gripper = InterbotixGripperXSInterface(
            self.core,
            gripper_name,
            gripper_pressure,
            gripper_pressure_lower_limit,
            gripper_pressure_upper_limit,
        )

        rclpy.spin_once(self.core)
        self.core.initialize()
        rclpy.spin_once(self.core)
        self.gripper.initialize()
        # TODO: is this the best way to do this?
        self._execution_thread = Thread(target=rclpy.spin, args=(self.core,))
        self._execution_thread.start()

    def shutdown(self):
        """Destroy the node and shuts down all threads and processes."""
        self.core.destroy_node()
        rclpy.shutdown()
        time.sleep(0.5)
        self._execution_thread.join(timeout=5.0)
        if self._execution_thread.is_alive():
            print('Taking a long time to destroy. Press Ctrl+C twice to exit.')


class InterbotixGripperXSInterface:
    def __init__(
        self,
        core: InterbotixRobotXSCore,
        gripper_name: str,
        gripper_pressure: float = 0.5,
        gripper_pressure_lower_limit: int = 150,
        gripper_pressure_upper_limit: int = 350,
    ) -> None:
        """
        Construct the Interbotix Gripper Module.

        :param core: reference to the InterbotixRobotXSCore class containing the
            internal ROS plumbing that drives the Python API
        :param gripper_name: name of the gripper joint as defined in the 'motor_config'
            yaml file; typically, this is 'gripper'
        :param gripper_pressure: fraction from 0 - 1 where '0' means the gripper
            operates at 'gripper_pressure_lower_limit' and '1' means the gripper
            operates at 'gripper_pressure_upper_limit'
        :param gripper_pressure_lower_limit: lowest 'effort' that should be applied to
            the gripper if gripper_pressure is set to 0; it should be high enough to
            open/close the gripper (~150 PWM or ~400 mA current)
        :param gripper_pressure_upper_limit: largest 'effort' that should be applied to
            the gripper if gripper_pressure is set to 1; it should be low enough that
            the motor doesn't 'overload' when gripping an object for a few seconds (~350
            PWM or ~900 mA)
        """
        self.core = core
        self.gripper_name = gripper_name
        self.gripper_pressure = gripper_pressure
        self.future_gripper_info = self.core.srv_get_info.call_async(
            RobotInfo.Request(cmd_type='single', name=gripper_name)
        )
        self.gripper_moving: bool = False
        self.gripper_command = JointSingleCommand(name='gripper')
        self.gripper_pressure_lower_limit = gripper_pressure_lower_limit
        self.gripper_pressure_upper_limit = gripper_pressure_upper_limit

        # value = lower + pressure * range
        self.gripper_value = gripper_pressure_lower_limit + (
            gripper_pressure
            * (gripper_pressure_upper_limit - gripper_pressure_lower_limit)
        )

        self.tmr_gripper_state = self.core.create_timer(
            timer_period_sec=0.02, callback=self.gripper_state
        )

    def initialize(self) -> None:
        """Initialize the InterbotixGripperXSInterface object."""
        while rclpy.ok() and not self.future_gripper_info.done():
            rclpy.spin_until_future_complete(self.core, self.future_gripper_info)
            rclpy.spin_once(self.core)

        self.gripper_info: RobotInfo.Response = self.future_gripper_info.result()
        self.left_finger_index = self.core.js_index_map[
            self.gripper_info.joint_names[0]
        ]
        self.left_finger_lower_limit = self.gripper_info.joint_lower_limits[0]
        self.left_finger_upper_limit = self.gripper_info.joint_upper_limits[0]

        if self.gripper_info.mode != 'current' and self.gripper_info.mode != 'pwm':
            self.core.get_logger().err(
                "Please set the gripper's 'operating mode' to 'pwm' or 'current'."
            )

        time.sleep(0.5)
        print(
            (
                f'Gripper Name: {self.gripper_name}\n'
                f'Gripper Pressure: {self.gripper_pressure*100}%'
            )
        )
        print('Initialized InterbotixGripperXSInterface!\n')

    def gripper_state(self) -> None:
        """Stop the gripper moving past its limits when in PWM mode using a ROS Timer Callback."""
        if self.gripper_moving:
            # update gripper position
            with self.core.js_mutex:
                gripper_pos = self.core.joint_states.position[self.left_finger_index]
            # stop the gripper if it has reached the lower or upper limit
            if (
                self.gripper_command.cmd > 0
                and gripper_pos >= self.left_finger_upper_limit
            ) or (
                self.gripper_command.cmd < 0
                and gripper_pos <= self.left_finger_lower_limit
            ):
                self.gripper_command.cmd = float(0.0)
                self.core.pub_single.publish(self.gripper_command)
                self.gripper_moving = False

    def gripper_controller(self, effort: float, delay: float) -> None:
        """
        Publish effort commands to the gripper (when in 'pwm' or 'current' mode).

        :param effort: effort command to send to the gripper motor
        :param delay: number of seconds to wait before returning control to the user
        """
        self.gripper_command.cmd = effort
        # update gripper position
        with self.core.js_mutex:
            gripper_pos = self.core.joint_states.position[self.left_finger_index]
        # check if the gripper is within its limits
        if (
            self.gripper_command.cmd > 0 and gripper_pos < self.left_finger_upper_limit
        ) or (
            self.gripper_command.cmd < 0 and gripper_pos > self.left_finger_lower_limit
        ):
            self.core.pub_single.publish(self.gripper_command)
            self.gripper_moving = True
            time.sleep(delay)

    def set_pressure(self, pressure: float) -> None:
        """
        Set the amount of pressure that the gripper should use when grasping an object.

        :param pressure: a scaling factor from 0 to 1 where the pressure increases as
            the factor increases
        """
        self.gripper_value = self.gripper_pressure_lower_limit + pressure * (
            self.gripper_pressure_upper_limit - self.gripper_pressure_lower_limit
        )

    def grasp(self, delay: float = 1.0) -> None:
        """
        Open the gripper (when in 'pwm' control mode).

        :param delay: number of seconds to delay before returning control to the user
        """
        self.gripper_controller(self.gripper_value, delay)

    def release(self, delay: float = 1.0) -> None:
        """
        Close the gripper (when in 'pwm' control mode).

        :param delay: number of seconds to delay before returning control to the user
        """
        self.gripper_controller(-self.gripper_value, delay)