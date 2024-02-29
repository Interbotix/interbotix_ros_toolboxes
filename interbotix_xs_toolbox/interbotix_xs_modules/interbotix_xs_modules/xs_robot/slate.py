# Copyright 2024 Trossen Robotics
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

"""Contains classes used to control the Interbotix Slate mobile base."""

from threading import Thread
from typing import Text

from interbotix_slate_msgs.srv import SetString
from interbotix_common_modules.common_robot import InterbotixRobotNode
from interbotix_xs_modules.xs_robot.mobile_base import InterbotixMobileBaseInterface
import rclpy
from rclpy.constants import S_TO_NS
from rclpy.duration import Duration
from std_srvs.srv import SetBool
from rclpy.executors import MultiThreadedExecutor
import time


class InterbotixSlate:
    def __init__(
        self,
        robot_name: Text = None,
        start_on_init: bool = True,
        node_owner: bool = True,
        node: InterbotixRobotNode = None,
    ) -> None:
        self.node_owner = node_owner
        if not self.node_owner and node is not None:
            self.core = node
        elif self.node_owner and node is None:
            self.core = InterbotixRobotNode(
                robot_name=robot_name,
            )
        else:
            raise
        self.base = InterbotixSlateInterface(
            core=self.core,
            robot_name=robot_name,
        )

        if start_on_init and self.node_owner:
            self.start()

    def start(self) -> None:
        """Start a background thread that builds and spins an executor."""
        self._execution_thread = Thread(target=self.run)
        self._execution_thread.start()

    def run(self) -> None:
        """Thread target."""
        self.ex = MultiThreadedExecutor()
        self.ex.add_node(self.core)
        self.ex.spin()

    def shutdown(self) -> None:
        """Destroy the node and shut down all threads and processes."""
        if self.node_owner:
            self.core.destroy_node()
            rclpy.try_shutdown()
            self._execution_thread.join()
            time.sleep(0.5)
        else:
            self.core.get_logger().error(
                'Cannot perform shutdown due to lack of ownership'
            )


class InterbotixSlateInterface(InterbotixMobileBaseInterface):
    """Definition of the Interbotix Slate Module."""

    def __init__(
        self,
        core: InterbotixRobotNode,
        robot_name: str,
        topic_base_joint_states: str = 'mobile_base/joint_states',
        topic_cmd_vel: str = 'mobile_base/cmd_vel',
        nav_timeout_sec: float = 300.0,
        use_nav: bool = False,
    ):
        """
        Construct the InterbotixSlateInterface object.

        :param core: reference to the InterbotixRobotNode class containing the internal ROS
            plumbing that drives the Python API
        :param robot_name: namespace of the Slate nodes
        :param topic_base_joint_states: (optional) name of the joints states topic that contains
            the states of the Slate's two wheels. defaults to `'mobile_base/joint_states'`
        :param topic_cmd_vel: (optional) name of the twist topic to which velocity commands should
            be published. defaults to `'mobile_base/cmd_vel'`
        :param nav_timeout_sec: (optional) length of time in seconds after which to cancel a
            navigation goal. defaults to `300` (five minutes)
        :param use_nav: (optional) whether or not to enable navigation features. requires that nav2
            be launched. defaults to `False`
        """
        super().__init__(
            core=core,
            robot_name=robot_name,
            topic_base_joint_states=topic_base_joint_states,
            topic_cmd_vel=topic_cmd_vel,
            nav_timeout_sec=nav_timeout_sec,
            use_nav=use_nav,
        )

        self.client_set_text = self.core.create_client(
            srv_type=SetString,
            srv_name='mobile_base/set_text',
        )
        self.client_set_motor_torque = self.core.create_client(
            srv_type=SetBool,
            srv_name='mobile_base/set_motor_torque_status',
        )

        while not self.client_set_motor_torque.wait_for_service(timeout_sec=5.0) and rclpy.ok():
            self.core.get_logger().error(
                (
                    "Failed to find services under namespace 'mobile_base'. Is the slate "
                    'base driver node running under that namespace?'
                )
            )

        self.core.get_clock().sleep_for(Duration(nanoseconds=int(0.5*S_TO_NS)))
        self.core.get_logger().info('Initialized InterbotixSlateInterface!')

    def set_text(self, text: str) -> bool:
        """
        Set the text on the Slate base's screen.

        :param text: Text to set
        :return: `True` if the set_text service succeeded, `False` otherwise
        """
        future_set_text = self.client_set_text.call_async(SetString.Request(data=text))
        self.core.robot_spin_until_future_complete(future_set_text)
        result: SetString.Response = future_set_text.result()
        self.core.get_logger().info(result.message)
        return result.success

    def set_motor_torque(self, enable: bool) -> bool:
        """
        Set the Slate base's motor torque status.

        :param enable: `True` to enable torque on the base's motors, `False` to disable
        :return: `True` if the set_motor_torque service succeeded, `False` otherwise
        """
        future_set_motor_torque = self.client_set_motor_torque.call_async(
            SetBool.Request(data=enable)
        )
        self.core.robot_spin_until_future_complete(future_set_motor_torque)
        result: SetBool.Response = future_set_motor_torque.result()
        self.core.get_logger().info(result.message)
        return result.success

    def play_sound(self, *args, **kwargs) -> None:
        raise NotImplementedError()

    def reset_odom(self, *args, **kwargs) -> None:
        raise NotImplementedError()
