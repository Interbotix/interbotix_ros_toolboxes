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

"""
Contains the `InterbotixGravityCompensationInterface` class.

It enables/disables the gravity compensation feature of an Interbotix arm using Python.
"""

from interbotix_common_modules.common_robot.robot import InterbotixRobotNode
from interbotix_xs_modules.xs_robot.core import InterbotixRobotXSCore
from rclpy.logging import LoggingSeverity
from std_srvs.srv import SetBool


class InterbotixGravityCompensation:
    """Standalone Module to run gravity compensation on an Interbotix Arm."""

    def __init__(
        self,
        robot_model: str,
        robot_name: str = None,
        topic_joint_states: str = 'joint_states',
        logging_level: LoggingSeverity = LoggingSeverity.INFO,
        node_name: str = 'interbotix_robot_manipulation',
        node: InterbotixRobotNode = None,
        args=None,
    ) -> None:
        """
        Construct the Interbotix Gravity Compensation Module.

        :param robot_model: Interbotix Arm model (ex. 'wx250' or 'aloha_wx250s')
        :param robot_name: (optional) defaults to value given to 'robot_model'; this can be
            customized if controlling two of the same arms from one computer (like 'arm1/wx250s'
            and 'arm2/wx250s')
        :param topic_joint_states: (optional) the specific JointState topic output by the xs_sdk
            node
        :param logging_level: (optional) rclpy logging severity level. Can be DEBUG, INFO, WARN,
            ERROR, or FATAL. defaults to INFO
        :param node_name: (optional) name to give to the node started by this class, defaults to
            'interbotix_robot_manipulation'
        :param node: (optional) the InterbotixRobotNode to base this class's ROS components on.
        """
        self.core = InterbotixRobotXSCore(
            robot_model=robot_model,
            robot_name=robot_name,
            topic_joint_states=topic_joint_states,
            logging_level=logging_level,
            node_name=node_name,
            node=node,
            args=args,
        )
        self.gravity_compensation = InterbotixGravityCompensationInterface(self.core)


class InterbotixGravityCompensationInterface:
    def __init__(self, core: InterbotixRobotXSCore) -> None:
        """
        Definition of the gravity compensation component.

        :param core: reference to the InterbotixRobotXSCore class containing the internal ROS
            plumbing that drives the Python API
        """
        self.core = core

        # Create the gravity compensation enable client
        self.client = self.core.robot_node.create_client(
            srv_type=SetBool,
            srv_name=f'{self.core.ns}/gravity_compensation_enable',
        )

        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.core.robot_node.get_logger().info(
                f'Service {self.client.srv_name} not available, waiting again...'
            )

        # Initialize the gravity compensation enable request
        self.request = SetBool.Request()

    def enable(self) -> None:
        """Enable gravity compensation on the robot."""
        self.request.data = True
        self.core.get_node().wait_until_future_complete(
            self.client.call_async(self.request))

    def disable(self) -> None:
        """Disable gravity compensation on the robot."""
        self.request.data = False
        self.core.get_node().wait_until_future_complete(
            self.client.call_async(self.request))
