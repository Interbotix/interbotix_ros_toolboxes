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

"""Contains a class used to manage a generic robot's ROS node state."""


import rclpy
from rclpy.node import Node
from rclpy.task import Future


class InterbotixRobotNode(Node):
    def __init__(
        self,
        robot_name=None,
        node_name='robot_manipulation',
    ) -> None:
        self.node_name = node_name
        self.robot_name = robot_name
        if self.robot_name == '':
            self.ns = ''
        else:
            self.ns = f'/{self.robot_name}'
        if robot_name is not None:
            node_name += '_robot_manipulation'
        if not rclpy.ok():
            rclpy.init()

        super().__init__(node_name=self.node_name, namespace=self.robot_name)
        self.get_logger().info("Initialized InterbotixRobotNode!")

    def robot_spin_once_until_future_complete(
        self, future: Future,
        timeout_sec: float = 0.1
    ) -> None:
        """
        Spin the core's executor once until the given future is complete within the timeout.

        :param future: future to complete
        :timeout_sec: seconds to wait. defaults to 0.1 seconds
        """
        if self.executor is not None:
            self.executor.spin_once_until_future_complete(future=future, timeout_sec=timeout_sec)
        else:
            raise NotImplementedError()

    def robot_spin_until_future_complete(
        self, future: Future,
        timeout_sec: float = None
    ) -> None:
        """
        Spin the core's executor until the given future is complete within the timeout.

        :param future: future to complete
        :timeout_sec: seconds to wait. defaults to None (no timeout)
        """
        if self.executor is not None:
            self.executor.spin_until_future_complete(future=future, timeout_sec=timeout_sec)
        else:
            rclpy.spin_until_future_complete(self, future=future, timeout_sec=timeout_sec)
