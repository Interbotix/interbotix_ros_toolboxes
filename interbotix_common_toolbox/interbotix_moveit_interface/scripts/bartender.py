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

import os
import sys
import yaml
import rclpy
import numpy as np
import quaternion
# message libraries
from geometry_msgs.msg import PoseStamped, Pose

# moveit_py
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState

from rclpy.logging import get_logger

# config file libraries
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

from interbotix_moveit_interface import moveit_interface_core
from interbotix_moveit_interface import move_arm


import numpy as np

"""
This script makes the end-effector perform pick, pour, and place tasks.
Note that this script may not work for every arm as it was designed for the wx250.
Make sure to adjust commanded joint positions and poses as necessary.

To get started, open a terminal and type:

    ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=wx250

Then change to this directory and type:

    python3 bartender.py
"""


def main():
    bot = move_arm.InterbotixManipulatorXS(
        robot_model='dx400',
        group_name='interbotix_arm',
        gripper_name='interbotix_gripper'
    )

    pose_goal1 = Pose()
    q1 = np.quaternion(0.7048, 0.0424, -0.0423, 0.7068).normalized()
    pose_goal1.position.x = -0.022
    pose_goal1.position.y = 0.221
    pose_goal1.position.z = 0.061
    pose_goal1.orientation.x = q1.x
    pose_goal1.orientation.y = q1.y
    pose_goal1.orientation.z = q1.z
    pose_goal1.orientation.w = q1.w


    pose_goal2 = Pose()
    q1 = np.quaternion(0.886, 0.027, 0.056, 0.459).normalized()
    pose_goal2.position.x = 0.204
    pose_goal2.position.y = 0.199
    pose_goal2.position.z = 0.214
    pose_goal2.orientation.x = q1.x
    pose_goal2.orientation.y = q1.y
    pose_goal2.orientation.z = q1.z
    pose_goal2.orientation.w = q1.w
    # bot.arm.go_to_home_pose()
    bot.arm.go_to_ee_pose(pose_goal1)
    bot.arm.go_to_ee_pose(pose_goal2)
    bot.arm.go_to_sleep_pose()
    bot.arm.go_to_joint_positions()
    bot.arm.go_to_sleep_pose()



if __name__ == '__main__':
    main()