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

import time
from typing import List

from interbotix_xs_modules.xs_robot.core import InterbotixRobotXSCore
from interbotix_xs_msgs.msg import JointGroupCommand, JointSingleCommand
from interbotix_xs_msgs.srv import RobotInfo

raise NotImplementedError('The turret module is not yet compatible with ROS2')


class InterbotixTurretXS:
    """Defines the standalone module to control an Interbotix Turret."""

    def __init__(
        self,
        robot_model: str,
        robot_name: str = None,
        group_name: str = 'turret',
        pan_profile_type: str = 'time',
        pan_profile_velocity: float = 2.0,
        pan_profile_acceleration: float = 0.3,
        tilt_profile_type: str = 'time',
        tilt_profile_velocity: float = 2.0,
        tilt_profile_acceleration: float = 0.3,
    ) -> None:
        """
        Construct the Standalone InterbotixTurretXS object.

        :param robot_model: Interbotix turret model (ex. 'wxxmt' or 'pxxls')
        :param robot_name: defaults to value given to 'robot_model'; this can be customized if
            controlling two of the same turrets from one computer (like 'turret1/wxxmt' and
            'turret2/pxxls')
        :param group_name: (optional) name of the desired Turret's pan-tilt joint group
        :param pan_profile_type: (optional) 'pan' joint settting; refer to the OperatingModes
            Service file for an explanation - can be either 'time' or 'velocity'
        :param pan_profile_velocity: (optional) 'pan' joint setting; refer to the OperatingModes
            Service file for an explanation - note that when 'profile_type' is 'time', units are in
            seconds, not milliseconds
        :param pan_profile_acceleration: (optional) 'pan' joint setting; refer to the
            OperatingModes Service file for an explanation - note that when 'profile_type' is
            'time', units are in seconds, not milliseconds
        :param tilt_profile_type: (optional) 'tilt' joint settting; refer to the OperatingModes
            Service file for an explanation - can be either 'time' or 'velocity'
        :param tilt_profile_velocity: (optional) 'tilt' joint setting; refer to the OperatingModes
            Service file for an explanation - note that when 'profile_type' is 'time', units are in
            seconds, not milliseconds
        :param tilt_profile_acceleration: (optional) 'tilt' joint setting; refer to the
            OperatingModes Service file for an explanation - note that when 'profile_type' is
            'time', units are in seconds, not milliseconds
        """
        self.core = InterbotixRobotXSCore(
            robot_model=robot_model,
            robot_name=robot_name
        )
        self.turret = InterbotixTurretXSInterface(
            core=self.core,
            group_name=group_name,
            pan_profile_type=pan_profile_type,
            pan_profile_velocity=pan_profile_velocity,
            pan_profile_acceleration=pan_profile_acceleration,
            tilt_profile_type=tilt_profile_type,
            tilt_profile_velocity=tilt_profile_velocity,
            tilt_profile_acceleration=tilt_profile_acceleration
        )


class InterbotixTurretXSInterface:
    """Definition of the Interbotix Turret Module."""

    def __init__(
        self,
        core: InterbotixRobotXSCore,
        group_name: str = 'turret',
        pan_profile_type: str = 'time',
        pan_profile_velocity: float = 2.0,
        pan_profile_acceleration: float = 0.3,
        tilt_profile_type: str = 'time',
        tilt_profile_velocity: float = 2.0,
        tilt_profile_acceleration: float = 0.3
    ):
        """
        Construct the InterbotixTurretXSInterface object.

        :param core: reference to the InterbotixRobotXSCore class containing the internal ROS
            plumbing that drives the Python API
        :param group_name: (optional) name of the desired Turret's pan-tilt joint group
        :param pan_profile_type: (optional) 'pan' joint settting; refer to the OperatingModes
            Service file for an explanation - can be either 'time' or 'velocity'
        :param pan_profile_velocity: (optional) 'pan' joint setting; refer to the OperatingModes
            Service file for an explanation - note that when 'profile_type' is 'time', units are in
            seconds, not milliseconds
        :param pan_profile_acceleration: (optional) 'pan' joint setting; refer to the
            OperatingModes Service file for an explanation - note that when 'profile_type' is
            'time', units are in seconds, not milliseconds
        :param tilt_profile_type: (optional) 'tilt' joint settting; refer to the OperatingModes
            Service file for an explanation - can be either 'time' or 'velocity'
        :param tilt_profile_velocity: (optional) 'tilt' joint setting; refer to the OperatingModes
            Service file for an explanation - note that when 'profile_type' is 'time', units are in
            seconds, not milliseconds
        :param tilt_profile_acceleration: (optional) 'tilt' joint setting; refer to the
            OperatingModes Service file for an explanation - note that when 'profile_type' is
            'time', units are in seconds, not milliseconds
        """
        self.core = core
        group_info: RobotInfo.Response = self.core.srv_get_info('group', group_name)
        self.group_name = group_name
        self.pan_name = group_info.joint_names[0]
        self.tilt_name = group_info.joint_names[1]
        pan_limits = [group_info.joint_lower_limits[0], group_info.joint_upper_limits[0]]
        tilt_limits = [group_info.joint_lower_limits[1], group_info.joint_upper_limits[1]]
        pan_position = self.core.joint_states.position[self.core.js_index_map[self.pan_name]]
        tilt_position = self.core.joint_states.position[self.core.js_index_map[self.tilt_name]]
        self.info = {
            self.pan_name: {
                'command': pan_position,
                'profile_type': pan_profile_type,
                'profile_velocity': pan_profile_velocity,
                'profile_acceleration': pan_profile_acceleration,
                'lower_limit': pan_limits[0],
                'upper_limit': pan_limits[1]
            },
            self.tilt_name: {
                'command': tilt_position,
                'profile_type': tilt_profile_type,
                'profile_velocity': tilt_profile_velocity,
                'profile_acceleration': tilt_profile_acceleration,
                'lower_limit': tilt_limits[0],
                'upper_limit': tilt_limits[1]}
            }
        self.change_profile(
            self.pan_name,
            pan_profile_type,
            pan_profile_velocity,
            pan_profile_acceleration
        )
        self.change_profile(
            self.tilt_name,
            tilt_profile_type,
            tilt_profile_velocity,
            tilt_profile_acceleration
        )
        print((
            f'Turret Group Name: {group_name}\n'
            f'Pan Name: {self.pan_name}, '
            f'Profile Type: {pan_profile_type}, '
            f'Profile Velocity: {pan_profile_velocity:.1f}, '
            f'Profile Acceleration: {pan_profile_acceleration:.1f}\n'
            f'Tilt Name: {self.tilt_name}, '
            f'Profile Type: {tilt_profile_type}, '
            f'Profile Velocity: {tilt_profile_velocity:.1f}, '
            f'Profile Acceleration: {tilt_profile_acceleration:.1f}'
        ))
        print('Initialized InterbotixTurretXSInterface!\n')

    def set_trajectory_profile(
        self,
        joint_name: str,
        profile_velocity: float = None,
        profile_acceleration: float = None
    ) -> None:
        """
        Command the 'Profile_Velocity' and 'Profile_Acceleration' motor registers.

        :param joint_name: joint to change
        :param profile_velocity: (optional) refer to the OperatingModes Service file for an
            explanation - note that when 'profile_type' is 'time', units are in seconds, not
            milliseconds
        :param profile_acceleration: (optional) refer to the OperatingModes Service file for an
            explanation - note that when 'profile_type' is 'time', units are in seconds, not
            milliseconds
        :details: note that if 'profile_velocity' and 'profile_acceleration' are not set, they
            retain the values they were set with previously
        """
        if (
            (profile_velocity is not None) and
            (profile_velocity != self.info[joint_name]['profile_velocity'])
        ):
            if (self.info[joint_name]['profile_type'] == 'velocity'):
                self.core.srv_set_reg(
                    cmd_type='single',
                    name=joint_name,
                    reg='Profile_Velocity',
                    value=profile_velocity
                )
            else:
                self.core.srv_set_reg(
                    cmd_type='single',
                    name=joint_name,
                    reg='Profile_Velocity',
                    value=int(profile_velocity * 1000)
                )
            self.info[joint_name]['profile_velocity'] = profile_velocity
        if (
            (profile_acceleration is not None) and
            (profile_acceleration != self.info[joint_name]['profile_acceleration'])
        ):
            if (self.info[joint_name]['profile_type'] == 'velocity'):
                self.core.srv_set_reg(
                    cmd_type='single',
                    name=joint_name,
                    reg='Profile_Acceleration',
                    value=profile_acceleration
                )
            else:
                self.core.srv_set_reg(
                    cmd_type='single',
                    name=joint_name,
                    reg='Profile_Acceleration',
                    value=int(profile_acceleration * 1000)
                )
            self.info[joint_name]['profile_acceleration'] = profile_acceleration

    def move(
        self,
        joint_name: str,
        position: float,
        profile_velocity: float = None,
        profile_acceleration: float = None,
        blocking: bool = True,
        delay: float = 0
    ) -> None:
        """
        Move a turret joint.

        :param joint_name: joint to change
        :param position: desired goal position [rad]
        :param profile_velocity: (optional) refer to the OperatingModes Service file for an
            explanation - note that when 'profile_type' is 'time', units are in seconds, not
            milliseconds
        :param profile_acceleration: (optional) refer to the OperatingModes Service file for an
            explanation - note that when 'profile_type' is 'time', units are in seconds, not
            milliseconds
        :param blocking: (optional) if 'profile_type' is 'time' and this is set to True, the
            function waits 'profile_velocity' seconds before returning control to the user;
            otherwise, the 'delay' parameter is used
        :param delay: (optional) number of seconds to wait after executing the position command
            before returning control to the user
        :details: (optional) note that if 'profile_velocity' and 'profile_acceleration' are not
            set, they retain the values they were set with previously
        """
        if (
            (self.info[joint_name]['lower_limit'] <= position) and
            (position <= self.info[joint_name]['upper_limit'])
        ):
            self.set_trajectory_profile(joint_name, profile_velocity, profile_acceleration)
            self.core.pub_single.publish(JointSingleCommand(joint_name, position))
            self.info[joint_name]['command'] = position
            if (self.info[joint_name]['profile_type'] == 'time' and blocking):
                time.sleep(self.info[joint_name]['profile_velocity'])
            else:
                time.sleep(delay)
        else:
            self.core.get_logger().warning(
                f"Goal position is outside the '{joint_name}' joint's limits."
            )

    def pan(
        self,
        position: float,
        profile_velocity: float = None,
        profile_acceleration: float = None,
        blocking: bool = True,
        delay: float = 0
    ) -> None:
        """
        Command the Pan joint on the Turret.

        :param position: desired goal position [rad]
        :param profile_velocity: refer to the OperatingModes Service file for an explanation - note
            that when 'profile_type' is 'time', units are in seconds, not milliseconds
        :param profile_acceleration: refer to the OperatingModes Service file for an explanation -
            note that when 'profile_type' is 'time', units are in seconds, not milliseconds
        :param blocking: if 'profile_type' is 'time' and this is set to True, the function waits
            'profile_velocity' seconds before returning control to the user; otherwise, the 'delay'
            parameter is used
        :param delay: number of seconds to wait after executing the position command before
            returning control to the user
        :details: note that if 'profile_velocity' and 'profile_acceleration' are not set, they
            retain the values they were set with previously
        """
        self.move(
            joint_name=self.pan_name,
            position=position,
            profile_velocity=profile_velocity,
            profile_acceleration=profile_acceleration,
            blocking=blocking,
            delay=delay
        )

    def tilt(
        self,
        position: float,
        profile_velocity: float = None,
        profile_acceleration: float = None,
        blocking: bool = True,
        delay: float = 0
    ) -> None:
        """
        Command the Tilt joint on the Turret.

        :param position: desired goal position [rad]
        :param profile_velocity: (optional) refer to the OperatingModes Service file for an
            explanation - note that when 'profile_type' is 'time', units are in seconds, not
            milliseconds
        :param profile_acceleration: (optional) refer to the OperatingModes Service file for an
            explanation - note that when 'profile_type' is 'time', units are in seconds, not
            milliseconds
        :param blocking: (optional) if 'profile_type' is 'time' and this is set to True, the
            function waits 'profile_velocity' seconds before returning control to the user;
            otherwise, the 'delay' parameter is used
        :param delay: (optional) number of seconds to wait after executing the position command
            before returning control to the user
        :details: (optional) note that if 'profile_velocity' and 'profile_acceleration' are not
            set, they retain the values they were set with previously
        """
        self.move(
            joint_name=self.tilt_name,
            position=position,
            profile_velocity=profile_velocity,
            profile_acceleration=profile_acceleration,
            blocking=blocking,
            delay=delay
        )

    def pan_tilt_go_home(
        self,
        pan_profile_velocity: float = None,
        pan_profile_acceleration: float = None,
        tilt_profile_velocity: float = None,
        tilt_profile_acceleration: float = None,
        blocking: bool = True,
        delay: float = 0
    ) -> None:
        """
        Reset the Turret to its Home pose (0 rad for both joints).

        :param pan_profile_velocity: (optional) 'pan' joint setting; refer to the OperatingModes
            Service file for an explanation - note that when 'profile_type' is 'time', units are in
            seconds, not milliseconds
        :param pan_profile_acceleration: (optional) 'pan' joint setting; refer to the
            OperatingModes Service file for an explanation - note that when 'profile_type' is
            'time', units are in seconds, not milliseconds
        :param tilt_profile_velocity: (optional) 'tilt' joint setting; refer to the OperatingModes
            Service file for an explanation - note that when 'profile_type' is 'time', units are in
            seconds, not milliseconds
        :param tilt_profile_acceleration: (optional) 'tilt' joint setting; refer to the
            OperatingModes Service file for an explanation - note that when 'profile_type' is
            'time', units are in seconds, not milliseconds
        :param blocking: (optional) if 'profile_type' for both joints is 'time' and this is set to
            `True`, the function waits either 'pan_profile_velocity' or 'tilt_profile_velocity'
            seconds (whichever is greater) before returning control to the user; otherwise, the
            'delay' parameter is used
        :param delay: (optional) number of seconds to wait after executing the position command
            before returning control to the user
        :details: note that if the 'profile_velocity' and 'profile_acceleration' parameters are not
            set, they retain the values they were set with previously
        """
        self.pan_tilt_move(
            pan_position=0,
            tilt_position=0,
            pan_profile_velocity=pan_profile_velocity,
            pan_profile_acceleration=pan_profile_acceleration,
            tilt_profile_velocity=tilt_profile_velocity,
            tilt_profile_acceleration=tilt_profile_acceleration,
            blocking=blocking,
            delay=delay
        )

    def pan_tilt_move(
        self,
        pan_position: float,
        tilt_position: float,
        pan_profile_velocity: float = None,
        pan_profile_acceleration: float = None,
        tilt_profile_velocity: float = None,
        tilt_profile_acceleration: float = None,
        blocking: bool = True,
        delay: float = 0
    ) -> None:
        """
        Command the pan and tilt joints on the Turret simultaneously.

        :param pan_position: desired pan goal position [rad]
        :param tilt_position: desired tilt goal position [rad]
        :param pan_profile_velocity: (optional) 'pan' joint setting; refer to the OperatingModes
            Service file for an explanation - note that when 'profile_type' is 'time', units are in
            seconds, not milliseconds
        :param pan_profile_acceleration: (optional) 'pan' joint setting; refer to the
            OperatingModes Service file for an explanation - note that when 'profile_type' is
            'time', units are in seconds, not milliseconds
        :param tilt_profile_velocity: (optional) 'tilt' joint setting; refer to the OperatingModes
            Service file for an explanation - note that when 'profile_type' is 'time', units are in
            seconds, not milliseconds
        :param tilt_profile_acceleration: (optional) 'tilt' joint setting; refer to the
            OperatingModes Service file for an explanation - note that when 'profile_type' is
            'time', units are in seconds, not milliseconds
        :param blocking: (optional) if 'profile_type' for both joints is 'time' and this is set to
            True, the function waits either 'pan_profile_velocity' or 'tilt_profile_velocity'
            seconds (whichever is greater) before returning control to the user; otherwise, the
            'delay' parameter is used
        :param delay: (optional) number of seconds to wait after executing the position command
            before returning control to the user
        :details: note that if the 'profile_velocity' and 'profile_acceleration' parameters are not
            set, they retain the values they were set with previously
        """
        if (
            (self.info[self.pan_name]['lower_limit'] <= pan_position) and
            (pan_position <= self.info[self.pan_name]['upper_limit']) and
            (self.info[self.tilt_name]['lower_limit'] <= tilt_position) and
            (tilt_position <= self.info[self.tilt_name]['upper_limit'])
        ):
            self.set_trajectory_profile(
                self.pan_name,
                pan_profile_velocity,
                pan_profile_acceleration
            )
            self.set_trajectory_profile(
               self.tilt_name,
               tilt_profile_velocity,
               tilt_profile_acceleration
            )
            self.core.pub_group.publish(
               JointGroupCommand(self.group_name, [pan_position, tilt_position]))
            self.info[self.pan_name]['command'] = pan_position
            self.info[self.tilt_name]['command'] = tilt_position
            if (
                (self.info[self.pan_name]['profile_type'] == 'time') and
                (self.info[self.tilt_name]['profile_type'] == 'time') and
                (blocking)
            ):
                time.sleep(
                    max(
                        self.info[self.pan_name]['profile_velocity'],
                        self.info[self.tilt_name]['profile_velocity']
                    )
                )
            else:
                time.sleep(delay)
        else:
            self.core.get_logger().warning('One or both goal positions are outside the limits!')

    def change_profile(
        self,
        joint_name: str,
        profile_type: str,
        profile_velocity: float,
        profile_acceleration: float,
    ) -> None:
        """
        Change the Profile Type for a given joint.

        :param joint_name: joint to change
        :param profile_type: either 'time' or 'velocity'; refer to the OperatingModes Service file
            for details
        :param profile_velocity: refer to the OperatingModes Service file for an explanation - note
            that when 'profile_type' is 'time', units are in seconds, not milliseconds
        :param profile_acceleration: refer to the OperatingModes Service file for an explanation -
            note that when 'profile_type' is 'time', units are in seconds, not milliseconds
        """
        if (profile_type == 'velocity'):
            self.core.srv_set_op_modes(
                'single',
                joint_name,
                'position',
                'velocity',
                profile_velocity,
                profile_acceleration
            )
            self.info[joint_name]['profile_velocity'] = profile_velocity
            self.info[joint_name]['profile_acceleration'] = profile_acceleration
        else:
            self.core.srv_set_op_modes(
                'single',
                joint_name,
                'position',
                'time',
                int(profile_velocity * 1000),
                int(profile_acceleration * 1000)
            )
            self.info[joint_name]['profile_velocity'] = profile_velocity
            self.info[joint_name]['profile_acceleration'] = profile_acceleration
        self.info[joint_name]['profile_type'] = profile_type

    def get_command(self, joint_name: str) -> float:
        """
        Get the last commanded position for a given joint.

        :param joint_name: desired joint name
        :return: last commanded position [rad]
        """
        return self.info[joint_name]['command']

    def get_joint_commands(self) -> List[float]:
        """
        Get the last commanded positions for the joints.

        :return: list of last commanded positions [rad]
        """
        return [self.info[self.pan_name]['command'], self.info[self.tilt_name]['command']]
