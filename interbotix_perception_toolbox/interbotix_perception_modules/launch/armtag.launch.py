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

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):

    tags_config_launch_arg = LaunchConfiguration('tags_config')
    camera_frame_launch_arg = LaunchConfiguration('camera_frame')
    apriltag_ns_launch_arg = LaunchConfiguration('apriltag_ns')
    camera_color_topic_launch_arg = LaunchConfiguration('camera_color_topic')
    camera_info_topic_launch_arg = LaunchConfiguration('camera_info_topic')

    armtag_ns_launch_arg = LaunchConfiguration('armtag_ns')
    ref_frame_launch_arg = LaunchConfiguration('ref_frame')
    arm_base_frame_launch_arg = LaunchConfiguration('arm_base_frame')
    arm_tag_frame_launch_arg = LaunchConfiguration('arm_tag_frame')
    position_only_launch_arg = LaunchConfiguration('position_only')
    use_armtag_tuner_gui_launch_arg = LaunchConfiguration('use_armtag_tuner_gui')

    apriltag_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_perception_modules'),
                'launch',
                'apriltag.launch.py',
            ])
        ]),
        launch_arguments={
            'tags_config': tags_config_launch_arg,
            'camera_frame': camera_frame_launch_arg,
            'apriltag_ns': apriltag_ns_launch_arg,
            'camera_color_topic': camera_color_topic_launch_arg,
            'camera_info_topic': camera_info_topic_launch_arg,
        }.items()
    )

    armtag_tuner_gui_node = Node(
        package='interbotix_perception_modules',
        executable='armtag_tuner_gui',
        namespace=armtag_ns_launch_arg,
        parameters=[{
            'position_only': position_only_launch_arg.perform(context),
            'ref_frame': ref_frame_launch_arg.perform(context),
            'arm_base_frame': arm_base_frame_launch_arg.perform(context),
            'arm_tag_frame': arm_tag_frame_launch_arg.perform(context),
        }],
        arguments=[
            '--armtag_ns', armtag_ns_launch_arg.perform(context),
            '--apriltag_ns', apriltag_ns_launch_arg.perform(context)
        ],
        condition=IfCondition(use_armtag_tuner_gui_launch_arg),
    )

    return [
        apriltag_launch_include,
        armtag_tuner_gui_node,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'tags_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_perception_modules'),
                'config',
                'tags.yaml'
            ]),
            description='',
        ),
        DeclareLaunchArgument(
            'camera_frame',
            default_value='camera_color_optical_frame',
            description='',
        ),
        DeclareLaunchArgument(
            'apriltag_ns',
            default_value='apriltag',
            description='',
        ),
        DeclareLaunchArgument(
            'camera_color_topic',
            default_value='camera/color/image_raw',
            description='',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='camera/color/camera_info',
            description='',
        ),
        DeclareLaunchArgument(
            'armtag_ns',
            default_value='armtag',
            description='',
        ),
        DeclareLaunchArgument(
            'ref_frame',
            default_value='camera_color_optical_frame',
            description='',
        ),
        DeclareLaunchArgument(
            'arm_base_frame',
            default_value='base_link',
            description='',
        ),
        DeclareLaunchArgument(
            'arm_tag_frame',
            default_value='ar_tag_link',
            description='',
        ),
        DeclareLaunchArgument(
            'use_armtag_tuner_gui',
            default_value='false',
            description='',
        ),
        DeclareLaunchArgument(
            'position_only',
            default_value='false',
            description='',
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
