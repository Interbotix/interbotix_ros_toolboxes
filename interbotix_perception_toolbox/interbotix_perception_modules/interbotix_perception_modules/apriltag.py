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
from typing import List, Sequence, Tuple, Union

from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray
from apriltag_msgs.srv import AnalyzeSingleImage
from geometry_msgs.msg import Pose, TransformStamped
from interbotix_perception_msgs.srv import SnapPicture
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo


class InterbotixAprilTagInterface(Node):
    """Python API to snap the pose of an AprilTag."""

    valid_tags = [5, 413, 820, 875, 1050]
    v = False

    def __init__(
        self,
        apriltag_ns: str = 'apriltag',
        full_img_get_path: str = '/tmp/get_image.png',
        full_img_save_path: str = '/tmp/save_image.png',
        node_inf: Node = None,
        args=None
    ) -> None:
        """
        Construct the InterbotixAprilTagInterface node.

        :param apriltag_ns: the namespace that the AprilTag node and other related parameters are
            in
        :param node_inf: reference to the rclpy.node.Node on which to build this interface. Leave
            as `None` to let this interface serve as its own Node
        """
        if node_inf is None:
            # start apriltag interface node
            rclpy.init(args=args)
            super().__init__(f"{apriltag_ns.strip('/')}_interface")
            self.node_inf = self
        else:
            self.node_inf = node_inf

        self.image_frame_id: str = None

        # set up subs, pubs, and services
        self.node_inf.declare_parameter(
            f'/{apriltag_ns}/camera_info_topic',
            '/camera/color/camera_info'
        )
        camera_info_topic = self.node_inf.get_parameter(
            f'/{apriltag_ns}/camera_info_topic').get_parameter_value().string_value.strip('/')
        self.srv_snap_picture = self.node_inf.create_client(
            SnapPicture,
            f'/{apriltag_ns}/snap_picture'
        )
        self.srv_analyze_image = self.node_inf.create_client(
            AnalyzeSingleImage,
            f'/{apriltag_ns}/single_image_tag_detection'
        )
        self.srv_snap_picture.wait_for_service()
        self.srv_analyze_image.wait_for_service()
        self.sub_camera_info = self.node_inf.create_subscription(
            CameraInfo,
            f'/{camera_info_topic}',
            self.camera_info_cb,
            10
        )
        self.request = AnalyzeSingleImage.Request()
        self.request.full_path_where_to_get_image = full_img_get_path
        self.request.full_path_where_to_save_image = full_img_save_path
        self.pub_transforms = self.node_inf.create_publisher(
            TransformStamped,
            '/static_transforms',
            50
        )

        time.sleep(0.5)

        # wait to receive camera info (means that we are properly subbed)
        while (self.request.camera_info == CameraInfo() and rclpy.ok()):
            rclpy.spin_once(self.node_inf)
        self.node_inf.destroy_subscription(self.sub_camera_info)

        self.node_inf.get_logger().info('Initialized InterbotixAprilTagInterface!')

    def camera_info_cb(self, msg: CameraInfo) -> None:
        """
        Get the camera info through a ROS subscriber callback.

        :param msg: ROS CameraInfo message
        :details: unsubscribes after receiving the first message as this never changes
        """
        self.image_frame_id = msg.header.frame_id
        self.request.camera_info = msg

    def find_pose(
        self,
        ar_tag_name: str = 'ar_tag',
        publish_tf: bool = False
    ) -> Pose:
        """
        Calculate the AprilTag pose w.r.t. the camera color image frame.

        :param ar_tag_name: AprilTag name (only used if publishing to the /tf tree)
        :param publish_tf: whether to publish the pose of the AprilTag to the /tf tree
        :return: Pose message of the proposed AR tag w.r.t. the camera color image frame

        :details: tf is published to the StaticTransformManager node (in this package) as a static
            transform if no tags are detected, a genertic invalid pose is returned
        """
        detections: Sequence[AprilTagDetection] = self._snap().detections

        # check if tags were detected, return genertic pose if not
        if len(detections) == 0:
            pose = Pose()
            if self.v:
                self.node_inf.get_logger().warning(
                    f"Could not find '{ar_tag_name}'. Returning a 'zero' Pose..."
                )
        else:
            pose = detections[0].pose.pose.pose  # TODO: support for multiple tags

        # publish pose to /static_transforms
        if publish_tf:
            msg = TransformStamped()
            msg.header.frame_id = self.image_frame_id
            msg.header.stamp = Time()
            msg.child_frame_id = ar_tag_name
            msg.transform.translation.x = pose.position.x
            msg.transform.translation.y = pose.position.y
            msg.transform.translation.z = pose.position.z
            msg.transform.rotation = pose.orientation
            self.pub_transforms.publish(msg)

        return pose

    def _snap(self) -> AprilTagDetectionArray:
        """
        Take snapshot using current camera, returns tag detections.

        :return: list of tag detections
        """
        # TODO(LSinterbotix): clean this up
        future_snap = self.srv_snap_picture.call_async(
            SnapPicture.Request(filename=self.request.full_path_where_to_get_image)
        )
        rclpy.spin_until_future_complete(self.node_inf, future=future_snap)
        future_analyze = self.srv_analyze_image.call_async(self.request)
        rclpy.spin_until_future_complete(self.node_inf, future=future_analyze)
        return future_analyze.result().tag_detections

    def set_valid_tags(self, ids: Union[List[int], None]):
        """
        Set list of valid tags.

        :param ids: list of valid ids
        :type ids: list of ints
        """
        if ids is not None:
            self.valid_tags = set(ids)

    def find_pose_id(self) -> Tuple[List[Pose], List[int]]:
        """
        Find the pose of valid tags that can be seen by the camera.

        :return: poses of the tags relative to camera, corresponding ids
        :rtype: list of Poses, list of ints
        """
        if self.valid_tags is None:
            self.node_inf.get_logger().warning(
                'Tried to find pose of valid tags but valid ids are not set'
            )
        detections: Sequence[AprilTagDetection] = self._snap().detections
        if len(detections) == 0:
            return [], []
        poses: List[Pose] = []
        tags: List[int] = []
        for d in detections:
            if d.id[0] in self.valid_tags:
                poses.append(d.pose.pose.pose)
                tags.append(d.id[0])
        return poses, tags
