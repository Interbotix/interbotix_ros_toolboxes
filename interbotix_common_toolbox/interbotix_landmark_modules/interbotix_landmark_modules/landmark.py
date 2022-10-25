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
from typing import Dict, List, Union

from geometry_msgs.msg import Pose, PoseStamped, Transform, TransformStamped
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.time import Time
import tf2_ros
from tf2_ros import TransformException
from tf_transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray
import yaml


class Landmark:
    """
    Module that describes a landmark.

    Stores data about landmarks, calculate transforms, and publish that information to the
        static TF tree.
    """

    def __init__(
        self,
        label: str,
        id_num: int,
        node_inf: Node = None,
        landmark_ns: str = 'landmarks',
    ):
        """
        Construct a Landmark object.

        :param label: name of this tracked landmark
        :param id_num: ID of this tracked landmark
        :param node_inf: reference to the rclpy.node.Node on which to build this interface
        :param landmark_ns: (optional) namespace where the ROS parameters needed by the module are
            located. Defaults to "landmarks".
        """
        self.node_inf = node_inf

        # tf buffer and listener
        self.tf_buffer = tf2_ros.Buffer(node=node_inf)
        self.listener = tf2_ros.TransformListener(buffer=self.tf_buffer, node=self.node_inf)

        self.label_ = label
        self.id_ = id_num
        self.landmark_ns = landmark_ns

        # transforms
        self.tf_wrt_cam = TransformStamped()
        self.tf_wrt_cam.child_frame_id = self.label_
        self.tf_wrt_map = TransformStamped()
        self.tf_wrt_map.child_frame_id = self.label_
        self.mounted_offset = 0.0

        # flags
        self.tf_set_ = False
        self.mounted_ = False

        self._init_markers()

    def get_id(self) -> int:
        """
        Get id.

        :return: id
        """
        return self.id_

    def set_id(self, id_num: int):
        """
        Set id.

        :param id_num: new id number for this landmark
        """
        self.id_ = id_num

    def transform_to_new_frame(self, parent_old: str, parent_new: str) -> PoseStamped:
        """
        Transform frame from one fixed pose to another.

        :param parent_old: current fixed frame
        :param parent_new: desired fixed frame
        :return: pose in new fixed frame
        """
        tf_old = PoseStamped()
        tf_old.pose.position.x = self.tf_wrt_cam.transform.translation.x
        tf_old.pose.position.y = self.tf_wrt_cam.transform.translation.y
        tf_old.pose.position.z = self.tf_wrt_cam.transform.translation.z
        tf_old.pose.orientation.x = self.tf_wrt_cam.transform.rotation.x
        tf_old.pose.orientation.y = self.tf_wrt_cam.transform.rotation.y
        tf_old.pose.orientation.z = self.tf_wrt_cam.transform.rotation.z
        tf_old.pose.orientation.w = self.tf_wrt_cam.transform.rotation.w
        tf_old.header.frame_id = parent_old
        tf_old.header.stamp = self.node_inf.get_clock().now().to_msg()
        try:
            # transform pose to frame parent_new
            tf_new = self.tf_buffer.transform(
                object_stamped=tf_old,
                target_frame=parent_new,
            )
            # set/update tf with respect to map
            return tf_new
            # self.set_tf_wrt_map(tf_new) # TODO
        except TransformException as e:
            self.node_inf.get_logger().error(
                f"Could not transform between frames '{parent_new}' and '{parent_old}': {e}",
            )
            raise e

    def get_label(self) -> str:
        """
        Get label.

        :return: label
        "rtype: string
        """
        return self.label_

    def set_label(self, label: str):
        """
        Set label.

        :param label: new label for this landmark
        """
        self.label_ = label

    def get_x(self) -> float:
        """
        Get landmark x position with respect to map frame.

        :return: landmark x position with respect to map frame
        :details: returns 0.0 if the TF of the landmark has not been set
        """
        if self.tf_set_:
            return self.tf_wrt_map.transform.translation.x
        else:
            return 0.0

    def get_y(self) -> float:
        """
        Get landmark y position with respect to map frame.

        :return: landmark y position with respect to map frame
        :details: returns 0.0 if the TF of the landmark has not been set
        """
        if self.tf_set_:
            return self.tf_wrt_map.transform.translation.y
        else:
            return 0.0

    def get_theta(self) -> float:
        """
        Get landmark yaw rotation with respect to map frame.

        :return: landmark yaw rotation with respect to map frame
        :details: returns 0.0 if the TF of the landmark has not been set
        """
        if self.tf_set_:
            q = self.tf_wrt_map.transform.rotation
            return euler_from_quaternion(
                [q.x, q.y, q.z, q.w],
                'rxyz'
            )[2]
        else:
            return 0.0

    def set_tf_wrt_cam(self, tf: Union[Transform, TransformStamped]):
        """
        Set for transform with respect to camera frame.

        :param tf: translation and orientation from cam to landmark
        """
        if isinstance(tf, Transform):
            self.tf_wrt_cam.transform.translation = tf.translation
            self.tf_wrt_cam.transform.rotation = tf.rotation
        elif isinstance(tf, TransformStamped):
            self.tf_wrt_cam.transform.translation = tf.transform.translation
            self.tf_wrt_cam.transform.rotation = tf.transform.rotation
        else:
            raise TypeError(
                'Must be of type geometry_msgs.msg.Transform or '
                f'geometry_msgs.msg.TransformStamped, was type {type(tf)}.'
            )

    def set_cam_frame_id(self, frame_id: str):
        """
        Set transform with respect to camera frame frame id.

        :param frame_id: The frame_id the transform should be relative to
        """
        self.tf_wrt_cam.header.frame_id = frame_id

    def get_tf_wrt_cam(self) -> TransformStamped:
        """
        Get for transform with respect to camera.

        :return: pose transform with respect to camera
        """
        return self.tf_wrt_cam

    def set_tf_wrt_map(self, pose: Union[Pose, PoseStamped], parent_new: str = 'map'):
        """
        Set transform with respect to map frame.

        :param pose: translation and orientation from cam to landmark
        :param parent_new: (optional) name of map frame. defaults to 'map'
        """
        if isinstance(pose, Pose):
            self.tf_wrt_map.transform.translation.x = pose.position.x
            self.tf_wrt_map.transform.translation.y = pose.position.y
            self.tf_wrt_map.transform.translation.z = pose.position.z
            self.tf_wrt_map.transform.rotation.x = pose.orientation.x
            self.tf_wrt_map.transform.rotation.y = pose.orientation.y
            self.tf_wrt_map.transform.rotation.z = pose.orientation.z
            self.tf_wrt_map.transform.rotation.w = pose.orientation.w
        elif isinstance(pose, PoseStamped):
            self.tf_wrt_map.transform.translation.x = pose.pose.position.x
            self.tf_wrt_map.transform.translation.y = pose.pose.position.y
            self.tf_wrt_map.transform.translation.z = pose.pose.position.z
            self.tf_wrt_map.transform.rotation.x = pose.pose.orientation.x
            self.tf_wrt_map.transform.rotation.y = pose.pose.orientation.y
            self.tf_wrt_map.transform.rotation.z = pose.pose.orientation.z
            self.tf_wrt_map.transform.rotation.w = pose.pose.orientation.w
        else:
            raise TypeError(
                'Must be of type geometry_msgs.msg.Pose or geometry_msgs.msg.PoseStamped, '
                f'was {type(pose)}.'
            )
        self.tf_wrt_map.header.stamp = self.node_inf.get_clock().now().to_msg()
        self.tf_wrt_map.header.frame_id = parent_new

    def get_tf_wrt_map(self) -> TransformStamped:
        """
        Set transform with respect to fixed frame.

        :return: transform with respect to fixed frame
        """
        return self.tf_wrt_map

    def update_tfs(self, parent_old: str, parent_new: str):
        """
        Update the transformation to fixed frame.

        :param parent_old: current frame
        :param parent_old: new frame
        :details: successfully calling this function will set the TF of this landmark
        """
        self.set_tf_wrt_map(
            pose=self.transform_to_new_frame(parent_old, parent_new),
            parent_new=parent_new
        )
        self.tf_set_ = True

    def set_mounted(self, mounted: bool):
        """
        Set mounted status of this landmark.

        :param mounted: boolean value to set the mounted status of this landmark to
        """
        self.mounted_ = mounted

    def is_mounted(self) -> bool:
        """
        Get the mounted status of this landmark.

        :return: `True` if this landmark is mounted; `False` otherwise
        """
        return self.mounted_

    def set_mounted_offset(self, offset: float = 0.0):
        """
        Set the mounted offset of this landmark.

        :param offset: (optional) the offset of this landmark. Defaults to 0.0
        """
        self.mounted_offset = offset

    def get_mounted_offset(self) -> float:
        """
        Get the mounted offset of this landmark.

        :return: the mounted offset of this landmark.
        """
        return self.mounted_offset

    def get_nav_goal(self) -> List[float]:
        """
        Get for the [x, y, theta] navigation goal for this landmark.

        :return: list containing [x, y, theta] navigation goal for this landmark
        """
        if not self.is_mounted():  # know offset is 0, can just return the [x, y, theta]
            return [self.get_x(), self.get_y(), self.get_theta()]
        else:
            tf_goal = TransformStamped()
            tf_goal.header.stamp = Time()
            tf_goal.header.frame_id = self.get_label()
            tf_goal.child_frame_id = f'{self.get_label()}_goal'

            # z-axis is normal to tag face, 'front of tag'
            tf_goal.transform.translation.z = self.get_mounted_offset()
            tf_goal.transform.rotation.w = 1.0  # ensure valid quaternion
            self.tf_buffer.set_transform(
                transform=tf_goal,
                authority='default_authority'
            )

            # get tf from map to goal frame [x, y, theta]
            tf_map_to_goal = self.tf_buffer.lookup_transform(
                target_frame=self.landmark_ns,
                source_frame=tf_goal.child_frame_id,
                time=Time()
            )
            return [
                tf_map_to_goal.transform.translation.x,
                tf_map_to_goal.transform.translation.y,
                euler_from_quaternion([
                    tf_map_to_goal.transform.rotation.x,
                    tf_map_to_goal.transform.rotation.y,
                    tf_map_to_goal.transform.rotation.z,
                    tf_map_to_goal.transform.rotation.w
                ])[2]
            ]

    def _init_markers(self) -> None:
        """Initialize marker values."""
        # spherical marker (location)
        self.marker_sphere = Marker()
        self.marker_sphere.id = 0
        self.marker_sphere.header.frame_id = self.landmark_ns
        self.marker_sphere.header.stamp = Time().to_msg()
        self.marker_sphere.ns = self.get_label()
        self.marker_sphere.type = Marker.SPHERE
        self.marker_sphere.action = Marker.ADD
        self.marker_sphere.scale.x = 0.025
        self.marker_sphere.scale.y = 0.025
        self.marker_sphere.scale.z = 0.025
        self.marker_sphere.color.a = 1.0
        self.marker_sphere.color.r = 1.0
        self.marker_sphere.color.g = 0.0
        self.marker_sphere.color.b = 0.0
        self.marker_sphere.pose.position.z = 0.0

        # text marker (label)
        self.marker_text = Marker()
        self.marker_text.id = 1
        self.marker_text.header.frame_id = self.landmark_ns
        self.marker_text.header.stamp = Time().to_msg()
        self.marker_text.ns = self.get_label()
        self.marker_text.type = Marker.TEXT_VIEW_FACING
        self.marker_text.text = f'{self.get_label()}_goal'
        self.marker_text.scale.x = 0.05
        self.marker_text.scale.y = 0.05
        self.marker_text.scale.z = 0.05
        self.marker_text.color.a = 1.0
        self.marker_text.color.r = 1.0
        self.marker_text.color.g = 1.0
        self.marker_text.color.b = 1.0
        self.marker_text.pose.position.z = 0.05

    def __eq__(self, other):
        if isinstance(other, str):  # match label if string
            return self.get_label() == other
        elif isinstance(other, int):  # match id if int
            return self.id_ == other

        if isinstance(other, Landmark):  # if Landmark, check both
            return (self.get_id() == other.get_id() or self.get_label() == other.get_label())

    def __repr__(self):
        """
        Define repr.

        Prints in the form of:
        ---
        label: {}
        id: {}
        tf_self: {}
        mounted: {}
        mounted_offset: {}
        tf: {
            ...
        }
        ---
        """
        tf = self.get_tf_wrt_map() if self.tf_set_ else self.get_tf_wrt_cam()
        return (
            f'---\nlabel: {self.get_label()}\nid: {self.get_id()}\ntf_set: {self.tf_set_}\n'
            f'mounted: {self.is_mounted()}\nmounted_offset: {self.get_mounted_offset()}\n'
            f'tf:\n{tf}\n---'
        )


class LandmarkCollection:
    """Class to interact with Landmark data structure."""

    static_tf_pub: Publisher
    marker_pub: Publisher

    def __init__(
        self,
        node_inf: Node,
        landmarks: Dict = {},
        observation_frame: str = None,
        fixed_frame: str = None,
        ros_on: bool = False,
    ):
        """
        Construct LandmarkCollection object.

        :param node_inf: reference to the rclpy.node.Node on which to build this interface
        :param landmarks: (optional): A dictionary of ints to Landmarks. Defaults to an empty
            dictionary.
        :param observation_frame: (optional): Frame that the Landmarks will be observed from;
            typically the camera's optical frame. Defaults to `None`.
        :param fixed_frame: (optional): Fixed frame that the Landmarks should be published relative
            to. Defaults to `None`.
        :param ros_on: (optional): Whether or not ROS is on. Defaults to `False`.
        """
        self.node_inf = node_inf
        self.data = landmarks

        # if we will use ROS, get params and set up pubs/subs
        if ros_on:
            self.ROS = True

            self.observation_frame = self.node_inf.get_parameter_or(
                'observation_frame',
                observation_frame
            )
            self.fixed_frame = self.node_inf.get_parameter_or(
                'fixed_frame',
                fixed_frame)

            self.static_tf_pub = self.node_inf.create_publisher(
                'static_transforms',
                TransformStamped,
                qos_profile=10,
            )

            self.marker_pub = self.node_inf.create_publisher(
                'landmark_markers',
                MarkerArray,
                qos_profile=10,)

        else:
            self.ROS = False
            self.observation_frame = observation_frame
            self.fixed_frame = fixed_frame

    def get_landmark(self, id_num: int) -> Landmark:
        """
        Get the specified landmark corresponding to the ID numbers.

        :return: landmark with specified ID number
        """
        return self.data.get(id_num)

    def get_landmarks(self, id_nums: Union[List[int], None] = None) -> List[Landmark]:
        """
        Get the specified landmarks corresponding to the ID numbers.

        :return: landmarks with specified ID number
        :details: If id_num is not given, returns all landmarks as dict_values. If id_num is a list
            of ints, returns landmarks corresponding to the ID numbers in that list
        """
        if id_nums is None:
            return self.data.values()
        elif isinstance(id_nums, list):
            return [self.data[x] for x in id_nums]
        else:
            raise TypeError('Must be of type List[int], or None.')

    def add_landmark(self, label: str, id_num: int):
        """
        Add landmark to data dictionary.

        :param label: label for added landmark to add
        :param id_num: tag id for added landmark and dictionary key
        """
        self.data.update({
            id_num: Landmark(
                label=label,
                id_num=id_num,
                node_inf=self.node_inf,
            )
        })

    def remove_landmark(self, id_num: int):
        """
        Remove landmark from data dictionary.

        :param id_num: id to remove
        """
        self.data.pop(id_num)

    def get_valid_tags(self) -> List[int]:
        """
        Return the ids of the tags of any landmark in the Collection.

        :return: list of ids corresponsing to landmarks in the Collection
        :details: updates the list of valid tags before returning
        """
        self.update_valid_tags()
        return self.valid_tags

    def update_valid_tags(self) -> None:
        """Update the list of valid tags."""
        self.valid_tags = [lm.get_id() for lm in self.get_landmarks()]

    def get_set_tags(self) -> List[int]:
        """
        Return the list of seen tags.

        :return: list of ids corresponding to seen landmarks in the Collection
        """
        return [lm.get_id() for lm in self.get_landmarks() if lm.tf_set_]

    def get_set_landmarks(self) -> List[Landmark]:
        """
        Return the list of seen landmarks.

        :return: list of seen landmarks in the Collection
        """
        return [lm for lm in self.get_landmarks() if lm.tf_set_]

    def save(self, filepath, ids: Union[List[int], None] = None) -> bool:
        """
        Save landmarks to a specified yaml file.

        :param filepath: absolute filepath to save structured landmark data to
        :param ids: (optional) IDs of the landmarks to save. If set to `None`, saves all Landmarks.
            Defaults to `None`
        :return: `True` if saved successfully, `False` otherwise
        :details: if the collection is empty, this method will always return `True`
        """
        if self.is_empty():
            return True

        lm_yaml = {}
        for lm in self.get_landmarks(ids):
            tf_map = lm.get_tf_wrt_map()
            lm_dict = {}
            lm_dict['id'] = lm.id_
            lm_dict['label'] = lm.label_
            lm_dict['set'] = lm.tf_set_
            lm_dict['mounted'] = lm.is_mounted()
            lm_dict['mounted_offset'] = lm.mounted_offset
            if lm.tf_set_:
                lm_dict['tf'] = {}
                lm_dict['tf']['frame_id'] = tf_map.header.frame_id
                lm_dict['tf']['child_frame_id'] = tf_map.child_frame_id
                lm_dict['tf']['x'] = float(tf_map.transform.translation.x)
                lm_dict['tf']['y'] = float(tf_map.transform.translation.y)
                lm_dict['tf']['z'] = float(tf_map.transform.translation.z)
                lm_dict['tf']['qx'] = float(tf_map.transform.rotation.x)
                lm_dict['tf']['qy'] = float(tf_map.transform.rotation.y)
                lm_dict['tf']['qz'] = float(tf_map.transform.rotation.z)
                lm_dict['tf']['qw'] = float(tf_map.transform.rotation.w)
            lm_yaml[lm.id_] = lm_dict

        with open(filepath, 'w') as yamlfile:
            yaml.dump(lm_yaml, yamlfile, default_flow_style=False)
        self.node_inf.get_logger().info(f"Saved landmarks to '{filepath}'.")

        return True

    def load(self, filepath: str) -> bool:
        """
        Load landmarks from a specified yaml file.

        :param filepath: path to yaml file containing structured landmark data
        :return: `True` if loaded successfully, `False` otherwise
        """
        try:
            with open(filepath, 'r') as yamlfile:
                lm_dict = yaml.safe_load(yamlfile)

            if lm_dict is None:
                return

            for key in lm_dict.keys():

                self.add_landmark(
                    label=lm_dict[key]['label'],
                    id_num=key)

                # get transform from configs
                if 'tf' in lm_dict[key].keys():
                    self.data[key].tf_wrt_map = TransformStamped()
                    self.data[key].tf_wrt_map.header.stamp = (
                        self.node_inf.get_clock().now().to_msg()
                    )
                    if lm_dict[key]['tf']['frame_id'] == '':
                        self.data[key].tf_wrt_map.header.frame_id = self.observation_frame
                    else:
                        self.data[key].tf_wrt_map.header.frame_id = lm_dict[key]['tf']['frame_id']
                    self.data[key].tf_wrt_map.child_frame_id = lm_dict[key]['tf']['child_frame_id']
                    self.data[key].tf_wrt_map.transform.translation.x = lm_dict[key]['tf']['x']
                    self.data[key].tf_wrt_map.transform.translation.y = lm_dict[key]['tf']['y']
                    self.data[key].tf_wrt_map.transform.translation.z = lm_dict[key]['tf']['z']
                    self.data[key].tf_wrt_map.transform.rotation.x = lm_dict[key]['tf']['qx']
                    self.data[key].tf_wrt_map.transform.rotation.y = lm_dict[key]['tf']['qy']
                    self.data[key].tf_wrt_map.transform.rotation.z = lm_dict[key]['tf']['qz']
                    self.data[key].tf_wrt_map.transform.rotation.w = lm_dict[key]['tf']['qw']
                    self.data[key].tf_set_ = True
                else:
                    self.data[key].tf_wrt_cam.header.frame_id = self.observation_frame
                    self.data[key].tf_wrt_cam.child_frame_id = self.data[key].get_label()
                    self.data[key].tf_set_ = False

                # get mounted/mounted_offset values from configs
                if 'mounted' in lm_dict[key].keys():
                    m = lm_dict[key]['mounted']
                    mo = lm_dict[key]['mounted_offset']

                self.data[key].mounted_ = m
                self.data[key].set_mounted_offset(mo)

            self.update_valid_tags()
            if self.ROS:
                self.pub_tfs(self.valid_tags)
                time.sleep(1.0)  # wait to make sure tf is published
                self.pub_markers(self.valid_tags)
            return True

        except IOError:
            self.node_inf.get_logger().warn(
                f"File at '{filepath}' does not exist yet. No landmarks loaded."
            )
            return False

    def is_empty(self) -> bool:
        """
        Check if landmarks is empty.

        :return: `True` if landmarks is empty, `False` otherwise
        """
        return self.data == {}

    def pub_tfs(self, tag_ids: Union[List[int], None] = None):
        """
        Publish TFs to static transforms.

        :param tag_ids: list of tags to publish. Publishes all set Landmarks if `None`. Defaults to
            `None`
        """
        if self.is_empty():
            return True

        if tag_ids is None:
            # when no tags are specified, pubs all
            for lm in self.get_landmarks():
                if lm.tf_set_:
                    self.static_tf_pub.publish(
                        lm.get_tf_wrt_map())
        else:
            # when list of tags is specified, pubs tags in list
            for lm in [self.get_landmark(x) for x in tag_ids]:
                if lm.tf_set_:
                    self.static_tf_pub.publish(
                        lm.get_tf_wrt_map())

    def update_markers(self):
        """Update markers."""
        for lm in self.get_set_landmarks():  # only publish seen tags
            g = lm.get_nav_goal()
            lm.marker_sphere.pose.position.x = g[0]
            lm.marker_sphere.pose.position.y = g[1]
            lm.marker_sphere.pose.position.z = 0.1
            lm.marker_sphere.pose.orientation.x = 0.0
            lm.marker_sphere.pose.orientation.y = 0.0
            lm.marker_sphere.pose.orientation.z = 0.0
            lm.marker_sphere.pose.orientation.w = 1.0
            lm.marker_sphere.header.stamp = Time()

            lm.marker_text.pose.position.x = g[0]
            lm.marker_text.pose.position.y = g[1]
            lm.marker_text.pose.position.z = 0.15
            lm.marker_text.pose.orientation.x = 0.0
            lm.marker_text.pose.orientation.y = 0.0
            lm.marker_text.pose.orientation.z = 0.0
            lm.marker_text.pose.orientation.w = 1.0
            lm.marker_text.header.stamp = Time()

    def pub_markers(self, tag_ids: Union[List[int], int, None] = None):
        """
        Publish specified markers.

        :param tag_ids: IDs to publish markers of. Will publish all if set to `None`. Defaults to
            `None`
        """
        if not self.ROS:
            self.node_inf.get_logger().warn('Tried to publish marker but node is not active.')
            return
        else:
            self.update_markers()
            msg = MarkerArray()
            for lm in self.get_set_landmarks():
                if tag_ids is None:
                    msg.markers.append(lm.marker_sphere)
                    msg.markers.append(lm.marker_text)
                    self.marker_pub.publish(msg)
                if lm.get_id() in tag_ids:
                    msg.markers.append(lm.marker_sphere)
                    msg.markers.append(lm.marker_text)
                    self.marker_pub.publish(msg)

    def __repr__(self):
        ls = []
        for lm in self.get_landmarks():
            ls.append(lm)
        return str(ls)

    def __len__(self):
        return len(self.data)
