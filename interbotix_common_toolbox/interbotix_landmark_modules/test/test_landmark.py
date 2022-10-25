#!/usr/bin/env python3

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


# from asyncio import Future
# import time
import unittest

# from geometry_msgs.msg import Pose, Transform
from interbotix_landmark_modules.landmark import Landmark, LandmarkCollection
import rclpy
# from std_msgs.msg import Header
# from rclpy.time import Time
import tf2_ros
# from tf2_geometry_msgs import TransformStamped


class LandmarkTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

    def setUp(self):
        # Create a ROS node for tests
        self.node = rclpy.create_node('test_landmark')
        self.tf_buffer = tf2_ros.Buffer(node=self.node)
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(node=self.node)

    def tearDown(self):
        self.node.destroy_node()

    def test_label(self):
        """Test landmark.get_label()."""
        label = 'test'
        lm = Landmark(label, 0, node_inf=self.node)
        self.assertEqual(
            label,
            lm.get_label(),
            'Landmark label is incorrect.'
        )

    def test_id(self):
        """Test landmark.get_id()."""
        id_num = 1
        lm = Landmark('test', 0, node_inf=self.node)
        lm.set_id(id_num)
        self.assertEqual(
            id_num, lm.get_id(),
            'Landmark ID is incorrect.'
        )

    def test_landmark_collection_add_pop(self):
        """Test adding and removing from landmark collection."""
        lc = LandmarkCollection(node_inf=self.node)
        self.assertTrue(
            lc.is_empty(),
            'LandmarkCollection is_empty gives incorrect result.'
        )

        lc.add_landmark('test', 0)
        self.assertEqual(
            0, lc.get_landmark(0).get_id(),
            "LandmarkCollection add_landmark didn't work."
        )

        lc.remove_landmark(0)
        self.assertTrue(
            lc.is_empty(),
            "LandmarkCollection remove_landmark didn't work."
        )

    # def test_set_and_pub_tfs(self):
    #     """Test setting, publishing, and transforming."""
    #     # self.tf_broadcaster.sendTransform(TransformStamped(
    #     #     header=Header(
    #     #         stamp=Time().to_msg(),
    #     #         frame_id='map'
    #     #     ),
    #     #     child_frame_id='landmarks',
    #     #     transform=Transform(),
    #     # ))

    #     lc = LandmarkCollection(node_inf=self.node)
    #     lc.add_landmark('test_1', 1)

    #     tf_1 = Transform()
    #     tf_1.translation.x = 0.0
    #     tf_1.translation.y = 0.0
    #     tf_1.translation.z = 0.0
    #     tf_1.rotation.x = 0.0
    #     tf_1.rotation.y = 0.0
    #     tf_1.rotation.z = 0.0
    #     tf_1.rotation.w = 1.0

    #     lc.get_landmark(1).set_cam_frame_id('landmarks')
    #     lc.get_landmark(1).set_tf_wrt_cam(tf_1)

    #     future_wait_for_tf: Future = self.tf_buffer.wait_for_transform_async(
    #         target_frame='map',
    #         source_frame='landmarks',
    #         time=Time(),
    #     )

    #     rclpy.spin_until_future_complete(
    #         node=self.node,
    #         future=future_wait_for_tf,
    #         timeout_sec=5.0
    #     )

    #     lc.get_landmark(1).update_tfs(
    #         parent_old='landmarks',
    #         parent_new='map'
    #     )

    #     lm_1 = lc.get_landmark(1)

    #     self.assertEqual(
    #         lm_1.get_tf_wrt_cam().transform.translation.x,
    #         tf_1.translation.x,
    #         'Landmark get_tf_wrt_cam x is wrong.'
    #     )
    #     self.assertEqual(
    #         lm_1.get_tf_wrt_cam().transform.translation.y,
    #         tf_1.translation.y,
    #         'Landmark get_tf_wrt_cam y is wrong.'
    #     )
    #     self.assertEqual(
    #         lm_1.get_tf_wrt_cam().transform.translation.z,
    #         tf_1.translation.z,
    #         'Landmark get_tf_wrt_cam z is wrong.'
    #     )
    #     self.assertEqual(
    #         lm_1.get_tf_wrt_cam().transform.rotation.x,
    #         tf_1.rotation.x,
    #         'Landmark get_tf_wrt_cam qx is wrong.'
    #     )
    #     self.assertEqual(
    #         lm_1.get_tf_wrt_cam().transform.rotation.y,
    #         tf_1.rotation.y,
    #         'Landmark get_tf_wrt_cam qy is wrong.'
    #     )
    #     self.assertEqual(
    #         lm_1.get_tf_wrt_cam().transform.rotation.z,
    #         tf_1.rotation.z,
    #         'Landmark get_tf_wrt_cam qz is wrong.'
    #     )
    #     self.assertEqual(
    #         lm_1.get_tf_wrt_cam().transform.rotation.w,
    #         tf_1.rotation.w,
    #         'Landmark get_tf_wrt_cam qw is wrong.'
    #     )
    #     self.assertEqual(
    #         lm_1.get_tf_wrt_cam().header.frame_id,
    #         'landmarks',
    #         'Landmark get_tf_wrt_cam frame_id is wrong'
    #     )
    #     self.assertEqual(
    #         lm_1.get_tf_wrt_cam().child_frame_id,
    #         'test_1',
    #         'Landmark get_tf_wrt_cam child_frame_id is wrong'
    #     )

    #     tf_2 = Transform()
    #     tf_2.translation.x = 1.0
    #     tf_2.translation.y = 0.0
    #     tf_2.translation.z = 0.0
    #     tf_2.rotation.x = 0.0
    #     tf_2.rotation.y = 0.0
    #     tf_2.rotation.z = 0.0
    #     tf_2.rotation.w = 1.0

    #     lc.add_landmark('test_2', 2)
    #     lc.get_landmark(2).set_cam_frame_id('landmarks')
    #     lc.get_landmark(2).set_tf_wrt_cam(tf_2)

    #     lc.get_landmark(2).update_tfs(
    #         parent_old='landmarks',
    #         parent_new='map'
    #     )

    #     self.assertTrue(
    #         lc.get_landmark(2).tf_set_,
    #         'Landmark tf_set_ was not set to True correctly.'
    #     )

    #     lc.pub_tfs(tag_ids=[2])

    #     time.sleep(1.0)

    #     tf2_ros.TransformListener(self.tf_buffer, node=self.node)
    #     tf_map_to_test_2 = self.tf_buffer.lookup_transform(
    #         'map',
    #         'test_2',
    #         Time(0)
    #     )

    #     pos_x = tf_map_to_test_2.transform.translation.x == tf_2.translation.x
    #     pos_y = tf_map_to_test_2.transform.translation.y == tf_2.translation.y
    #     pos_z = tf_map_to_test_2.transform.translation.z == tf_2.translation.z

    #     rot_x = tf_map_to_test_2.transform.rotation.x == tf_2.rotation.x
    #     rot_y = tf_map_to_test_2.transform.rotation.y == tf_2.rotation.y
    #     rot_z = tf_map_to_test_2.transform.rotation.z == tf_2.rotation.z
    #     rot_w = tf_map_to_test_2.transform.rotation.w == tf_2.rotation.w

    #     self.assertTrue(
    #         (pos_x and pos_y and pos_z),
    #         'Landmark transform translation is incorrect.'
    #     )
    #     self.assertTrue(
    #         (rot_x and rot_y and rot_z and rot_w),
    #         'Landmark transform rotation is incorrect.'
    #     )
