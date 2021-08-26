#!/usr/bin/env python

import sys
import unittest
from math import cos, pi

import rospy
import rosunit
import tf2_ros

from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Pose, Pose2D, TransformStamped, Quaternion
from interbotix_landmark_modules.landmark import Landmark, LandmarkCollection

## Integration tests for the landmark and landmark collection modules

PKG = 'interbotix_landmark_modules'
NAME = 'test_landmark'

class LandmarkTest(unittest.TestCase):
    def test_label(self):
        """test landmark.get_label()"""
        label = "test"
        lm = Landmark(label, 0)
        self.assertEquals(
            label, 
            lm.get_label(), 
            "Landmark label is incorrect.")

    def test_id(self):
        """test landmark.get_id()"""
        id_num = 1
        lm = Landmark("test", 0)
        lm.set_id(id_num)
        self.assertEquals(
            id_num, lm.get_id(), 
        "Landmark ID is incorrect.")

    def test_landmark_collection_add_pop(self):
        """test adding and removing from landmark collection"""
        
        lc = LandmarkCollection()
        self.assertTrue(
            lc.is_empty(), 
            "LandmarkCollection is_empty gives incorrect result.")

        lc.add_landmark("test", 0)
        self.assertEquals(
            0, lc.get_landmark(0).get_id(), 
            "LandmarkCollection add_landmark didn't work.")

        lc.remove_landmark(0)
        self.assertTrue(
            lc.is_empty(), 
            "LandmarkCollection remove_landmark didn't work.")

    def test_set_and_pub_tfs(self):
        """test setting, publishing, and transforming"""

        lc = LandmarkCollection(ros_on=True)
        lc.add_landmark("test_1", 1)

        pose_1 = Pose()
        pose_1.position.x = 0.0
        pose_1.position.y = 0.0
        pose_1.position.z = 0.0
        pose_1.orientation.x = 0.0
        pose_1.orientation.y = 0.0
        pose_1.orientation.z = 0.0
        pose_1.orientation.w = 1.0

        lc.data[1].set_cam_frame_id("landmarks")
        lc.data[1].set_tf_wrt_cam(pose_1)
        lc.data[1].update_tfs(
            parent_old="landmarks",
            parent_new="map")

        self.assertEquals(
            lc.data[1].get_tf_wrt_cam().transform.translation.x, pose_1.position.x, 
            "Landmark get_tf_wrt_cam x is wrong.")
        self.assertEquals(
            lc.data[1].get_tf_wrt_cam().transform.translation.y, pose_1.position.y, 
            "Landmark get_tf_wrt_cam y is wrong.")
        self.assertEquals(
            lc.data[1].get_tf_wrt_cam().transform.translation.z, pose_1.position.z, 
            "Landmark get_tf_wrt_cam z is wrong.")
        self.assertEquals(
            lc.data[1].get_tf_wrt_cam().transform.rotation.x, pose_1.orientation.x, 
            "Landmark get_tf_wrt_cam qx is wrong.")
        self.assertEquals(
            lc.data[1].get_tf_wrt_cam().transform.rotation.y, pose_1.orientation.y, 
            "Landmark get_tf_wrt_cam qy is wrong.")
        self.assertEquals(
            lc.data[1].get_tf_wrt_cam().transform.rotation.z, pose_1.orientation.z, 
            "Landmark get_tf_wrt_cam qz is wrong.")
        self.assertEquals(
            lc.data[1].get_tf_wrt_cam().transform.rotation.w, pose_1.orientation.w, 
            "Landmark get_tf_wrt_cam qw is wrong.")
        self.assertEquals(
            lc.data[1].get_tf_wrt_cam().header.frame_id, "landmarks", 
            "Landmark get_tf_wrt_cam frame_id is wrong")
        self.assertEquals(
            lc.data[1].get_tf_wrt_cam().child_frame_id, "test_1", 
            "Landmark get_tf_wrt_cam child_frame_id is wrong")

        pose_2 = Pose()
        pose_2.position.x = 1.0
        pose_2.position.y = 0.0
        pose_2.position.z = 0.0
        pose_2.orientation.x = 0.0
        pose_2.orientation.y = 0.0
        pose_2.orientation.z = 0.0
        pose_2.orientation.w = 1.0

        lc.add_landmark("test_2", 2)
        lc.data[2].set_cam_frame_id("landmarks")
        lc.data[2].set_tf_wrt_cam(pose_2)

        lc.data[2].update_tfs(
            parent_old="landmarks",
            parent_new="map")
        
        self.assertTrue(
            lc.data[2].tf_set_,
            "Landmark tf_set_ was not set to True correctly.")

        lc.pub_tfs(tag_ids=[2])

        rospy.sleep(rospy.Duration(1))

        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        tf_map_to_test_2 = tfBuffer.lookup_transform(
                                        "map", "test_2", rospy.Time(0))

        pos_x = tf_map_to_test_2.transform.translation.x == pose_2.position.x
        pos_y = tf_map_to_test_2.transform.translation.y == pose_2.position.y
        pos_z = tf_map_to_test_2.transform.translation.z == pose_2.position.z

        rot_x = tf_map_to_test_2.transform.rotation.x == pose_2.orientation.x
        rot_y = tf_map_to_test_2.transform.rotation.y == pose_2.orientation.y
        rot_z = tf_map_to_test_2.transform.rotation.z == pose_2.orientation.z
        rot_w = tf_map_to_test_2.transform.rotation.w == pose_2.orientation.w

        self.assertTrue(
            (pos_x and pos_y and pos_z), 
            "Landmark transform translation is incorrect.")
        self.assertTrue(
            (rot_x and rot_y and rot_z and rot_w), 
            "Landmark transform rotation is incorrect.")


if __name__ == "__main__":
    rospy.init_node("landmark_test")
    rosunit.unitrun(
        package=PKG, 
        test_name=NAME,
        test=LandmarkTest)
