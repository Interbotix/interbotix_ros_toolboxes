import os
import sys
import rospy
import signal
import tf2_ros
import numpy as np
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, Quaternion, Pose
from interbotix_perception_modules.apriltag import InterbotixAprilTagInterface
from interbotix_common_modules import angle_manipulation as ang
from interbotix_common_modules import geometry as g


class InterbotixLandmarkInterface(object):
    """A module to store data about landmarks. Using the AprilTag Interface, 
    this class is able to store the Tag associated with the landmark, 
    search for the Tag in the map, calculate TF data, and publish that 
    information to the static TF tree.

    :param label: name of this tracked landmark
    :param landmark_ns: namespace where the ROS parameters needed by the 
        module are located
    :param apriltag_ns: namespace where the ROS parameters needed by the 
        InterbotixAprilTagInterface module are located
    :param init_node: whether or not the module should initalize a ROS node; 
        set to False if a node was already initalized somwhere else
    :param path_to_image: full path to an image containing the landmark's AR 
        tag [unused]
    :param timer_callback_period: period at which the apriltag's get_pose function 
        should be called 
    """
    pose_wrt_map = Pose()
    def __init__(self, label="landmark", landmark_ns="landmarks", 
                 apriltag_ns="apriltag", init_node=False,
                 path_to_image=None, timer_callback_period=3):
        
        if (init_node):
            rospy.init_node(landmarktag_ns.strip("/") + "_interface")

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.label = label
        self.set = False

        # get parameters
        self.landmark_frame = rospy.get_param(
            "/landmark_manager/landmark_tag_frame", self.label)
        self.obs_frame = rospy.get_param(
            "/landmark_manager/obs_frame", "camera_color_frame")
        self.fixed_frame = rospy.get_param(
            "/landmark_manager/fixed_frame", "landmarks")

        # build the header for this landmark's TF
        self.trans = TransformStamped()
        self.trans.header.frame_id = self.fixed_frame
        self.trans.child_frame_id = self.landmark_frame
        
        self.apriltag = InterbotixAprilTagInterface(
            apriltag_ns=apriltag_ns, 
            init_node=False)
        self.timer = rospy.Timer(
            rospy.Duration(timer_callback_period), 
            self.timer_callback)
        rospy.sleep(rospy.Duration(5))
        print("Initialized InterbotixLandmarkInterface!\n")

    def transform_to_new_frame(self, child, parent_old, parent_new):
        """transforms frame from one fixed pose to another

        :param child: pose to be transformed
        :type child: Pose
        :param parent_old: current fixed frame
        :type parent_old: Pose
        :param parent_new: desired fixed frame
        :type parent_new: Pose
        :returns: pose in new fixed frame
        :rtype: tf2_geometry_msgs.PoseStamped
        """
        # build old pose
        pose_old = tf2_geometry_msgs.PoseStamped()
        pose_old.pose = child
        pose_old.header.frame_id = parent_old
        pose_old.header.stamp = rospy.Time.now()
        try:
            # transform pose to frame parent_new
            pose_new = self.tf_buffer.transform(
                pose_old, parent_new, rospy.Duration(1))
            return pose_new.pose
        except (tf2_ros.LookupException, 
                tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException):
            raise

    def timer_callback(self, timer):
        """timed callback that gets the landmark's pose if it is view of the
            camera, transforms the pose to the world's fixed frame, and 
            publishes the TF to the static transform publisher
        """
        # get pose with respect to camera frame
        pose_wrt_cam = self.apriltag.find_pose(
            ar_tag_name=self.label, 
            publish_tf=False)
        # if we get a valid pose (if given quaternion is valid)...
        if g.quaternion_is_valid(pose_wrt_cam.orientation):
            # transform the pose to the new frame
            self.pose_wrt_map = self.transform_to_new_frame(
                child=pose_wrt_cam, 
                parent_old=self.obs_frame, 
                parent_new=self.fixed_frame)
            
            # build and publish the new TF for the pose in the fixed frame
            self.trans.header.stamp = rospy.Time.now()
            self.trans.transform.translation = self.pose_wrt_map.position
            self.trans.transform.rotation = self.pose_wrt_map.orientation
            self.apriltag.pub_transforms.publish(self.trans)
            self.set = True

    def get_label(self):
        """getter for label attribute
        :returns: label
        "rtype: string
        """
        return self.label

    def set_label(self, label):
        """setting for label attribute
        :param label: new label for this landmark
        "type label: string
        """
        self.label = label

    def get_x(self):
        """getter for landmark x position
        :returns: landmark x position
        :rtype: float
        """
        if self.set:
            return self.trans.transform.translation.x
        else:
            rospy.logwarn(
                "Tried to get x position of " + self.label + 
                " but it is not set yet. Returning 0.0.")
            return 0.0

    def get_y(self):
        """getter for landmark y position
        :returns: landmark y position
        :rtype: float
        """
        if self.set:
            return self.trans.transform.translation.y
        else:
            rospy.logwarn(
                "Tried to get y position of " + self.label + 
                " but it is not set yet. Returning 0.0.")
            return 0.0