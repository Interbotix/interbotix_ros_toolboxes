import yaml
import copy
import rospy
import tf2_ros
# from enum import Enum
import tf2_geometry_msgs
from tf.transformations import *
from interbotix_common_modules import geometry as g
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TransformStamped, Quaternion, Pose, PoseStamped, Pose2D


class Landmark(object):
    """A module to store data about landmarks, calculate TF data, and 
    publish that information to the static TF tree.

    :param label: name of this tracked landmark
    :param landmark_ns: namespace where the ROS parameters needed by the 
        module are located
    """
    def __init__(self, label=None, id_num=None, landmark_ns="landmarks"):

        # tf buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.label_         = label
        self.id_            = id_num
        self.landmark_ns    = landmark_ns

        # transforms
        self.tf_wrt_cam = TransformStamped()
        self.tf_wrt_cam.child_frame_id = self.label_
        self.tf_wrt_map = TransformStamped()
        self.tf_wrt_map.child_frame_id = self.label_
        self.goto_ = Pose2D()

        # flags
        self.tf_set_    = False
        self.goto_set_  = False

        self._init_goto_markers()
        # print("Initialized Landmark \"{}\"".format(label))

    def _init_goto_markers(self):
        """initializes goto marker paramers"""
        # spherical goto marker
        self.goto_marker_sphere = Marker()
        self.goto_marker_sphere.id = 0
        self.goto_marker_sphere.header.frame_id = self.landmark_ns
        self.goto_marker_sphere.header.stamp = rospy.Time.now()
        self.goto_marker_sphere.ns = self.get_label()
        self.goto_marker_sphere.type = Marker.SPHERE
        self.goto_marker_sphere.action = Marker.ADD
        self.goto_marker_sphere.scale.x = 0.025
        self.goto_marker_sphere.scale.y = 0.025
        self.goto_marker_sphere.scale.z = 0.025
        self.goto_marker_sphere.color.a = 1
        self.goto_marker_sphere.color.r = 1
        self.goto_marker_sphere.color.g = 0
        self.goto_marker_sphere.color.b = 0
        self.goto_marker_sphere.pose.position.z = 0

        # text goto marker
        self.goto_marker_text = Marker()
        self.goto_marker_text.id = 1
        self.goto_marker_text.header.frame_id = self.landmark_ns
        self.goto_marker_text.header.stamp = rospy.Time.now()
        self.goto_marker_text.ns = self.get_label()
        self.goto_marker_text.type = Marker.TEXT_VIEW_FACING
        self.goto_marker_text.text = "{}_goto".format(self.get_label())
        self.goto_marker_text.scale.x = 0.05
        self.goto_marker_text.scale.y = 0.05
        self.goto_marker_text.scale.z = 0.05
        self.goto_marker_text.color.a = 1
        self.goto_marker_text.color.r = 1
        self.goto_marker_text.color.g = 1
        self.goto_marker_text.color.b = 1
        self.goto_marker_text.pose.position.z = 0.05

    def get_id(self):
        """getter for id attribute

        :return: id
        "rtype: float
        """
        return self.id_

    def set_id(self, id_num):
        """setting for id attribute

        :param id_num: new id number for this landmark
        "type id_num: int
        """
        self.id_ = id_num

    def transform_to_new_frame(self, parent_old, parent_new):
        """transforms frame from one fixed pose to another

        :param parent_old: current fixed frame
        :type parent_old: Pose
        :param parent_new: desired fixed frame
        :type parent_new: Pose
        :return: pose in new fixed frame
        :rtype: PoseStamped
        """
        # build old pose
        pose_old = tf2_geometry_msgs.PoseStamped()
        pose_old.pose.position = self.tf_wrt_cam.transform.translation
        pose_old.pose.orientation = self.tf_wrt_cam.transform.rotation
        pose_old.header.frame_id = parent_old
        pose_old.header.stamp = rospy.Time.now()
        try:
            # transform pose to frame parent_new
            pose_new = self.tf_buffer.transform(
                pose_old, parent_new, rospy.Duration(1))
            # set/update tf wrt map
            return pose_new
            # self.set_tf_wrt_map(pose_new) # TODO
        except (tf2_ros.LookupException, 
                tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException):
            raise

    def get_label(self):
        """getter for label attribute

        :return: label
        "rtype: string
        """
        return self.label_

    def set_label(self, label):
        """setting for label attribute

        :param label: new label for this landmark
        "type label: string
        """
        self.label_ = label

    def get_x(self):
        """getter for landmark x position

        :return: landmark x position
        :rtype: float
        """
        if self.tf_set_:
            return self.tf_wrt_map.transform.translation.x
        else:
            return 0.0

    def get_y(self):
        """getter for landmark y position

        :return: landmark y position
        :rtype: float
        """
        if self.tf_set_:
            return self.tf_wrt_map.transform.translation.y
        else:
            return 0.0

    def get_theta(self):
        """getter for landmark yaw rotation

        :return: landmark yaw rotation
        :rtype: float
        """
        if self.tf_set_:
            q = self.tf_wrt_map.transform.rotation
            return euler_from_quaternion(
                [q.x, q.y, q.z, q.w])[2]
        else:
            return 0.0

    def set_tf_wrt_cam(self, pose):
        """setter for transform wrt camera frame

        :param pose: translation and orientation from cam to landmark
        :pose type: Pose or PoseStamped
        """
        if isinstance(pose, Pose):
            self.tf_wrt_cam.transform.translation = pose.position
            self.tf_wrt_cam.transform.rotation = pose.orientation
        elif isinstance(pose, PoseStamped):
            self.tf_wrt_cam.transform.translation = pose.pose.position
            self.tf_wrt_cam.transform.rotation = pose.pose.orientation
        else:
            raise TypeError

    def set_cam_frame_id(self, frame_id):
        """setter for transform wrt camera frame frame id"""
        self.tf_wrt_cam.header.frame_id = frame_id

    def get_tf_wrt_cam(self):
        """getter for transform wtf camera

        :return: pose transform with respect to camera
        :rtype: TransformStamped
        """
        return self.tf_wrt_cam

    def set_tf_wrt_map(self, pose, parent_new):
        """setter for transform wrt camera frame

        :param pose: translation and orientation from cam to landmark
        :pose type: Pose or PoseStamped
        :param parent_new: name of map frame
        :type parent_new: str
        """
        if isinstance(pose, Pose):
            self.tf_wrt_map.transform.translation = pose.position
            self.tf_wrt_map.transform.rotation = pose.orientation
        elif isinstance(pose, PoseStamped):
            self.tf_wrt_map.transform.translation = pose.pose.position
            self.tf_wrt_map.transform.rotation = pose.pose.orientation
        self.tf_wrt_map.header.stamp = rospy.Time.now()
        self.tf_wrt_map.header.frame_id = parent_new

    def get_tf_wrt_map(self):
        """setter for transform wrt fixed frame

        :return: transform with respect to fixed frame
        :rtype: TransformStamped
        """
        return self.tf_wrt_map

    def update_tfs(self, parent_old, parent_new):
        """updates the transformation to fixed frame"""
        pose = self.transform_to_new_frame(parent_old, parent_new)
        self.set_tf_wrt_map(pose, parent_new)
        self.tf_set_ = True

    def set_goto(self, p2d=None, x=None, y=None, theta=None):
        """sets the goto pose"""
        if isinstance(p2d, Pose2D) and (x is None):
            self.goto_ = p2d
            return
        elif (p2d is None) and (x is not None):
            p2d = Pose2D()
            p2d.x = x
            p2d.y = y
            p2d.theta = theta
            self.goto_ = p2d

    def get_goto(self):
        """gets the goto pose
        
        :return: goto
        :rtype: Pose2D
        """
        return self.goto_

    def get_mb_goal(self):
        """gets the x, y, theta goal for move_base action

        :return: list containing x, y, and theta for move base goal
        :rype: list of floats
        """
        mb_x = self.get_x() + self.get_goto().x
        mb_y = self.get_y() + self.get_goto().y
        mb_theta = self.get_theta() + self.get_goto().theta
        goal = [mb_x, mb_y, mb_theta]
        return goal

    def __eq__(self, other):
        if isinstance(other, str):      # match label if string
            return self.label_ == other
        elif isinstance(other, int):    # match id if int
            return self.id_ == other

        if isinstance(other, Landmark): # if Landmark, check both
            return (self.id_ == other.id_ or self.label_ == other.label_)

    def __repr__(self):
        """Define repr
        
        Prints in the form of:
        ---
        label: {}
        id: {}
        tf_self: {}
        goto: {


        }
        tf: {


        }
        ---
        """
        tf = self.get_tf_wrt_map() if self.tf_set_ else self.get_tf_wrt_cam()
        r = "---\nlabel: {}\nid: {}\ntf_set: {}\ngoto:\n{}\ntf:\n{}\n---".format(
            self.label_, self.id_, self.tf_set_, self.goto_, tf)
        return r

class LandmarkCollection(object):
    """Class to interact with Landmark data structure
    
    Contains useful helper functions.
    """
    def __init__(self, landmarks=dict(), obs_frame=None, fixed_frame=None, 
                ros_on=False, init_node=False):
        self.data = landmarks

        if init_node:
            rospy.init_node("landmark_collection")

        # if we will use ROS, get params and set up pubs/subs
        if ros_on:
            self.ROS = True
            
            self.obs_frame = rospy.get_param(
                "obs_frame", 
                obs_frame)
            self.fixed_frame = rospy.get_param(
                "fixed_frame", 
                fixed_frame)
            
            self.static_tf_pub = rospy.Publisher(
                'static_transforms', 
                TransformStamped,
                queue_size=10,
                latch=True)
            
            self.goto_marker_pub = rospy.Publisher(
                'goto_viz', 
                MarkerArray, 
                queue_size=10,
                latch=True)
        else:
            self.ROS=False
            self.obs_frame = obs_frame
            self.fixed_frame = fixed_frame

    def get_landmark(self, id_num):
        """returns the specified landmark
        
        :return: landmark with specified id
        :rtype: Landmark
        """
        if id_num == None:
            return self.data.values()
        elif type(id_num) == list:
            return [self.data[x] for x in id_num]
        else:
            return self.data[id_num]

    def add_landmark(self, label, id_num):
        """adds landmark to data dictionary
        
        :param label: label for added landmark to add
        :type label: str
        :param id_num: tag id for added landmark and dictionary key
        :type id_num: int
        """
        self.data.update(
            {id_num: Landmark(
                label=label, 
                id_num=id_num)})

    def remove_landmark(self, id_num):
        """removes landmark from data dictionary
        
        :param id_num: id to remove
        :type id_num: int
        """
        self.data.pop(id_num)

    def get_valid_tags(self):
        """returns the ids of the tags of any landmark in the Collection

        :return: list of ids corresponsing to landmarks in the Collection
        :rtype: list of ints
        """
        self.update_valid_tags()
        return self.valid_tags

    def update_valid_tags(self):
        """updates the list of valid tags"""
        self.valid_tags = set([l.get_id() for l in self.data.values()])
        
    def save(self, filepath, ids=None):
        """save landmarks to a specified yaml file

        :param filepath: path to save structured landmark data to
        :type filepath: str
        :return: `True` if saved successfully, `False` otherwise
        :rtype: bool
        """
        if self.is_empty():
            return True

        lm_yaml = {}
        for lm in self.get_landmark(ids):
            tf_map = lm.get_tf_wrt_map()
            lm_dict = {}
            lm_dict["id"] = lm.id_
            lm_dict["label"] = lm.label_
            lm_dict["set"] = lm.tf_set_
            lm_dict["goto"] = {}
            lm_dict["goto"]["x"] = lm.goto_.x
            lm_dict["goto"]["y"] = lm.goto_.y
            lm_dict["goto"]["theta"] = lm.goto_.theta
            if lm.tf_set_:
                lm_dict["tf"] = {}
                lm_dict["tf"]["frame_id"] = tf_map.header.frame_id
                lm_dict["tf"]["child_frame_id"] = tf_map.child_frame_id
                lm_dict["tf"]["x"] = float(tf_map.transform.translation.x)
                lm_dict["tf"]["y"] = float(tf_map.transform.translation.y)
                lm_dict["tf"]["z"] = float(tf_map.transform.translation.z)
                lm_dict["tf"]["qx"] = float(tf_map.transform.rotation.x)
                lm_dict["tf"]["qy"] = float(tf_map.transform.rotation.y)
                lm_dict["tf"]["qz"] = float(tf_map.transform.rotation.z)
                lm_dict["tf"]["qw"] = float(tf_map.transform.rotation.w)
            lm_yaml[lm.id_] = lm_dict

        with open(filepath, "w") as yamlfile:
            yaml.dump(lm_yaml, yamlfile, default_flow_style=False)
        rospy.loginfo("Saved landmarks to {}".format(filepath))

        return True

    def load(self, filepath):
        """loads landmarks from a specified yaml file

        :param filepath: path to yaml file containing structured landmark data
        :type filepath: str
        :return: `True` if loaded successfully, `False` otherwise
        :rtype: bool
        """
        try:
            with open(filepath, "r") as yamlfile:
                lm_dict = yaml.safe_load(yamlfile)

            if lm_dict == None:
                return

            for key in lm_dict.keys():

                self.add_landmark(
                    label=lm_dict[key]["label"],
                    id_num=key)

                # get transform from configs
                if "tf" in lm_dict[key].keys():
                    self.data[key].tf_wrt_map = TransformStamped()
                    self.data[key].tf_wrt_map.header.stamp = rospy.Time.now()
                    if lm_dict[key]["tf"]["frame_id"] == '':
                        self.data[key].tf_wrt_map.header.frame_id       = self.obs_frame
                    else:
                        self.data[key].tf_wrt_map.header.frame_id       = lm_dict[key]["tf"]["frame_id"]
                    self.data[key].tf_wrt_map.child_frame_id            = lm_dict[key]["tf"]["child_frame_id"]
                    self.data[key].tf_wrt_map.transform.translation.x   = lm_dict[key]["tf"]["x"]
                    self.data[key].tf_wrt_map.transform.translation.y   = lm_dict[key]["tf"]["y"]
                    self.data[key].tf_wrt_map.transform.translation.z   = lm_dict[key]["tf"]["z"]
                    self.data[key].tf_wrt_map.transform.rotation.x      = lm_dict[key]["tf"]["qx"]
                    self.data[key].tf_wrt_map.transform.rotation.y      = lm_dict[key]["tf"]["qy"]
                    self.data[key].tf_wrt_map.transform.rotation.z      = lm_dict[key]["tf"]["qz"]
                    self.data[key].tf_wrt_map.transform.rotation.w      = lm_dict[key]["tf"]["qw"]
                    self.data[key].tf_set_ = True
                else:
                    self.data[key].tf_wrt_cam.header.frame_id = self.obs_frame
                    self.data[key].tf_wrt_cam.child_frame_id = self.data[key].get_label()
                    self.data[key].tf_set_ = False
                
                # get goto values from configs
                if "goto" in lm_dict[key].keys():
                    goto = Pose2D()
                    goto.x = lm_dict[key]["goto"]["x"]
                    goto.y = lm_dict[key]["goto"]["y"]
                    goto.theta = lm_dict[key]["goto"]["theta"]
                else:
                    goto = Pose2D()

                self.data[key].set_goto(goto)

                if self.ROS:
                    self.pub_tfs()
                    self.pub_gotos()

            self.update_valid_tags()
            return True
        
        except IOError:
            rospy.logwarn(
                "File at {} does not exist yet. No landmarks loaded.".format(
                    filepath))
            return False

    def is_empty(self):
        """checks if landmarks is empty

        :return: `True` if landmarks is empty, `False` otherwise
        :rtype: bool
        """
        return self.data == {}

    def pub_tfs(self, tag_ids=None):
        """publishes TFs to static transforms
        
        :param tag_ids: list of tags to publish
        :type tag_ids: list of ints
        :return: `True`
        :rtype: bool
        """
        if self.is_empty():
            return True
        
        if tag_ids == None:
            # when no tags are specified, pubs all
            for lm in self.data.values():
                if lm.tf_set_:
                    self.static_tf_pub.publish(
                        lm.get_tf_wrt_map())
            return True
        else:
            # when list of tags is specified, pubs tags in list
            for lm in [self.data[x] for x in tag_ids]:
                if lm.tf_set_:
                    self.static_tf_pub.publish(
                        lm.get_tf_wrt_map())
                return True

    def update_goto_markers(self):
        """updates goto markers"""
        for lm in self.data.values():
            g = lm.get_goto()
            lm.goto_marker_sphere.pose.position.x = lm.get_x() + g.x # TODO tf in tag TF
            lm.goto_marker_sphere.pose.position.y = lm.get_y() + g.y
            lm.goto_marker_sphere.pose.position.z = 0.0
            lm.goto_marker_sphere.pose.orientation.x = 0.0
            lm.goto_marker_sphere.pose.orientation.y = 0.0
            lm.goto_marker_sphere.pose.orientation.z = 0.0
            lm.goto_marker_sphere.pose.orientation.w = 1.0
            lm.goto_marker_sphere.header.stamp = rospy.Time.now()

            lm.goto_marker_text.pose.position.x = lm.get_x() # TODO tf in tag TF
            lm.goto_marker_text.pose.position.y = lm.get_y()
            lm.goto_marker_text.pose.position.z = 0.1
            lm.goto_marker_text.pose.orientation.x = 0.0
            lm.goto_marker_text.pose.orientation.y = 0.0
            lm.goto_marker_text.pose.orientation.z = 0.0
            lm.goto_marker_text.pose.orientation.w = 1.0
            lm.goto_marker_text.header.stamp = rospy.Time.now()

    def pub_gotos(self):
        """publishes goto markers"""
        if not self.ROS:
            rospy.logwarn(
                "Tried to publish goto marker but node is not active.")
            return
        else:
            self.update_goto_markers()
            msg = MarkerArray()
            for lm in self.data.values():
                msg.markers.append(lm.goto_marker_sphere)
                msg.markers.append(lm.goto_marker_text)
                self.goto_marker_pub.publish(msg)

    def __repr__(self):
        ls = []
        for lm in self.data.values():
            ls.append(lm)
        return str(ls)
        
    def __len__(self):
        return len(self.data)
