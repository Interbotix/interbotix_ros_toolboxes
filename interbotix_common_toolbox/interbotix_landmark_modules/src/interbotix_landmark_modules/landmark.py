import yaml
import rospy
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import *
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TransformStamped, Pose, PoseStamped


class Landmark(object):
    """A module used to store, configure, and publish data about landmarks."""

    def __init__(self, label=None, id_num=None, landmark_ns="landmarks"):
        """
        Construct new landmark.

        :param label: name of this tracked landmark
        :param id: id of this landmark
        :param landmark_ns: namespace where the ROS parameters needed by the module are located
        """
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
        self.mounted_offset = 0.0

        # flags
        self.tf_set_    = False
        self.mounted_   = False

        self._init_markers()

    def get_id(self):
        """
        Get id.

        :returns: id
        """
        return self.id_

    def set_id(self, id_num):
        """
        Set id.

        :param id_num: new id number for this landmark
        """
        self.id_ = id_num

    def is_set(self):
        """
        Return whether or not the landmark has been set.

        :returns: `True` if the landmark is set, `False` otherwise
        """
        return self.tf_set_

    def transform_to_new_frame(self, parent_old, parent_new):
        """
        Transform frame from one fixed pose to another.

        :param parent_old: current fixed frame
        :param parent_new: desired fixed frame
        :returns: pose in new fixed frame
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
        """
        Get label.

        :returns: label
        """
        return self.label_

    def set_label(self, label):
        """
        Set label.

        :param label: new label for this landmark
        """
        self.label_ = label

    def get_x(self):
        """
        Get landmark x position wrt map frame.

        If the landmark has not been set, defaults to 0.

        :returns: landmark x position wrt map frame
        """
        if self.is_set():
            return self.tf_wrt_map.transform.translation.x
        else:
            return 0.0

    def get_y(self):
        """
        Get landmark y position wrt map frame.

        If the landmark has not been set, defaults to 0.

        :returns: landmark y position wrt map frame
        """
        if self.is_set():
            return self.tf_wrt_map.transform.translation.y
        else:
            return 0.0

    def get_theta(self):
        """
        Get landmark yaw rotation wrt map frame.

        If the landmark has not been set, defaults to 0.

        :returns: landmark yaw rotation wrt map frame
        """
        if self.is_set():
            q = self.tf_wrt_map.transform.rotation
            return euler_from_quaternion(
                [q.x, q.y, q.z, q.w], 'rxyz')[2]
        else:
            return 0.0

    def set_tf_wrt_cam(self, pose):
        """
        Set transform wrt camera frame.

        :param pose: translation and orientation from cam to landmark
        :raises: TypeError if pose is not the correct type
        """
        if isinstance(pose, Pose):
            self.tf_wrt_cam.transform.translation = pose.position
            self.tf_wrt_cam.transform.rotation = pose.orientation
        elif isinstance(pose, PoseStamped):
            self.tf_wrt_cam.transform.translation = pose.pose.position
            self.tf_wrt_cam.transform.rotation = pose.pose.orientation
        else:
            raise TypeError("Expected a Pose or PoseStamped.")

    def set_cam_frame_id(self, frame_id):
        """
        Set transform wrt camera frame frame id.

        :param frame_id: id of the frame
        """
        self.tf_wrt_cam.header.frame_id = frame_id

    def get_tf_wrt_cam(self):
        """
        Get transform wtf camera.

        :returns: pose transform with respect to camera
        """
        return self.tf_wrt_cam

    def set_tf_wrt_map(self, pose, parent_new):
        """
        Set transform wrt camera frame.

        :param pose: translation and orientation from cam to landmark
        :param parent_new: name of map frame
        :raises: TypeError if pose is not the correct type
        """
        if isinstance(pose, Pose):
            self.tf_wrt_map.transform.translation = pose.position
            self.tf_wrt_map.transform.rotation = pose.orientation
        elif isinstance(pose, PoseStamped):
            self.tf_wrt_map.transform.translation = pose.pose.position
            self.tf_wrt_map.transform.rotation = pose.pose.orientation
        else:
            raise TypeError("Expected a Pose or PoseStamped.")
        self.tf_wrt_map.header.stamp = rospy.Time.now()
        self.tf_wrt_map.header.frame_id = parent_new

    def get_tf_wrt_map(self):
        """
        Set transform wrt fixed frame.

        :returns: transform with respect to fixed frame
        """
        return self.tf_wrt_map

    def update_tfs(self, parent_old, parent_new):
        """
        Update the transformation to fixed frame.

        :param parent_old: Previous reference frame
        :param parent_new: New reference frame
        """
        pose = self.transform_to_new_frame(parent_old, parent_new)
        self.set_tf_wrt_map(pose, parent_new)
        self.tf_set_ = True

    def set_mounted(self, mounted):
        """
        Set whether or not the landmark is mounted to a vertical surface.

        :param: `True` if mounted, `False` otherwise
        """
        self.mounted_ = mounted

    def set_mounted_offset(self, offset=0.0):
        """
        Set the mounted offset distance.

        :param offset: The offset from the landmark, defaults to `0.0`
        """
        self.mounted_offset = offset

    def is_mounted(self):
        """
        Get whether or not the landmark is mounted to a vertical surface.

        :returns: `True` if the landmark is mounted, `False` otherwise
        """
        return self.mounted_

    def get_mounted_offset(self):
        """
        Get the mounted offset distance.

        :returns: mounted_offset
        """
        return self.mounted_offset

    def get_mb_goal(self):
        """
        Get for the [x, y, theta] goal for move_base action.

        :returns: list containing x, y, and theta for move base goal
        """
        if not self.is_mounted(): # we know offset is 0
            return [self.get_x(), self.get_y(), self.get_theta()]
        else:
            tf_goal = TransformStamped()
            tf_goal.header.stamp = rospy.Time(0)
            tf_goal.header.frame_id = self.get_label()
            tf_goal.child_frame_id = "{}_goal".format(self.get_label())

            # z-axis is normal to tag face, 'front of tag'
            tf_goal.transform.translation.z = self.get_mounted_offset()
            tf_goal.transform.rotation.w = 1.0 # ensure valid qt
            self.tf_buffer.set_transform(tf_goal, "default_authority")

            # get tf from map to goal frame [x,y,theta]
            tf_map_to_goal = self.tf_buffer.lookup_transform_core(
                self.landmark_ns,
                tf_goal.child_frame_id,
                rospy.Time(0))
            mb_goal = [
                tf_map_to_goal.transform.translation.x,
                tf_map_to_goal.transform.translation.y,
                euler_from_quaternion([
                    tf_map_to_goal.transform.rotation.x,
                    tf_map_to_goal.transform.rotation.y,
                    tf_map_to_goal.transform.rotation.z,
                    tf_map_to_goal.transform.rotation.w
                ])[2]
            ]
            return mb_goal

    def _init_markers(self):
        """Initialize constant marker paramers."""
        # spherical marker (location)
        self.marker_sphere = Marker()
        self.marker_sphere.id = 0
        self.marker_sphere.header.frame_id = self.landmark_ns
        self.marker_sphere.header.stamp = rospy.Time(0)
        self.marker_sphere.ns = self.get_label()
        self.marker_sphere.type = Marker.SPHERE
        self.marker_sphere.action = Marker.ADD
        self.marker_sphere.scale.x = 0.025
        self.marker_sphere.scale.y = 0.025
        self.marker_sphere.scale.z = 0.025
        self.marker_sphere.color.a = 1
        self.marker_sphere.color.r = 1
        self.marker_sphere.color.g = 0
        self.marker_sphere.color.b = 0
        self.marker_sphere.pose.position.z = 0

        # text marker (label)
        self.marker_text = Marker()
        self.marker_text.id = 1
        self.marker_text.header.frame_id = self.landmark_ns
        self.marker_text.header.stamp = rospy.Time(0)
        self.marker_text.ns = self.get_label()
        self.marker_text.type = Marker.TEXT_VIEW_FACING
        self.marker_text.text = "{}_goal".format(self.get_label())
        self.marker_text.scale.x = 0.05
        self.marker_text.scale.y = 0.05
        self.marker_text.scale.z = 0.05
        self.marker_text.color.a = 1
        self.marker_text.color.r = 1
        self.marker_text.color.g = 1
        self.marker_text.color.b = 1
        self.marker_text.pose.position.z = 0.05

    def __eq__(self, other):
        if isinstance(other, str):      # match label if string
            return self.label_ == other
        elif isinstance(other, int):    # match id if int
            return self.id_ == other

        if isinstance(other, Landmark): # if Landmark, check both
            return (self.id_ == other.id_ or self.label_ == other.label_)

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
        tf = self.get_tf_wrt_map() if self.is_set() else self.get_tf_wrt_cam()
        r = "---\nlabel: {}\nid: {}\ntf_set: {}\nmounted: {}\nmounted_offset: {}\ntf:\n{}\n---".format(
            self.label_, self.id_, self.is_set(), self.is_mounted(), self.mounted_offset, tf)
        return r

class LandmarkCollection(object):
    """
    Class to interact with Landmark data structure.

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

            self.marker_pub = rospy.Publisher(
                'marker_viz',
                MarkerArray,
                queue_size=10,
                latch=True)

        else:
            self.ROS=False
            self.obs_frame = obs_frame
            self.fixed_frame = fixed_frame

    def get_landmark(self, id_num):
        """
        Return the specified landmark.

        :returns: landmark with specified id
        """
        if id_num == None:
            return self.data.values()
        elif type(id_num) == list:
            return [self.data[x] for x in id_num]
        else:
            return self.data[id_num]

    def add_landmark(self, label, id_num):
        """
        Add landmark to data dictionary.

        :param label: label for added landmark to add
        :param id_num: tag id for added landmark and dictionary key
        """
        self.data.update({
            id_num: Landmark(label=label, id_num=id_num)
        })

    def remove_landmark(self, id_num):
        """
        Remove landmark from data dictionary.

        :param id_num: id of landmark to remove
        """
        self.data.pop(id_num)

    def get_valid_tags(self):
        """
        Return the ids of the tags of all valid landmark in the Collection.

        :returns: list of ids corresponsing to landmarks in the Collection
        """
        self.update_valid_tags()
        return self.valid_tags

    def update_valid_tags(self):
        """Update the list of valid tags."""
        self.valid_tags = [l.get_id() for l in self.data.values()]

    def get_set_tags(self):
        """
        Return the list of seen tags.

        :returns: list of ids corresponding to seen landmarks in the Collection
        """
        return [l.get_id() for l in self.data.values() if l.is_set()]

    def get_set_landmarks(self):
        """
        Return the list of seen landmarks.

        :returns: list of seen landmarks in the Collection
        """
        return [l for l in self.data.values() if l.is_set()]

    def save(self, filepath, ids=None):
        """
        Save landmarks to a specified yaml file.

        :param filepath: path to save structured landmark data to
        :param ids: if given, only save those landmarks, otherwise save all landmarks
        :returns: `True` if saved successfully, `False` otherwise
        """
        if self.is_empty():
            return True

        lm_yaml = {}
        for lm in self.get_landmark(ids):
            tf_map = lm.get_tf_wrt_map()
            lm_dict = {}
            lm_dict["id"] = lm.id_
            lm_dict["label"] = lm.label_
            lm_dict["set"] = lm.is_set()
            lm_dict["mounted"] = lm.is_mounted()
            lm_dict["mounted_offset"] = lm.mounted_offset
            if lm.is_set():
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
        """
        Load landmarks from a specified yaml file.

        :param filepath: path to yaml file containing structured landmark data
        :returns: `True` if loaded successfully, `False` otherwise
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
                        self.data[key].tf_wrt_map.header.frame_id = lm_dict[key]["tf"]["frame_id"]
                    self.data[key].tf_wrt_map.child_frame_id = lm_dict[key]["tf"]["child_frame_id"]
                    self.data[key].tf_wrt_map.transform.translation.x = lm_dict[key]["tf"]["x"]
                    self.data[key].tf_wrt_map.transform.translation.y = lm_dict[key]["tf"]["y"]
                    self.data[key].tf_wrt_map.transform.translation.z = lm_dict[key]["tf"]["z"]
                    self.data[key].tf_wrt_map.transform.rotation.x = lm_dict[key]["tf"]["qx"]
                    self.data[key].tf_wrt_map.transform.rotation.y = lm_dict[key]["tf"]["qy"]
                    self.data[key].tf_wrt_map.transform.rotation.z = lm_dict[key]["tf"]["qz"]
                    self.data[key].tf_wrt_map.transform.rotation.w = lm_dict[key]["tf"]["qw"]
                    self.data[key].tf_set_ = True
                else:
                    self.data[key].tf_wrt_cam.header.frame_id = self.obs_frame
                    self.data[key].tf_wrt_cam.child_frame_id = self.data[key].get_label()
                    self.data[key].tf_set_ = False

                # get mounted/mounted_offset values from configs
                if "mounted" in lm_dict[key].keys():
                    m = lm_dict[key]["mounted"]
                    mo = lm_dict[key]["mounted_offset"]

                self.data[key].mounted_ = m
                self.data[key].set_mounted_offset(mo)

            self.update_valid_tags()
            if self.ROS:
                self.pub_tfs(self.valid_tags)
                rospy.sleep(rospy.Duration(1.0)) # wait to make sure tf is published
                self.pub_markers(self.valid_tags)

            return True

        except IOError as e:
            rospy.logerr(
                "Error opening '{}': {}. No landmarks loaded.".format(
                    filepath, e))
            return False

    def is_empty(self):
        """
        Check if landmarks is empty.

        :returns: `True` if landmarks is empty, `False` otherwise
        """
        return self.data == {}

    def pub_tfs(self, tag_ids=None):
        """
        Publish TFs to static transforms.

        :param tag_ids: list of tags to publish
        :returns: `True`
        """
        if self.is_empty():
            return True

        if tag_ids == None:
            # when no tags are specified, pubs all
            for lm in self.data.values():
                if lm.is_set():
                    self.static_tf_pub.publish(
                        lm.get_tf_wrt_map())
            return True
        else:
            # when list of tags is specified, pubs tags in list
            for lm in [self.data[x] for x in tag_ids]:
                if lm.is_set():
                    self.static_tf_pub.publish(
                        lm.get_tf_wrt_map())
                return True

    def update_markers(self):
        """Update markers."""
        for lm in self.get_set_landmarks(): # only publish seen tags
            g = lm.get_mb_goal()
            lm.marker_sphere.pose.position.x = g[0]
            lm.marker_sphere.pose.position.y = g[1]
            lm.marker_sphere.pose.position.z = 0.1
            lm.marker_sphere.pose.orientation.x = 0.0
            lm.marker_sphere.pose.orientation.y = 0.0
            lm.marker_sphere.pose.orientation.z = 0.0
            lm.marker_sphere.pose.orientation.w = 1.0
            lm.marker_sphere.header.stamp = rospy.Time(0)

            lm.marker_text.pose.position.x = g[0]
            lm.marker_text.pose.position.y = g[1]
            lm.marker_text.pose.position.z = 0.15
            lm.marker_text.pose.orientation.x = 0.0
            lm.marker_text.pose.orientation.y = 0.0
            lm.marker_text.pose.orientation.z = 0.0
            lm.marker_text.pose.orientation.w = 1.0
            lm.marker_text.header.stamp = rospy.Time(0)

    def pub_markers(self, tag_ids):
        """
        Publish markers.

        :param tag_ids: Markers to publish
        """
        if not self.ROS:
            rospy.logwarn(
                "Tried to publish marker but node is not active.")
            return
        else:
            self.update_markers()
            msg = MarkerArray()
            for lm in self.get_set_landmarks():
                if lm.get_id() in tag_ids:
                    msg.markers.append(lm.marker_sphere)
                    msg.markers.append(lm.marker_text)
                    self.marker_pub.publish(msg)

    def __repr__(self):
        ls = []
        for lm in self.data.values():
            ls.append(lm)
        return str(ls)

    def __len__(self):
        return len(self.data)
