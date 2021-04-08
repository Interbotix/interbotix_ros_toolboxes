import rospy
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped, Quaternion, Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from interbotix_perception_modules.apriltag import InterbotixAprilTagInterface
from interbotix_common_modules import angle_manipulation as ang

### @brief A module to find an arm's base link frame relative to some reference frame (using the help of the AprilTag on the arm)
### @param armtag_ns - namespace where the ROS parameters needed by the module are located
### @param apriltag_ns - namespace where the ROS parameters needed by the InterbotixAprilTagInterface module are located
### @param init_node - whether or not the module should initalize a ROS node; set to False if a node was already initalized somwhere else
class InterbotixArmTagInterface(object):
    def __init__(self, armtag_ns="armtag", apriltag_ns="apriltag", init_node=False):
        if (init_node):
            rospy.init_node(armtag_ns.strip("/") + "_interface")
        self.arm_tag_frame = rospy.get_param("/" + armtag_ns + "/arm_tag_frame")
        self.ref_frame = rospy.get_param("/" + armtag_ns + "/ref_frame")
        self.arm_base_frame = rospy.get_param("/" + armtag_ns + "/arm_base_frame")
        self.trans = TransformStamped()
        self.trans.header.frame_id = self.ref_frame
        self.trans.child_frame_id = self.arm_base_frame
        self.trans.transform.rotation.w = 1.0
        self.rpy = [0,0,0]
        self.apriltag = InterbotixAprilTagInterface(apriltag_ns, False)
        print("Initialized InterbotixArmTagInterface!\n")

    ### @brief Snaps an image of the AprilTag, then computes the transform of the robot's base_link frame w.r.t. the desired reference frame
    ### @param ref_frame - desired reference frame; defaults to self.ref_frame if not specified
    ### @param arm_base_frame - desired base_link frame (either the arm's actual base_link frame or a parent frame); defaults to self.arm_base_frame if not specified
    ### @param num_samples - number of AprilTag snapshots to take before averaging them to get a more accurate pose
    ### @param position_only - if True, only the x,y,z position of the snapped AR tag will be considered; if False, the orientation of the AR tag will be used as well
    ### @details - the 'position_only' parameter can only be set to True if there already exists a 'tf' path from the camera color frame to the AR tag frame on the arm;
    ###            it can be used to try to get a more accurate position of the AR tag than what is dictated by the URDF
    def find_ref_to_arm_base_transform(self, ref_frame=None, arm_base_frame=None, num_samples=5, position_only=False):
        if ref_frame == None:
            ref_frame = self.ref_frame
        if arm_base_frame == None:
            arm_base_frame = self.arm_base_frame

        # take the average pose (w.r.t. the camera frame) of the AprilTag over 'num_samples' samples
        point = Point()
        rpy = [0, 0, 0]
        for x in range(num_samples):
            ps = self.apriltag.find_pose()
            point.x += ps.position.x / float(num_samples)
            point.y += ps.position.y / float(num_samples)
            point.z += ps.position.z / float(num_samples)
            quat_sample = ps.orientation
            quat_list = [quat_sample.x, quat_sample.y, quat_sample.z, quat_sample.w]
            rpy_sample = euler_from_quaternion(quat_list)
            rpy[0] += rpy_sample[0] / float(num_samples)
            rpy[1] += rpy_sample[1] / float(num_samples)
            rpy[2] += rpy_sample[2] / float(num_samples)
        T_CamTag = ang.poseToTransformationMatrix([point.x, point.y, point.z, rpy[0], rpy[1], rpy[2]])

        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        # If position_only, set the orientation of the found AR tag to be equivalent to the orientation of the arm's AR tag as dictated by the URDF
        if (position_only):
            T_CamActualTag = self.get_transform(tfBuffer, self.apriltag.image_frame_id, self.arm_tag_frame)
            T_CamTag[:3,:3] = T_CamActualTag[:3,:3]

        # Now, get a snapshot of the pose of arm's base_link frame w.r.t. the AR tag link (as defined in the URDF - not the one found by the algorithm)
        # We can't publish the AR tag pose found using the AprilTag algorithm to the /tf tree since ROS forbids a link to have multiple parents
        T_TagBase = self.get_transform(tfBuffer, self.arm_tag_frame, arm_base_frame)

        # Now, lets find the transform of the arm's base_link frame w.r.t. the reference frame
        T_CamBase = np.dot(T_CamTag, T_TagBase)
        if ref_frame == self.apriltag.image_frame_id:
            T_RefBase = T_CamBase
        else:
            T_RefCam = self.get_transform(tfBuffer, ref_frame, self.apriltag.image_frame_id)
            T_RefBase = np.dot(T_RefCam, T_CamBase)

        # Now, we can publish the transform from the reference link to the arm's base_link legally as the arm's base_link has no parent
        # (or even if it does, we can safely overwrite it since the 'tf' tree will remain intact)
        self.rpy = ang.rotationMatrixToEulerAngles(T_RefBase[:3,:3])
        quat = quaternion_from_euler(self.rpy[0], self.rpy[1], self.rpy[2])
        self.trans = TransformStamped()
        self.trans.transform.translation.x = T_RefBase[0,3]
        self.trans.transform.translation.y = T_RefBase[1,3]
        self.trans.transform.translation.z = T_RefBase[2,3]
        self.trans.transform.rotation = Quaternion(quat[0], quat[1], quat[2], quat[3])
        self.trans.header.frame_id = ref_frame
        self.trans.child_frame_id = arm_base_frame
        self.trans.header.stamp = rospy.Time.now()
        self.apriltag.pub_transforms.publish(self.trans)

    ### @brief Helper function to lookup a transform and convert it into a 4x4 transformation matrix
    ### @param tfBuffer - tf2_ros buffer instance from which to lookup transforms from the 'tf' tree
    ### @param target_frame - the frame to which data should be transformed
    ### @param source_frame - the frame where the data originated
    ### @return T_TargetSource - desired 4x4 numpy transformation matrix
    def get_transform(self, tfBuffer, target_frame, source_frame):
        try:
            trans = tfBuffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(4.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Failed to look up the transform from '%s' to '%s'." % (target_frame, source_frame))
            return np.identity(4)
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        z = trans.transform.translation.z
        quat = trans.transform.rotation
        quat_list = [quat.x, quat.y, quat.z, quat.w]
        rpy = euler_from_quaternion(quat_list)
        T_TargetSource = ang.poseToTransformationMatrix([x, y, z, rpy[0], rpy[1], rpy[2]])
        return T_TargetSource

    ### @brief Get the 'x' component of T_RefBase
    ### @return 'x' [m]
    def get_x(self):
        return self.trans.transform.translation.x

    ### @brief Get the 'y' component of T_RefBase
    ### @return 'y' [m]
    def get_y(self):
        return self.trans.transform.translation.y

    ### @brief Get the 'z' component of T_RefBase
    ### @return 'z' [m]
    def get_z(self):
        return self.trans.transform.translation.z

    ### @brief Get the 'roll' component of T_RefBase
    ### @return 'roll' [rad]
    def get_roll(self):
        return self.rpy[0]

    ### @brief Get the 'pitch' component of T_RefBase
    ### @return 'pitch' [rad]
    def get_pitch(self):
        return self.rpy[1]

    ### @brief Get the 'yaw' component of T_RefBase
    ### @return 'yaw' [rad]
    def get_yaw(self):
        return self.rpy[2]

    ### @brief Get the parent frame of T_RefBase (usually something like 'camera_color_optical_frame')
    ### @return 'frame_id'
    def get_parent_frame(self):
        return self.trans.header.frame_id

    ### @brief Get the child frame of T_RefBase (usually something like 'base_link')
    ### @return 'child_frame_id'
    def get_child_frame(self):
        return self.trans.child_frame_id
