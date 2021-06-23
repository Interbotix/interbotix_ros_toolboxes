import rospy
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import TransformStamped, Pose
from interbotix_perception_modules.srv import SnapPicture
from apriltag_ros.srv import AnalyzeSingleImage, AnalyzeSingleImageRequest

### @brief Python API to snap the pose of an AprilTag
### @param apriltag_ns - the namespace that the AprilTag node and other related parameters are in
### @param init_node - whether or not the module should initalize a ROS node; set to False if a node was already initalized somwhere else
class InterbotixAprilTagInterface(object):
    def __init__(self, apriltag_ns="apriltag", init_node=False, 
                full_img_get_path="/tmp/get_image.png",
                full_img_save_path="/tmp/save_image.png"):
        
        # start apriltag interface node
        if (init_node):
            rospy.init_node(apriltag_ns.strip("/") + "_interface")
        
        self.image_frame_id = None
        
        # set up subs, pubs, and services
        camera_info_topic = rospy.get_param(
            "/" + apriltag_ns + "/camera_info_topic",
            default="camera/color/image_raw").strip("/")
        rospy.wait_for_service("/" + apriltag_ns + "/snap_picture")
        rospy.wait_for_service("/" + apriltag_ns + "/single_image_tag_detection")
        self.sub_camera_info = rospy.Subscriber("/" + camera_info_topic, CameraInfo, self.camera_info_cb)
        self.srv_snap_picture = rospy.ServiceProxy("/" + apriltag_ns + "/snap_picture", SnapPicture)
        self.request = AnalyzeSingleImageRequest()
        self.request.full_path_where_to_get_image = full_img_get_path
        self.request.full_path_where_to_save_image = full_img_save_path
        self.srv_analyze_image = rospy.ServiceProxy("/" + apriltag_ns + "/single_image_tag_detection", AnalyzeSingleImage)
        self.pub_transforms = rospy.Publisher("/static_transforms", TransformStamped, queue_size=50)
        
        rospy.sleep(0.5)
        
        # wait to receive camera info (means that we are properly subbed)
        while (self.request.camera_info == CameraInfo() and not rospy.is_shutdown()):
            pass
        
        print("Initialized InterbotixAprilTagInterface!\n")

    ### @brief ROS Subscriber Callback to get the camera info
    ### @param msg - ROS CameraInfo message
    ### @details - unsubscribes after receiving the first message as this never changes
    def camera_info_cb(self, msg):
        self.image_frame_id = msg.header.frame_id
        self.request.camera_info = msg
        self.sub_camera_info.unregister()

    ### @brief Calculates the AprilTag pose w.r.t. the camera color image frame
    ### @param ar_tag_name - AprilTag name (only used if publishing to the /tf tree)
    ### @param publish_tf - whether to publish the pose of the AprilTag to the /tf tree
    ### @return pose - Pose message of the proposed AR tag w.r.t. the camera color image frame
    ### @details - tf is published to the StaticTransformManager node (in this package) as a static transform
    ###            if no tags are detected, a genertic invalid pose is returned
    def find_pose(self, ar_tag_name="ar_tag", publish_tf=False):
        self.srv_snap_picture(self.request.full_path_where_to_get_image)
        detections = self.srv_analyze_image(self.request).tag_detections.detections
        
        # check if tags were detected, return genertic pose if not
        if len(detections) == 0:
            pose = Pose()
        else:
            pose = detections[0].pose.pose.pose #TODO: support for multiple tags
        
        # publish pose to /static_transforms
        if publish_tf:
            rospy.logwarn("Could not find " + ar_tag_name + ". Returning a 'zero' Pose...")
            msg = TransformStamped()
            msg.header.frame_id = self.image_frame_id
            msg.header.stamp = rospy.Time.now()
            msg.child_frame_id = ar_tag_name
            msg.transform.translation.x = pose.position.x
            msg.transform.translation.y = pose.position.y
            msg.transform.translation.z = pose.position.z
            msg.transform.rotation = pose.orientation
            self.pub_transforms.publish(msg)
        
        return pose
