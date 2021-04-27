import yaml
import rospy
import tf2_ros
import numpy as np
from std_srvs.srv import SetBool
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, Quaternion
from interbotix_perception_modules.srv import *
from interbotix_perception_modules.msg import ClusterInfo
from interbotix_common_modules import angle_manipulation as ang

### @brief Python API to tune filter parameters to get accurate position estimates of objects seen by the camera
### @param filter_ns - namespace where the ROS parameters needed by the module are located
### @param init_node - whether or not the module should initalize a ROS node; set to False if a node was already initalized somwhere else
### @details - Note that the default reference frame for the pointcloud is used (usually something like 'camera_depth_optical_frame')
class InterbotixPointCloudInterface(object):
    def __init__(self, filter_ns="pc_filter", init_node=False):
        if (init_node):
            rospy.init_node(filter_ns.strip("/") + "_interface")
        self.params = FilterParamsRequest()
        self.param_filepath = rospy.get_param("/" + filter_ns + "/filter_params")
        self.load_params_from_param_server(filter_ns)
        rospy.wait_for_service("/" + filter_ns + "/set_filter_params")
        rospy.wait_for_service("/" + filter_ns + "/enable_pipeline")
        rospy.wait_for_service("/" + filter_ns + "/get_cluster_positions")
        self.srv_set_params = rospy.ServiceProxy("/" + filter_ns + "/set_filter_params", FilterParams)
        self.srv_enable_pipeline = rospy.ServiceProxy("/" + filter_ns + "/enable_pipeline", SetBool)
        self.srv_get_cluster_positions = rospy.ServiceProxy("/" + filter_ns + "/get_cluster_positions", ClusterInfoArray)
        self.br = tf2_ros.TransformBroadcaster()
        print("Initialized InterbotixPointCloudInterface\n")

    ### @brief Helper function to convert from a Python dictionary to a FilterParams Service message
    ### @param param_dict - Python Dictionary to convert to a FilterParams message
    def set_params(self, param_dict):
        self.params.x_filter_min = param_dict["x_filter_min"]
        self.params.x_filter_max = param_dict["x_filter_max"]
        self.params.y_filter_min = param_dict["y_filter_min"]
        self.params.y_filter_max = param_dict["y_filter_max"]
        self.params.z_filter_min = param_dict["z_filter_min"]
        self.params.z_filter_max = param_dict["z_filter_max"]
        self.params.voxel_leaf_size = param_dict["voxel_leaf_size"]
        self.params.plane_max_iter = int(param_dict["plane_max_iter"])
        self.params.plane_dist_thresh = param_dict["plane_dist_thresh"]
        self.params.ror_radius_search = param_dict["ror_radius_search"]
        self.params.ror_min_neighbors = int(param_dict["ror_min_neighbors"])
        self.params.cluster_tol = param_dict["cluster_tol"]
        self.params.cluster_min_size = int(param_dict["cluster_min_size"])
        self.params.cluster_max_size = int(param_dict["cluster_max_size"])

    ### @brief Filters out any data point in a pointcloud with an 'x' value less than 'x_filter_min'
    ### @param x_filter_min - desired minimum 'x' value [m]
    def set_x_filter_min(self, x_filter_min):
        self.params.x_filter_min = x_filter_min
        self.srv_set_params(self.params)

    ### @brief Filters out any data point in a pointcloud with an 'x' value more than 'x_filter_max'
    ### @param x_filter_max - desired maximum 'x' value [m]
    def set_x_filter_max(self, x_filter_max):
        self.params.x_filter_max = x_filter_max
        self.srv_set_params(self.params)

    ### @brief Filters out any data point in a pointcloud with a 'y' value less than 'y_filter_min'
    ### @param y_filter_min - desired minimum 'y' value [m]
    def set_y_filter_min(self, y_filter_min):
        self.params.y_filter_min = y_filter_min
        self.srv_set_params(self.params)

    ### @brief Filters out any data point in a pointcloud with a 'y' value more than 'y_filter_max'
    ### @param y_filter_max - desired maximum 'y' value [m]
    def set_y_filter_max(self, y_filter_max):
        self.params.y_filter_max = y_filter_max
        self.srv_set_params(self.params)

    ### @brief Filters out any data point in a pointcloud with a 'z' value less than 'z_filter_min'
    ### @param z_filter_min - desired minimum 'z' value [m]
    def set_z_filter_min(self, z_filter_min):
        self.params.z_filter_min = z_filter_min
        self.srv_set_params(self.params)

    ### @brief Filters out any data point in a pointcloud with a 'z' value more than 'z_filter_max'
    ### @param z_filter_max - desired maximum 'z' value [m]
    def set_z_filter_max(self, z_filter_max):
        self.params.z_filter_max = z_filter_max
        self.srv_set_params(self.params)

    ### @brief Sets the voxel leaf size
    ### @param voxel_leaf_size - desired voxel size [m] that applies to the x, y, and z dimensions
    def set_voxel_leaf_size(self, voxel_leaf_size):
        self.params.voxel_leaf_size = voxel_leaf_size
        self.srv_set_params(self.params)

    ### @brief Set the maximum number of iterations the sample consensus method should run
    ### @param plane_max_iter - desired max iterations (default is 50)
    def set_plane_max_iter(self, plane_max_iter):
        self.params.plane_max_iter = int(plane_max_iter)
        self.srv_set_params(self.params)

    ### @brief Set the max distance perpendicular from the calculated 'plane' in which a point should be considerd part of the plane
    ### @param plane_dist_thresh - desired max distance [m] (default is about 0.01 meters)
    def set_plane_dist_thresh(self, plane_dist_thresh):
        self.params.plane_dist_thresh = plane_dist_thresh
        self.srv_set_params(self.params)

    ### @brief Set the radius around any given point to search for neighbors
    ### @param ror_radius_search - desired radius [m] (default is about 0.01 meters)
    def set_ror_radius_search(self, ror_radius_search):
        self.params.ror_radius_search = ror_radius_search
        self.srv_set_params(self.params)

    ### @brief Set the minimum number of neighbors (within the radius set above)
    ###        that any given point should have to remain in the pointcloud
    def set_ror_min_neighbors(self, ror_min_neighbors):
        self.params.ror_min_neighbors = int(ror_min_neighbors)
        self.srv_set_params(self.params)

    ### @brief Set the cluster tolerance - all points that are within 'cluster_tol'
    ###        of each other are considered to be part of the same cluster
    ### @param cluster_tol - desired cluster tolerance [m] (default is about 0.02 meters)
    def set_cluster_tol(self, cluster_tol):
        self.params.cluster_tol = cluster_tol
        self.srv_set_params(self.params)

    ### @brief Set the minimum size that a cluster must be to be considered a cluster
    ### @param cluster_min_size - desired minimum size [number of points in a cluster's pointcloud]
    def set_cluster_min_size(self, cluster_min_size):
        self.params.cluster_min_size = int(cluster_min_size)
        self.srv_set_params(self.params)

    ### @brief Set the maximum size that a cluster can be to be considered a cluster
    ### @param cluster_max_size - desired maximum size [number of points in a cluster's pointcloud]
    def set_cluster_max_size(self, cluster_max_size):
        self.params.cluster_max_size = int(cluster_max_size)
        self.srv_set_params(self.params)

    ### @brief Helper function to convert from a FilterParams Service type to a Python Dictionary
    ### @return param_dict - python dictionary containing the data from the FilterParams service message
    def get_params(self):
        param_dict = {}
        param_dict["x_filter_min"] = self.params.x_filter_min
        param_dict["x_filter_max"] = self.params.x_filter_max
        param_dict["y_filter_min"] = self.params.y_filter_min
        param_dict["y_filter_max"] = self.params.y_filter_max
        param_dict["z_filter_min"] = self.params.z_filter_min
        param_dict["z_filter_max"] = self.params.z_filter_max
        param_dict["voxel_leaf_size"] = self.params.voxel_leaf_size
        param_dict["plane_max_iter"] = self.params.plane_max_iter
        param_dict["plane_dist_thresh"] = self.params.plane_dist_thresh
        param_dict["ror_radius_search"] = self.params.ror_radius_search
        param_dict["ror_min_neighbors"] = self.params.ror_min_neighbors
        param_dict["cluster_tol"] = self.params.cluster_tol
        param_dict["cluster_min_size"] = self.params.cluster_min_size
        param_dict["cluster_max_size"] = self.params.cluster_max_size
        return param_dict

    ### @brief Gets the current minimum 'x' value
    ### @return x_filter_min [m]
    def get_x_filter_min(self):
        return self.params.x_filter_min

    ### @brief Gets the current maximum 'x' value
    ### @return x_filter_max [m]
    def get_x_filter_max(self):
        return self.params.x_filter_max

    ### @brief Gets the current minimum 'y' value
    ### @return y_filter_min [m]
    def get_y_filter_min(self):
        return self.params.y_filter_min

    ### @brief Gets the current maximum 'y' value
    ### @return y_filter_max [m]
    def get_y_filter_max(self):
        return self.params.y_filter_max

    ### @brief Gets the current minimum 'z' value
    ### @return z_filter_min [m]
    def get_z_filter_min(self):
        return self.params.z_filter_min

    ### @brief Gets the current maximum 'z' value
    ### @return z_filter_max [m]
    def get_z_filter_max(self):
        return self.params.z_filter_max

    ### @brief Gets the current voxel size
    ### @return voxel_leaf_size [m]
    def get_voxel_leaf_size(self):
        return self.params.voxel_leaf_size

    ### @brief Gets the current max number of iterations for the planar segmentation algorithm
    ### @return plane_max_iter
    def get_plane_max_iter(self):
        return self.params.plane_max_iter

    ### @brief Gets the current max distance from the plane model to be considered part of the plane
    ### @return plane_dist_thresh [m]
    def get_plane_dist_thresh(self):
        return self.params.plane_dist_thresh

    ### @brief Gets the current radius used when searching for nearest neighbor points
    ### @return ror_radius_search [m]
    def get_ror_radius_search(self):
        return self.params.ror_radius_search

    ### @brief Gets the current minimum number of neighbors to search for for a point not to be removed
    ### @return ror_min_neighbors
    def get_ror_min_neighbors(self):
        return self.params.ror_min_neighbors

    ### @brief Gets the current cluster tolerance
    ### @return cluster_tol [m]
    def get_cluster_tol(self):
        return self.params.cluster_tol

    ### @brief Gets the current minimum cluster size
    ### @return cluster_min_size [number of points in the pointcloud cluster]
    def get_cluster_min_size(self):
        return self.params.cluster_min_size

    ### @brief Gets the current maximum cluster size
    ### @return cluster_max_size [number of points in the pointcloud cluster]
    def get_cluster_max_size(self):
        return self.params.cluster_max_size

    ### @brief Get the absolute filepath to the config file containing all the filter parameters
    ### @return param_filepath - string with the absolute filepath
    def get_filepath(self):
        return self.param_filepath

    ### @brief Enables/Disables the pointcloud filtering process
    ### @param enable - boolean that it True, enables the pipeline, and if False, disables the pipeline
    def enable_pipeline(self, enable):
        self.srv_enable_pipeline(enable)

    ### @brief Get the estimated positions of all pointcloud clusters
    ### @param num_samples - number of times to run the pipeline to get the cluster positions; these samples are then averaged together to get a more accurate result
    ### @param period - number of seconds to wait between sampling (give time for the backend to get updated with a new pointcloud)
    ### @param ref_frame - the desired reference frame the cluster positions should be transformed into; if unspecified, the camera's depth frame is used
    ### @param sort_axis - the axis of the 'ref_frame' by which to sort the cluster positions when returning them to the user
    ### @param reverse - if False, cluster positions are sorted in ascending order along the axis specified by 'sort_axis'; if True, the positions are sorted in descending order
    ### @param is_parallel - if False, the cluster positions returned to the user represent the centroids of each cluster w.r.t. the 'ref_frame';
    ###                      if True, the cluster positions returned to the user represent the centroids of each cluster, but positioned at the top of each cluster w.r.t. the 'ref_frame';
    ###                      set this to True if the 'ref_frame' is parallel to the surface that the objects are on
    ### @return <bool>, final_clusters - True if the algorithm succeeded or False otherwise. If False, the 'final_clusters' list is empty, but if True, a list of
    ###                                  dictionaries representing each cluster is returned to the user
    def get_cluster_positions(self, num_samples=5, period=0.1, ref_frame=None, sort_axis="y", reverse=False, is_parallel=True):
        root_clusters = self.srv_get_cluster_positions().clusters
        num_clusters = len(root_clusters)
        if num_clusters == 0:
            rospy.logwarn("No clusters found...")
            return False, []
        cluster_frame = root_clusters[0].frame_id

        # Calculate the average for each cluster based on 'num_samples' samples
        avg_clusters = [ClusterInfo() for i in range(num_clusters)]
        for x in range(num_samples):
            clusters = self.srv_get_cluster_positions().clusters
            if len(clusters) != num_clusters:
                rospy.logwarn("Found %d clusters instead of %d clusters..." % (len(clusters), num_clusters))
                return False, []
            valid_indices = list(range(num_clusters))
            for cluster in clusters:
                for indx in valid_indices:
                    if (abs(root_clusters[indx].position.x - cluster.position.x) < self.params.cluster_tol and
                       abs(root_clusters[indx].position.y - cluster.position.y) < self.params.cluster_tol and
                       abs(root_clusters[indx].position.z - cluster.position.z) < self.params.cluster_tol):
                       avg_clusters[indx].position.x += cluster.position.x / num_samples
                       avg_clusters[indx].position.y += cluster.position.y / num_samples
                       avg_clusters[indx].position.z += cluster.position.z /num_samples
                       avg_clusters[indx].color.r += cluster.color.r / num_samples
                       avg_clusters[indx].color.g += cluster.color.g / num_samples
                       avg_clusters[indx].color.b += cluster.color.b / num_samples
                       avg_clusters[indx].min_z_point.x += cluster.min_z_point.x / num_samples
                       avg_clusters[indx].min_z_point.y += cluster.min_z_point.y / num_samples
                       avg_clusters[indx].min_z_point.z += cluster.min_z_point.z / num_samples
                       avg_clusters[indx].num_points += cluster.num_points / float(num_samples)
                       valid_indices.remove(indx)
                       break
                    elif (indx == num_clusters - 1):
                        rospy.logwarn("Could not match the cluster. Please tune the filter parameters such that all spherical 'object markers' are constant in their respective clusters and do not flicker.")
                        return False, []
            rospy.sleep(period)

        # Get the transform from the 'ref_frame' to the cluster frame (i.e. the camera's depth frame) - known as T_rc
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        try:
            trans = tfBuffer.lookup_transform(ref_frame, cluster_frame, rospy.Time(0), rospy.Duration(4.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Failed to look up the transform from '%s' to '%s'." % (ref_frame, cluster_frame))
            return False, []
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        z = trans.transform.translation.z
        quat = trans.transform.rotation
        q = [quat.x, quat.y, quat.z, quat.w]
        rpy = euler_from_quaternion(q)
        T_rc = ang.poseToTransformationMatrix([x, y, z, rpy[0], rpy[1], rpy[2]])

        # Sort the clusters from left to right w.r.t. the camera's depth frame
        sorted_cam_clusters = sorted(avg_clusters, key=lambda cluster : cluster.position.x)

        # TO DO...
        # if is_parallel is True...
        # find 'yaw' of each cluster relative to a virtual frame located at 'cluster_frame' (with the 'x' axis
        # pointed in the same direction, but with the 'z' axis pointing straight up) using OpenCV (order is left to right)
        # set this 'yaw' to the yaw variable in each cluster; it will be added to the yaw part of T_rc before the transform is published

        # Transform the clusters to be w.r.t. the 'ref_frame' instead of the camera's depth frame
        for cluster in sorted_cam_clusters:
            # p_co is the cluster's position w.r.t. the camera's depth frame
            # p_ro is the cluster's position w.r.t. the desired reference frame
            p_co = [cluster.position.x, cluster.position.y, cluster.position.z, 1]
            p_ro = np.dot(T_rc, p_co)
            if (is_parallel):
                # p_comin is the minimum point of the cluster (in the 'z' direction) w.r.t. the camera's depth frame;
                # it is assumed that this point lies at the top or very near the top of the cluster
                # p_romin is the same point w.r.t. the desired reference frame; the 'z' element of this point replaces
                # the 'z' element in p_ro; thus, a tf frame published at this point should appear at the 'top-center' of the cluster
                p_comin = [cluster.min_z_point.x, cluster.min_z_point.y, cluster.min_z_point.z, 1]
                p_romin = np.dot(T_rc, p_comin)
                p_ro[2] = p_romin[2]
            cluster.position.x = p_ro[0]
            cluster.position.y = p_ro[1]
            cluster.position.z = p_ro[2]

        # Sort the clusters based on user input
        if sort_axis == "x":
            key = lambda cluster : cluster.position.x
        elif sort_axis == "y":
            key = lambda cluster : cluster.position.y
        elif sort_axis == "z":
            key = lambda cluster : cluster.position.z
        else:
            key = lambda cluster : cluster.position.y
            rospy.logwarn("'%s' is not a valid sorting axis. Set the 'sort_axis' argument to 'x', 'y', or 'z'. Defaulting to 'y'." % sort_axis)
        sorted_ref_clusters = sorted(sorted_cam_clusters, key=key, reverse=reverse)

        # publish transforms to the /tf tree for debugging purposes (only once)
        final_trans = []
        cluster_num = 1
        time_now = rospy.Time.now()
        for cluster in sorted_ref_clusters:
            trans = TransformStamped()
            trans.header.frame_id = ref_frame
            trans.header.stamp = time_now
            trans.child_frame_id = "cluster_" + str(cluster_num)
            trans.transform.translation.x = cluster.position.x
            trans.transform.translation.y = cluster.position.y
            trans.transform.translation.z = cluster.position.z
            trans.transform.rotation = Quaternion(0, 0, 0, 1)
            final_trans.append(trans)
            cluster_num += 1
        self.br.sendTransform(final_trans)

        # create a list of Python dictionaries to return to the user
        final_clusters = []
        for indx in range(num_clusters):
            name = final_trans[indx].child_frame_id
            x = final_trans[indx].transform.translation.x
            y = final_trans[indx].transform.translation.y
            z = final_trans[indx].transform.translation.z
            yaw = 0
            r = sorted_ref_clusters[indx].color.r
            g = sorted_ref_clusters[indx].color.g
            b = sorted_ref_clusters[indx].color.b
            num_points = sorted_ref_clusters[indx].num_points
            cluster = {"name" : name, "position" : [x, y, z], "yaw" : yaw, "color" : [r, g, b], "num_points" : num_points}
            final_clusters.append(cluster)
        return True, final_clusters

    ### @brief Get params from the specified filepath
    ### @param filepath - YAML config file to get the params; if None, the default filepath specified by the 'filter_params' ROS parameter is used
    def load_params(self, filepath=None):
        if filepath is None:
            filepath = self.param_filepath
        with open(filepath, "r") as yamlfile:
            param_dict = yaml.safe_load(yamlfile)
        self.set_params(param_dict)

    def load_params_from_param_server(self, filter_ns):
        self.params.x_filter_min = rospy.get_param("/" + filter_ns + "/x_filter_min")
        self.params.x_filter_max = rospy.get_param("/" + filter_ns + "/x_filter_max")
        self.params.y_filter_min = rospy.get_param("/" + filter_ns + "/y_filter_min")
        self.params.y_filter_max = rospy.get_param("/" + filter_ns + "/y_filter_max")
        self.params.z_filter_min = rospy.get_param("/" + filter_ns + "/z_filter_min")
        self.params.z_filter_max = rospy.get_param("/" + filter_ns + "/z_filter_max")
        self.params.voxel_leaf_size = rospy.get_param("/" + filter_ns + "/voxel_leaf_size")
        self.params.plane_max_iter = int(rospy.get_param("/" + filter_ns + "/plane_max_iter"))
        self.params.plane_dist_thresh = rospy.get_param("/" + filter_ns + "/plane_dist_thresh")
        self.params.ror_radius_search = rospy.get_param("/" + filter_ns + "/ror_radius_search")
        self.params.ror_min_neighbors = int(rospy.get_param("/" + filter_ns + "/ror_min_neighbors"))
        self.params.cluster_tol = rospy.get_param("/" + filter_ns + "/cluster_tol")
        self.params.cluster_min_size = int(rospy.get_param("/" + filter_ns + "/cluster_min_size"))
        self.params.cluster_max_size = int(rospy.get_param("/" + filter_ns + "/cluster_max_size"))

    ### @brief Save params to the specified filepath
    ### @param filepath - YAML config file to save the params; if None, the default filepath specified by the 'filter_params' ROS parameter is used
    def save_params(self, filepath=None):
        if filepath is None:
            filepath = self.param_filepath
        param_dict = self.get_params()
        with open(filepath, "w") as yamlfile:
            yaml.dump(param_dict, yamlfile, default_flow_style=False)
