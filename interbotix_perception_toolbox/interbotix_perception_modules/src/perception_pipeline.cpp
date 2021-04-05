#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include "interbotix_perception_modules/FilterParams.h"
#include "interbotix_perception_modules/ClusterInfo.h"
#include "interbotix_perception_modules/ClusterInfoArray.h"

typedef pcl::PointXYZRGB PointT;

ros::Publisher pub_pc_obj, pub_pc_filter, pub_marker_obj, pub_marker_crop;
visualization_msgs::Marker marker_obj, marker_crop;
sensor_msgs::PointCloud2ConstPtr input;
std::vector<interbotix_perception_modules::ClusterInfo> cluster_info_vector;
bool enable_pipeline = true;
std::string cloud_topic;
float x_filter_min, y_filter_min, z_filter_min;
float x_filter_max, y_filter_max, z_filter_max;
float voxel_leaf_size, plane_dist_thresh, ror_radius_search, cluster_tol;
int plane_max_iter, ror_min_neighbors, cluster_min_size, cluster_max_size;

// @brief Processes the raw pointcloud and finds the centroids of every cluster
void perception_pipeline()
{
  // Convert to pcl point cloud
  pcl::PointCloud<PointT>::Ptr cloud_raw (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_voxel_downsampled (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_cropped (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_no_plane (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_no_noise (new pcl::PointCloud<PointT>);
  pcl::fromROSMsg (*input, *cloud_raw);
  if (cloud_raw->size() == 0) return;

  // Cropbox Filter
  pcl::CropBox<PointT> crop;
  crop.setInputCloud(cloud_raw);
  Eigen::Vector4f min_point = Eigen::Vector4f(x_filter_min, y_filter_min, z_filter_min, 0);
  Eigen::Vector4f max_point = Eigen::Vector4f(x_filter_max, y_filter_max, z_filter_max, 0);
  crop.setMin(min_point);
  crop.setMax(max_point);
  crop.filter(*cloud_cropped);
  if (cloud_cropped->size() == 0) return;

  // Downsample via the VoxelFilter
  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud (cloud_cropped);
  vg.setLeafSize (voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
  vg.filter (*cloud_voxel_downsampled);
  if(cloud_voxel_downsampled->size() < 4) return;

  // Convert to ROS PointCloud2 msg and publish quasi-filtered pointcloud (for debugging)
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud_voxel_downsampled, output);
  pub_pc_filter.publish (output);

  // Publish Semi-transparent cube outlining the crop box filter's borders
  marker_crop.header.frame_id = input->header.frame_id;
  marker_crop.pose.position.x = (x_filter_min + x_filter_max) / 2.0;
  marker_crop.pose.position.y = (y_filter_min + y_filter_max) / 2.0;
  marker_crop.pose.position.z = (z_filter_min + z_filter_max) / 2.0;
  marker_crop.scale.x = x_filter_max - x_filter_min;
  marker_crop.scale.y = y_filter_max - y_filter_min;
  marker_crop.scale.z = z_filter_max - z_filter_min;
  pub_marker_crop.publish(marker_crop);

  // Do segmentation of the table
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations(plane_max_iter);
  seg.setDistanceThreshold (plane_dist_thresh);
  seg.setInputCloud (cloud_voxel_downsampled);
  seg.segment (*inliers, *coefficients);

  // Extract indices of the segmented plane and remove it from the cloud
  pcl::ExtractIndices<PointT> ei;
  ei.setInputCloud(cloud_voxel_downsampled);
  ei.setIndices(inliers);
  ei.setNegative(true);
  ei.filter(*cloud_no_plane);
  if(cloud_no_plane->size() == 0) return;

  // Radius Outlier Removal (removes random noise in the cloud)
  pcl::RadiusOutlierRemoval<PointT> ror;
  ror.setInputCloud(cloud_no_plane);
  ror.setRadiusSearch(ror_radius_search);
  ror.setMinNeighborsInRadius (ror_min_neighbors);
  ror.setKeepOrganized(false);
  ror.filter (*cloud_no_noise);
  if(cloud_no_noise->size() == 0) return;

  // Publish the final processed pointcloud
  sensor_msgs::PointCloud2 output_2;
  pcl::toROSMsg(*cloud_no_noise, output_2);
  pub_pc_obj.publish (output_2);

  // Extract Clusters
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud (cloud_no_noise);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (cluster_tol); // 2cm
  ec.setMinClusterSize (cluster_min_size);
  ec.setMaxClusterSize (cluster_max_size);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_no_noise);
  ec.extract (cluster_indices);

  int j = 0;
  cluster_info_vector.clear();
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->push_back ((*cloud_no_noise)[*pit]);
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    // Compute the centroid of each cluster. Then find the point with the min 'z'
    // value. This point is assumed to be at the top of the pointcloud cluster,
    // and can be used to make sure the arm doesn't go too far down when grasping
    PointT cntrd, min_point;
    pcl::computeCentroid(*cloud_cluster, cntrd);
    min_point = cloud_cluster->points[0];
    for (size_t indx {1}; indx < cloud_cluster->size(); indx++)
    {
      if (cloud_cluster->points[indx].z < min_point.z)
        min_point = cloud_cluster->points[indx];
    }

    // Publish a spherical marker showing the centroid of the cluster
    marker_obj.id = j;
    marker_obj.header.frame_id = input->header.frame_id;
    marker_obj.pose.position.x = cntrd.x;
    marker_obj.pose.position.y = cntrd.y;
    marker_obj.pose.position.z = cntrd.z;
    marker_obj.color.r = cntrd.r / 255.0;
    marker_obj.color.g = cntrd.g / 255.5;
    marker_obj.color.b = cntrd.b / 255.0;
    marker_obj.header.stamp = ros::Time::now();
    pub_marker_obj.publish(marker_obj);

    // Save a ClusterInfo message with info on the cluster
    interbotix_perception_modules::ClusterInfo ci_msg;
    ci_msg.frame_id = input->header.frame_id;
    ci_msg.position.x = cntrd.x;
    ci_msg.position.y = cntrd.y;
    ci_msg.position.z = cntrd.z;
    ci_msg.color.r = cntrd.r;
    ci_msg.color.g = cntrd.g;
    ci_msg.color.b = cntrd.b;
    ci_msg.min_z_point.x = min_point.x;
    ci_msg.min_z_point.y = min_point.y;
    ci_msg.min_z_point.z = min_point.z;
    ci_msg.num_points = cloud_cluster->size();
    cluster_info_vector.push_back(ci_msg);
    j++;
  }
}

// @brief ROS Subscriber Callback function to get the latest pointcloud
// @param pointcloud - ROS PointCloud2 message containing the latest pointcloud
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& pointcloud)
{
  input = pointcloud;

  if (enable_pipeline)
    perception_pipeline();
}

// @brief ROS Service to tune the various filter parameters used when processing the pointclouds
// @param req - Custom FilterParams service request containing the parameters to change
// @param res - Custom FitlerParams service response (empty)
// @return <bool> - true if service executed successfully
bool srv_set_filter_params(interbotix_perception_modules::FilterParams::Request& req, interbotix_perception_modules::FilterParams::Response& res)
{
  voxel_leaf_size = req.voxel_leaf_size;
  x_filter_min = req.x_filter_min;
  x_filter_max = req.x_filter_max;
  y_filter_min = req.y_filter_min;
  y_filter_max = req.y_filter_max;
  z_filter_min = req.z_filter_min;
  z_filter_max = req.z_filter_max;
  plane_max_iter = req.plane_max_iter;
  plane_dist_thresh = req.plane_dist_thresh;
  ror_radius_search = req.ror_radius_search;
  ror_min_neighbors = req.ror_min_neighbors;
  cluster_tol = req.cluster_tol;
  cluster_min_size = req.cluster_min_size;
  cluster_max_size = req.cluster_max_size;
  return true;
}

// @brief Get the cluster positions from the latest pointcloud
// @param req - Custom ClusterInfoArray service request (empty)
// @param res - Custom ClusterInfoArray service response containing a list of ClusterInfo messages for each cluster found
// @return <bool> - true if service executed successfully
bool srv_get_cluster_positions(interbotix_perception_modules::ClusterInfoArray::Request& req, interbotix_perception_modules::ClusterInfoArray::Response& res)
{
  if (!enable_pipeline)
    perception_pipeline();

  res.clusters = cluster_info_vector;
  return true;
}

// @brief Enable or Disable the perception pipeline from running continuously
// @param req - SetBool service request containing a boolean on whether to enable/disable pipeline
// @param res - SetBool service response (unused)
// @details - Even if the pipeline is disabled, it will be run once to get the cluster positions if the
//            'get_cluster_positions' service is called;
bool srv_enable_pipeline(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
  enable_pipeline = req.data;
  return true;
}

// @brief Main function
int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pc_filter");
  ros::NodeHandle nh;

  // Get parameters from the parameter server
  // Private Parameters
  ros::param::get("~enable_pipeline", enable_pipeline);
  ros::param::get("~cloud_topic", cloud_topic);

  // Public Parameters
  std::string filter_ns = ros::this_node::getNamespace();
  ros::param::get(filter_ns + "/voxel_leaf_size", voxel_leaf_size);
  ros::param::get(filter_ns + "/x_filter_min", x_filter_min);
  ros::param::get(filter_ns + "/y_filter_min", y_filter_min);
  ros::param::get(filter_ns + "/z_filter_min", z_filter_min);
  ros::param::get(filter_ns + "/x_filter_max", x_filter_max);
  ros::param::get(filter_ns + "/y_filter_max", y_filter_max);
  ros::param::get(filter_ns + "/z_filter_max", z_filter_max);
  ros::param::get(filter_ns + "/plane_max_iter", plane_max_iter);
  ros::param::get(filter_ns + "/plane_dist_thresh", plane_dist_thresh);
  ros::param::get(filter_ns + "/ror_radius_search", ror_radius_search);
  ros::param::get(filter_ns + "/ror_min_neighbors", ror_min_neighbors);
  ros::param::get(filter_ns + "/cluster_tol", cluster_tol);
  ros::param::get(filter_ns + "/cluster_min_size", cluster_min_size);
  ros::param::get(filter_ns + "/cluster_max_size", cluster_max_size);

  // Initialize ROS Services, Subscribers, and Publishers
  ros::ServiceServer service_set_filter_params = nh.advertiseService("set_filter_params", srv_set_filter_params);
  ros::ServiceServer service_get_clusters = nh.advertiseService("get_cluster_positions", srv_get_cluster_positions);
  ros::ServiceServer service_enable_pipeline = nh.advertiseService("enable_pipeline", srv_enable_pipeline);
  ros::Subscriber sub = nh.subscribe (cloud_topic, 1, cloud_cb);
  pub_pc_obj = nh.advertise<sensor_msgs::PointCloud2> ("pointcloud/objects", 1);
  pub_pc_filter = nh.advertise<sensor_msgs::PointCloud2> ("pointcloud/filtered", 1);
  pub_marker_obj = nh.advertise<visualization_msgs::Marker>( "markers/objects", 50);
  pub_marker_crop = nh.advertise<visualization_msgs::Marker>("markers/crop_box", 1);

  // Populate the marker object used to show the centroid of each cluster with values that will never be changed
  marker_obj.type = visualization_msgs::Marker::SPHERE;
  marker_obj.action = visualization_msgs::Marker::ADD;
  marker_obj.lifetime = ros::Duration(0.1);
  marker_obj.pose.orientation.x = 0.0;
  marker_obj.pose.orientation.y = 0.0;
  marker_obj.pose.orientation.z = 0.0;
  marker_obj.pose.orientation.w = 1.0;
  marker_obj.color.a = 1.0;
  marker_obj.scale.x = 0.01;
  marker_obj.scale.y = 0.01;
  marker_obj.scale.z = 0.01;

  // Populate the marker object used to show the Crop Box limits with values that will never be changed
  marker_crop.type = visualization_msgs::Marker::CUBE;
  marker_crop.action = visualization_msgs::Marker::ADD;
  marker_crop.pose.orientation.x = 0.0;
  marker_crop.pose.orientation.y = 0.0;
  marker_crop.pose.orientation.z = 0.0;
  marker_crop.pose.orientation.w = 1.0;
  marker_crop.color.g = 1.0;
  marker_crop.color.a = 0.3;

  ros::spin ();
}
