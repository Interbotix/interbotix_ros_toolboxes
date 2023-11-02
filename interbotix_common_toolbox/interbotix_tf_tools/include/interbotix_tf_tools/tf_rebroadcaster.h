#ifndef _TF_REBROADCASTER_H_
#define _TF_REBROADCASTER_H_

#include <string>
#include <vector>

#include "geometry_msgs/TransformStamped.h"
#include "ros/ros.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "yaml-cpp/yaml.h"

struct Frame
{
  std::string parent_frame_id;
  std::string child_frame_id;
  std::string prefix;
  bool logged = false;
};

class TFRebroadcaster
{
public:
  explicit TFRebroadcaster(ros::NodeHandle *node_handle);

private:
  void tf_cb(const tf2_msgs::TFMessage & msg);

  // ROS Node handle
  ros::NodeHandle node;

  // Subscription to the topic to get TFs from
  ros::Subscriber sub_tf_;

  // Publisher to the new topic
  ros::Publisher pub_tf_;

  // Absolute filepath to the configuration file
  std::string filepath_config_;

  // String containing the topic that the rebroadcaster should listen to TFs from
  std::string topic_from_;

  // String containing the topic that the rebroadcaster should publish to
  std::string topic_to_;

  // Whether or not to use the timestamp on the incoming TF
  bool use_incoming_time_;

  // Whether or not to set the outgoing TF's z translation to zero. This is useful to correct for
  // drift in the z direction.
  bool set_zero_z_translation_;

  // YAML node containing configuration info
  YAML::Node config_;

  // Vector containing all frames to be re-broadcasted from the configuration
  std::vector<Frame> frames_;
};

#endif  // _TF_REBROADCASTER_H_
