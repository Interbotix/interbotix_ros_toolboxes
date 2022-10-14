// Copyright 2022 Trossen Robotics
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "interbotix_tf_tools/tf_rebroadcaster.hpp"

#include <string>

using namespace std::chrono_literals;

namespace interbotix_tf_tools
{

TFRebroadcaster::TFRebroadcaster(const rclcpp::NodeOptions & options)
: Node("tf_rebroadcaster", options)
{
  // Declare and get parameters
  this->declare_parameter<std::string>("filepath_config");
  this->declare_parameter<std::string>("topic_to");
  this->declare_parameter<std::string>("topic_from");
  this->get_parameter("filepath_config", filepath_config_);
  this->get_parameter("topic_to", topic_to_);
  this->get_parameter("topic_from", topic_from_);

  // Load configuration file
  try {
    // try to load motor_configs yaml file
    config_ = YAML::LoadFile(filepath_config_.c_str());
  } catch (YAML::BadFile & error) {
    // if file is not found or a bad format, shut down
    RCLCPP_FATAL(
      this->get_logger(),
      "Config file at '%s' was not found or has a bad format. Shutting down...",
      filepath_config_.c_str());
    RCLCPP_FATAL(this->get_logger(), "YAML Error: '%s'", error.what());
    return;
  }

  // Print out configuration info
  RCLCPP_INFO(
    this->get_logger(),
    "Will broadcast TFs from topic '%s' to topic '%s'.",
    topic_from_.c_str(),
    topic_to_.c_str());

  // Get frames config
  YAML::Node all_frames = config_["frames"];
  for (
    YAML::const_iterator frame_itr = all_frames.begin();
    frame_itr != all_frames.end();
    frame_itr++)
  {
    // Get all parent and child frames, insert into map
    std::string frame_parent = frame_itr->first.as<std::string>();
    std::string frame_child = frame_itr->second["child_frame_id"].as<std::string>();
    std::string prefix = frame_itr->second["prefix"].as<std::string>();
    frames_.push_back(Frame{frame_parent, frame_child, prefix});

    if (prefix == "") {
      RCLCPP_INFO(
        this->get_logger(),
        "Will broadcast TF from frame '%s' to frame '%s'.",
        frame_parent.c_str(),
        frame_child.c_str());
    } else {
      // Print out which frames will be rebroadcasted
      RCLCPP_INFO(
        this->get_logger(),
        "Will broadcast TF from frame '%s' to frame '%s', prepending prefix '%s'.",
        frame_parent.c_str(),
        frame_child.c_str(),
        prefix.c_str());
    }
  }

  // Create pubs and subs
  using namespace std::placeholders;
  sub_tf_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
    topic_from_,
    10,
    std::bind(&TFRebroadcaster::tf_cb, this, _1));
  pub_tf_ = this->create_publisher<tf2_msgs::msg::TFMessage>(topic_to_, 10);
}

void TFRebroadcaster::tf_cb(const tf2_msgs::msg::TFMessage & msg)
{
  // TODO(lsinterbotix): optimize
  // Loop over each parent/child frame
  for (const auto & frame : frames_) {
    // Loop over every transform in message
    for (auto & tf : msg.transforms) {
      // Check if parent and child match
      if (
        frame.parent_frame_id == tf.header.frame_id &&
        frame.child_frame_id == tf.child_frame_id)
      {
        // If they do match, rebroadcast to new topic, prepending the prefix if necessary
        auto tf_ = tf;
        tf2_msgs::msg::TFMessage rebroadcast_tf;
        if (frame.prefix != "") {
          tf_.child_frame_id = frame.prefix + tf.child_frame_id;
          tf_.header.frame_id = frame.prefix + tf.header.frame_id;
        }
        rebroadcast_tf.transforms.push_back(tf_);
        pub_tf_->publish(rebroadcast_tf);
      }
    }
  }
}

}  // namespace interbotix_tf_tools

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(interbotix_tf_tools::TFRebroadcaster)
