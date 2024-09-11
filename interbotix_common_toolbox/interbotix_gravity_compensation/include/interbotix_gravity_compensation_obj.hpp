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

#ifndef INTERBOTIX_GRAVITY_COMPENSATION_OBJ_HPP_
#define INTERBOTIX_GRAVITY_COMPENSATION_OBJ_HPP_

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
#include <kdl/treeidsolver_recursive_newton_euler.hpp>
#include <kdl/jntarray.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "interbotix_xs_msgs/msg/joint_group_command.hpp"
#include "interbotix_xs_msgs/srv/operating_modes.hpp"
#include "interbotix_xs_msgs/srv/torque_enable.hpp"

class InterbotixGravityCompensation : public rclcpp::Node
{
public:
  /// @brief Constructor for the InterbotixGravityCompensation class
  InterbotixGravityCompensation();

private:
  // subscribers, publishers, servers, and clients, if any
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<interbotix_xs_msgs::msg::JointGroupCommand>::SharedPtr joint_group_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr gravity_compensation_enable_srv_;
  rclcpp::Client<interbotix_xs_msgs::srv::OperatingModes>::SharedPtr operating_modes_client_;
  rclcpp::Client<interbotix_xs_msgs::srv::TorqueEnable>::SharedPtr torque_enable_client_;

  // member functions
  /// @brief Set the operating mode of a joint or a group of joints.
  /// @param cmd_type - the type of command being sent; either 'single' or 'group'
  /// @param name - the name of the joint or group of joints
  /// @param mode - the desired operating mode
  void set_operating_modes(
    const std::string & cmd_type,
    const std::string & name,
    const std::string & mode
  );

  /// @brief Callback function for the 'OperatingModes' service.
  /// @param future - the future object containing the response from the service
  /// @param joint_indices - indices of the joints whose modes were set
  /// @details This function sets the 'set_operating_modes_responses_' flag to the requested value.
  ///    If all flags are true, it sets the 'gravity_compensation_enabled_' flag to true.
  void set_operating_modes_callback(
    const rclcpp::Client<interbotix_xs_msgs::srv::OperatingModes>::SharedFuture future,
    const std::vector<size_t> & joint_indices
  );

  /// @brief Enable or disable torque on a joint.
  /// @param cmd_type - the type of command being sent; either 'single' or 'group'
  /// @param name - the name of the joint or group of joints
  /// @param enable - true to enable torque; false to disable torque
  void torque_enable(
    const std::string & cmd_type,
    const std::string & name,
    const bool & enable
  );

  /// @brief Callback function for the 'TorqueEnable' service.
  /// @param future - the future object containing the response from the service
  /// @param joint_indices - indices of the joints whose torque was enabled/disabled
  /// @details This function sets the 'torque_enable_responses_' flag to the requested value.
  ///    If all flags are true, it sets the 'gravity_compensation_enabled_' flag to true.
  void torque_enable_callback(
    const rclcpp::Client<interbotix_xs_msgs::srv::TorqueEnable>::SharedFuture future,
    const std::vector<size_t> & joint_indices
  );

  /// @brief Callback function for the joint_state_sub_ subscriber.
  /// @param msg - the incoming JointState message
  /// @details This function computes the current commands needed to compensate for gravity and
  ///     publishes them to the <robot_name>/commands/joint_group topic.
  void joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg);

  /// @brief Callback function for the 'gravity_compensation_enable' service.
  /// @param request - the incoming request to enable or disable gravity compensation
  /// @param response - the response to the request
  /// @details This function enables or disables gravity compensation based on the request.
  void gravity_compensation_enable_cb(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response
  );

  /// @brief Load motor specs from a YAML file.
  /// @param motor_specs - the path to the YAML file containing motor specs
  /// @details This function loads the motor specs from the YAML file and initializes the
  ///     motor specs vectors. It also initializes the operating mode and torque enable
  ///     request/response flags and sets the number of joints in the 'arm' group.
  void load_motor_specs(const std::string & motor_specs);

  /// @brief Prepare the KDL tree for the inverse dynamics solver.
  /// @param robot_name - name of the robot (typically equal to `robot_model`,
  ///     but could be anything), corresponding to the topic prefix.
  void prepare_tree(std::string robot_name);

  // member variables
  bool gravity_compensation_enabled_ = false;

  size_t num_joints_arm_;
  std::unordered_map<std::string, size_t> joint_name_to_index_;
  std::vector<std::string> joint_names_;
  std::vector<float> torque_constants_;
  std::vector<float> current_units_;
  std::vector<float> no_load_currents_;
  std::vector<bool> set_operating_modes_requests_;
  std::vector<bool> torque_enable_requests_;
  std::vector<bool> set_operating_modes_responses_;
  std::vector<bool> torque_enable_responses_;

  KDL::Tree tree_;
};

#endif  // INTERBOTIX_GRAVITY_COMPENSATION_OBJ_HPP_
