// Copyright 2024 Trossen Robotics
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

#include "interbotix_gravity_compensation_obj.hpp"

InterbotixGravityCompensation::InterbotixGravityCompensation(
  bool & success,
  const rclcpp::NodeOptions & options)
: Node("gravity_compensation")
{
  // Declare parameters
  this->declare_parameter("motor_specs", "");

  // Get parameters
  std::string motor_specs;
  this->get_parameter("motor_specs", motor_specs);

  // Create the Subscription for the JointState message
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10,
    std::bind(&InterbotixGravityCompensation::joint_state_cb, this, std::placeholders::_1));

  // Create the Publisher for the JointGroupCommand message
  joint_group_pub_ = this->create_publisher<interbotix_xs_msgs::msg::JointGroupCommand>(
    "commands/joint_group", 10);

  // Create the Service for enabling/disabling gravity compensation
  gravity_compensation_enable_srv_ = this->create_service<std_srvs::srv::SetBool>(
    "gravity_compensation_enable",
    std::bind(
      &InterbotixGravityCompensation::gravity_compensation_enable_cb, this,
      std::placeholders::_1, std::placeholders::_2));

  // Create the client for the 'OperatingModes' service
  operating_modes_client_ = this->create_client<interbotix_xs_msgs::srv::OperatingModes>(
    "set_operating_modes");

  // Create the client for the 'TorqueEnable' service
  torque_enable_client_ = this->create_client<interbotix_xs_msgs::srv::TorqueEnable>(
    "torque_enable");

  // Wait for the 'OperatingModes' service to be available
  while (!operating_modes_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Interrupted while waiting for %s. Exiting.",
        operating_modes_client_->get_service_name());
      success = false;
      return;
    }
    RCLCPP_INFO(
      this->get_logger(),
      "%s not available, waiting again...",
      operating_modes_client_->get_service_name());
  }

  // Wait for the 'TorqueEnable' service to be available
  while (!torque_enable_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Interrupted while waiting for %s. Exiting.",
        torque_enable_client_->get_service_name());
      success = false;
      return;
    }
    RCLCPP_INFO(
      this->get_logger(),
      "%s not available, waiting again...",
      torque_enable_client_->get_service_name());
  }

  // Load the motor specs
  if (!load_motor_specs(motor_specs)) {
    success = false;
    return;
  }

  // Prepare for the inverse dynamics solver
  if (!prepare_tree()) {
    success = false;
    return;
  }

  // Prompt the user to enable gravity compensation
  RCLCPP_INFO(
    this->get_logger(),
    "Gravity compensation node is up."
  );

  // Warn the user that the joints will be turned off temporarily
  RCLCPP_WARN(
    this->get_logger(),
    "WARNING: The joints will be turned off temporarily while enabling/disabling "
    "gravity compensation. Please make sure the robot is in a safe position or "
    "manually held before enabling gravity compensation."
  );
}

void InterbotixGravityCompensation::joint_state_cb(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // If gravity compensation is not enabled, return
  if (!gravity_compensation_enabled_) {
    return;
  }

  // Construct the id solver
  KDL::TreeIdSolver_RNE idsolver(tree_, KDL::Vector(0, 0, -9.81));

  // Set the joint positions and velocities
  for (size_t i = 0; i < msg->position.size(); i++) {
    q_(i) = msg->position[i];
    q_dot_(i) = msg->velocity[i];
  }

  // Compute the torques
  idsolver.CartToJnt(q_, q_dot_, q_dotdot_, f_ext_, torques_);

  // Create a JointGroupCommand message
  interbotix_xs_msgs::msg::JointGroupCommand command_msg;

  // Set the necessary fields in the message
  command_msg.name = "arm";
  command_msg.cmd.resize(num_joints_arm_);
  for (size_t i = 0; i < num_joints_arm_; i++) {
    // Desired current for joint i
    command_msg.cmd[i] = torques_(i) / torque_constants_[i];
    // Pad the no-load current towards moving direction to ease the joint friction
    if (msg->velocity[i] > 0.0) {
      command_msg.cmd[i] += no_load_currents_[i];
    } else {
      command_msg.cmd[i] -= no_load_currents_[i];
    }
    // Convert the current to the current command unit
    command_msg.cmd[i] /= current_units_[i];
  }

  // Publish the message
  joint_group_pub_->publish(command_msg);
}

void InterbotixGravityCompensation::gravity_compensation_enable_cb(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  // Set false to the gravity compensation flag
  gravity_compensation_enabled_ = false;

  if (request->data) {
    // Set the operating mode to 'current' if the joints support it
    set_operating_modes("group", "arm", "current");
    set_operating_modes("single", "gripper", "current");
    std::fill(
      set_operating_modes_requests_.begin(),
      set_operating_modes_requests_.end(),
      true);
  } else {
    // Set the operating mode to 'position' for all joints
    set_operating_modes("group", "arm", "position");
    set_operating_modes("single", "gripper", "current_based_position");
    std::fill(
      set_operating_modes_requests_.begin(),
      set_operating_modes_requests_.end(),
      false);
  }

  // Sleep for a bit to allow the 'OperatingModes' service to process the requests
  rclcpp::sleep_for(std::chrono::seconds(1));

  if (request->data) {
    // Enable the torque for the joints if they support current control mode
    for (size_t i = 0; i < joint_names_.size(); i++) {
      if (i < num_joints_arm_) {
        // Arm joints
        if (torque_constants_[i] == 0.0) {
          torque_enable("single", joint_names_[i], false);
        } else {
          torque_enable("single", joint_names_[i], true);
        }
      } else {
        // Gripper joint
        torque_enable("single", joint_names_[i], false);
      }
      torque_enable_requests_[i] = true;
    }
  } else {
    // Enable the torque for all joints
    for (size_t i = 0; i < joint_names_.size(); i++) {
      torque_enable("single", joint_names_[i], true);
      torque_enable_requests_[i] = false;
    }
  }

  // Set the gravity compensation flag and return the response
  response->success = true;
  response->message = std::string("Gravity Compensation ");
  response->message += request->data ? "Enabled" : "Disabled";
  RCLCPP_INFO(this->get_logger(), response->message.c_str());
}

bool InterbotixGravityCompensation::load_motor_specs(const std::string & motor_specs)
{
  // Load the torque constants and current units from the motor_specs file
  YAML::Node motor_specs_node;
  try {
    motor_specs_node = YAML::LoadFile(motor_specs);
  } catch (YAML::Exception & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to load the motor specs file '%s'! Error: %s",
      motor_specs.c_str(), e.what());
    return false;
  }

  // Load the joint names and joint name to index mapping
  for (size_t i = 0; i < motor_specs_node["joint_names"].size(); i++) {
    std::string joint_name = motor_specs_node["joint_names"][i].as<std::string>();
    joint_names_.push_back(joint_name);
    joint_name_to_index_[joint_name] = i;
  }

  // Load the torque constants, current units, and no load currents
  float all = motor_specs_node["motor_assist"]["all"].as<float>();
  float single = 0.0;
  for (const auto & joint_name : joint_names_) {
    if (motor_specs_node["motor_assist"][joint_name].IsDefined()) {
      // Load the motor specs
      torque_constants_.push_back(
        motor_specs_node["motor_specs"][joint_name]["torque_constant"].as<float>()
      );
      current_units_.push_back(
        motor_specs_node["motor_specs"][joint_name]["current_unit"].as<float>()
      );
      no_load_currents_.push_back(
        motor_specs_node["motor_specs"][joint_name]["no_load_current"].as<float>()
      );

      // Scale the no load current according to the motor assist setting
      if (all == -1) {
        single = motor_specs_node["motor_assist"][joint_name].as<float>();
        if (0 <= single && single <= 1) {
          no_load_currents_.back() *= single;
        } else {
          RCLCPP_WARN(
            this->get_logger(),
            "Motor assist value not in the range [0, 1] for joint %s. Setting it to 0.",
            joint_name.c_str());
          no_load_currents_.back() *= 0;
        }
      } else if (0 <= all && all <= 1) {
        no_load_currents_.back() *= motor_specs_node["motor_assist"]["all"].as<float>();
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Motor assist value not in the range [0, 1] or -1 for all joints. Setting it to 0.");
        no_load_currents_.back() *= 0;
      }
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "Motor specs not found for joint %s in '%s', "
        "because it cannot run in the Current Control Mode. "
        "Its torque will be disabled.",
        joint_name.c_str(), motor_specs.c_str());
      torque_constants_.push_back(-1);
      current_units_.push_back(-1);
      no_load_currents_.push_back(-1);
    }

    // Initialize the operating mode and torque enable request/response flags
    set_operating_modes_requests_.push_back(false);
    torque_enable_requests_.push_back(false);
    set_operating_modes_responses_.push_back(false);
    torque_enable_responses_.push_back(false);
  }

  // Get the number of joints in the 'arm' group
  num_joints_arm_ = joint_names_.size() - 1;

  return true;
}

void InterbotixGravityCompensation::set_operating_modes(
  const std::string & cmd_type,
  const std::string & name,
  const std::string & mode)
{
  // Create a request message for the 'OperatingModes' service
  auto request = std::make_shared<interbotix_xs_msgs::srv::OperatingModes::Request>();
  request->cmd_type = cmd_type;
  request->name = name;
  request->mode = mode;

  // Create the joint index vector
  std::vector<size_t> joint_indices;
  if (cmd_type == "group") {
    for (size_t i = 0; i < num_joints_arm_; i++) {
      joint_indices.push_back(i);
    }
  } else {
    joint_indices.push_back(joint_name_to_index_[name]);
  }

  // Call the 'OperatingModes' service
  std::function<void(
      const rclcpp::Client<interbotix_xs_msgs::srv::OperatingModes>::SharedFuture
    )> callback;
  callback = std::bind(
    &InterbotixGravityCompensation::set_operating_modes_future_done_cb,
    this,
    std::placeholders::_1,
    joint_indices);
  auto future = operating_modes_client_->async_send_request(request, callback);
}

void InterbotixGravityCompensation::set_operating_modes_future_done_cb(
  const rclcpp::Client<interbotix_xs_msgs::srv::OperatingModes>::SharedFuture future,
  const std::vector<size_t> & joint_indices)
{
  // Set the result flag
  for (const auto & joint_index : joint_indices) {
    set_operating_modes_responses_[joint_index] = set_operating_modes_requests_[joint_index];
  }

  // If both the operating mode and torque enable were ready, set the gravity compensation flag
  if (
    std::all_of(
      set_operating_modes_responses_.begin(),
      set_operating_modes_responses_.end(),
      [](bool i) {
        return i;
      }) &&
    std::all_of(
      torque_enable_responses_.begin(),
      torque_enable_responses_.end(),
      [](bool i) {
        return i;
      }))
  {
    gravity_compensation_enabled_ = true;
  } else {
    gravity_compensation_enabled_ = false;
  }
}

void InterbotixGravityCompensation::torque_enable(
  const std::string & cmd_type,
  const std::string & name,
  const bool & enable)
{
  // Create a request message for the 'TorqueEnable' service
  auto request = std::make_shared<interbotix_xs_msgs::srv::TorqueEnable::Request>();
  request->cmd_type = cmd_type;
  request->name = name;
  request->enable = enable;

  // Create the joint index vector
  std::vector<size_t> joint_indices;
  if (cmd_type == "group") {
    for (size_t i = 0; i < num_joints_arm_; i++) {
      joint_indices.push_back(i);
    }
  } else {
    joint_indices.push_back(joint_name_to_index_[name]);
  }

  // Call the 'TorqueEnable' service
  std::function<void(
      const rclcpp::Client<interbotix_xs_msgs::srv::TorqueEnable>::SharedFuture
    )> callback;
  callback = std::bind(
    &InterbotixGravityCompensation::torque_enable_future_done_cb,
    this,
    std::placeholders::_1,
    joint_indices);
  auto future = torque_enable_client_->async_send_request(request, callback);
}

void InterbotixGravityCompensation::torque_enable_future_done_cb(
  const rclcpp::Client<interbotix_xs_msgs::srv::TorqueEnable>::SharedFuture future,
  const std::vector<size_t> & joint_indices)
{
  // Set the result flag
  for (const auto & joint_index : joint_indices) {
    torque_enable_responses_[joint_index] = torque_enable_requests_[joint_index];
  }

  // If both the operating mode and torque enable were ready, set the gravity compensation flag
  if (
    std::all_of(
      set_operating_modes_responses_.begin(),
      set_operating_modes_responses_.end(),
      [](bool i) {
        return i;
      }) &&
    std::all_of(
      torque_enable_responses_.begin(),
      torque_enable_responses_.end(),
      [](bool i) {
        return i;
      }))
  {
    gravity_compensation_enabled_ = true;
  } else {
    gravity_compensation_enabled_ = false;
  }
}

bool InterbotixGravityCompensation::prepare_tree()
{
  // Create the client to get the robot description string
  auto param_client = std::make_shared<rclcpp::SyncParametersClient>(
    this, "robot_state_publisher"
  );

  // Wait for the robot description string to be available
  while (!param_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Interrupted while waiting for robot_state_publisher. Exiting.");
      return false;
    }
    RCLCPP_INFO(
      this->get_logger(),
      "robot_state_publisher not available, waiting again...");
  }

  // Get the robot description string
  std::string robot_desc_string = param_client->get_parameter<std::string>("robot_description");

  // Parse the robot description string to get the KDL tree
  if (!kdl_parser::treeFromString(robot_desc_string, tree_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse the robot description string!");
    rclcpp::shutdown();
    return false;
  } else {
    RCLCPP_INFO(this->get_logger(), "Successfully parsed the robot description string!");
  }

  // Resize the joint arrays
  q_.resize(tree_.getNrOfJoints());
  q_dot_.resize(tree_.getNrOfJoints());
  q_dotdot_.resize(tree_.getNrOfJoints());
  torques_.resize(tree_.getNrOfJoints());

  return true;
}
