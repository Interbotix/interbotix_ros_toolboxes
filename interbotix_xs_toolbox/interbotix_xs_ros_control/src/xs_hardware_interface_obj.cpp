#include <interbotix_xs_ros_control/xs_hardware_interface_obj.hpp>

XSHardwareInterface::XSHardwareInterface(){
    init();
}

void XSHardwareInterface::executor_cb(){
  RCLCPP_INFO(nh->get_logger(), "SPINNING");
  executor->spin_some();
}

XSHardwareInterface::~XSHardwareInterface(){}

void XSHardwareInterface::init()
{
  nh = std::make_shared<rclcpp::Node>();
  executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  std::string js_topic;
  pub_group = nh->create_publisher<xseries_msgs::msg::JointGroupCommand>("commands/joint_group", 10);
  pub_gripper =  nh->create_publisher<xseries_msgs::msg::JointSingleCommand>("commands/joint_single", 10);
  sub_joint_states =  nh->create_subscription<sensor_msgs::msg::JointState>("joint_states", 1, std::bind(&XSHardwareInterface::joint_state_cb, this));
  srv_robot_info =nh->create_client<xseries_msgs::srv::RobotInfo>("get_robot_info");
  xseries_msgs::srv::RobotInfo::Request::SharedPtr group_info_srv, gripper_info_srv;
  update_thread = std::thread(executor_cb);
  group_info_srv->cmd_type = "group";
  group_info_srv->name = group_name;
  gripper_info_srv->cmd_type = "single";
  gripper_info_srv->name = gripper_name;
  using namespace std::chrono_literals;
  srv_robot_info->wait_for_service(500ms);
  auto group_res = srv_robot_info->async_send_request(group_info_srv);
  auto gripper_res = srv_robot_info->async_send_request(gripper_info_srv);
  num_joints = group_res.get()->num_joints + 1;
  joint_state_indices = group_res.get()->joint_state_indices;
  joint_state_indices.push_back(gripper_res.get()->joint_state_indices.at(0));
  std::vector<std::string> joint_names = group_res.get()->joint_names;
  joint_names.push_back(gripper_res.get()->joint_names.at(0));

  // Resize vectors
  joint_positions.resize(num_joints);
  joint_velocities.resize(num_joints);
  joint_efforts.resize(num_joints);
  joint_position_commands.resize(num_joints);
  joint_commands_prev.resize(num_joints);

  while (joint_states.position.size() == 0 && rclcpp::ok())
  {
    RCLCPP_INFO(nh->get_logger(), "WAITING FOR JOINT STATES")
  }

  // Initialize the joint_position_commands vector to the current joint states
  for (size_t i{0}; i < num_joints; i++)
  {
    joint_position_commands.at(i) = joint_states.position.at(joint_state_indices.at(i));
    joint_commands_prev.at(i) = joint_position_commands.at(i);
  }
  joint_commands_prev.resize(num_joints - 1);
  gripper_cmd_prev = joint_states.position.at(joint_state_indices.back()) * 2;

  urdf::Model model;
  std::string robot_name = nh.getNamespace();
  urdf::JointConstSharedPtr ptr;
  model.initParam(robot_name + "/robot_description");

  // Initialize Controller
  for (int i = 0; i < num_joints; ++i) {
     // Create joint state interface
    hardware_interface::JointStateHandle jointStateHandle(joint_names.at(i), &joint_positions.at(i), &joint_velocities.at(i), &joint_efforts.at(i));
    joint_state_interface.registerHandle(jointStateHandle);

    joint_limits_interface::JointLimits limits;
    ptr = model.getJoint(joint_names.at(i));
    getJointLimits(ptr, limits);
    getJointLimits(joint_names.at(i), nh, limits);

    // Create position joint interface
    hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_commands.at(i));
    joint_limits_interface::PositionJointSaturationHandle jointPositionSaturationHandle(jointPositionHandle, limits);
    position_joint_saturation_interface.registerHandle(jointPositionSaturationHandle);
    position_joint_interface.registerHandle(jointPositionHandle);
  }

  registerInterface(&joint_state_interface);
  registerInterface(&position_joint_interface);
  registerInterface(&position_joint_saturation_interface);
}


hardware_interface::return_type XSHardwareInterface::configure(const hardware_interface::HardwareInfo & info) {

  }


hardware_interface::return_type XSHardwareInterface::read()
{
  for (int i = 0; i < num_joints; i++)
  {
    joint_positions.at(i) = joint_states.position.at(joint_state_indices.at(i));
    joint_velocities.at(i) = joint_states.velocity.at(joint_state_indices.at(i));
    joint_efforts.at(i) = joint_states.effort.at(joint_state_indices.at(i));
  }
}

hardware_interface::return_type XSHardwareInterface::write()
{
  xseries_msgs::msg::JointGroupCommand group_msg;
  xseries_msgs::msg::JointSingleCommand gripper_msg;
  group_msg.name = group_name;
  gripper_msg.name = "gripper";
  gripper_msg.cmd = joint_position_commands.back() * 2;


  for (size_t i{0}; i < num_joints - 1; i++)
    group_msg.cmd.push_back(joint_position_commands.at(i));

  if (joint_commands_prev != group_msg.cmd)
  {
    pub_group->publish(group_msg);
    joint_commands_prev = group_msg.cmd;
  }
  if (gripper_cmd_prev != gripper_msg.cmd)
  {
    pub_gripper->publish(gripper_msg);
    gripper_cmd_prev = gripper_msg.cmd;
  }
}

void XSHardwareInterface::joint_state_cb(const sensor_msgs::msg::JointState &msg)
{
  joint_states = msg;
}
