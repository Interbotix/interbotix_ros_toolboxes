#include "interbotix_xs_ros_control/xs_hardware_interface_obj.h"

XSHardwareInterface::XSHardwareInterface(ros::NodeHandle& nh) : nh(nh)
{
    init();
    controller_manager.reset(new controller_manager::ControllerManager(this, nh));
    ros::Duration update_freq = ros::Duration(1.0/loop_hz);
    tmr_control_loop = nh.createTimer(update_freq, &XSHardwareInterface::update, this);
}

XSHardwareInterface::~XSHardwareInterface(){}

void XSHardwareInterface::init()
{
  std::string js_topic;
  nh.getParam("hardware_interface/loop_hz", loop_hz);
  nh.getParam("hardware_interface/group_name", group_name);
  nh.getParam("hardware_interface/gripper_name", gripper_name);
  nh.getParam("hardware_interface/joint_states_topic", js_topic);
  pub_group = nh.advertise<interbotix_xs_msgs::JointGroupCommand>("commands/joint_group", 1);
  pub_gripper = nh.advertise<interbotix_xs_msgs::JointSingleCommand>("commands/joint_single", 1);
  sub_joint_states = nh.subscribe(js_topic, 1, &XSHardwareInterface::joint_state_cb, this);
  srv_robot_info = nh.serviceClient<interbotix_xs_msgs::RobotInfo>("get_robot_info");

  interbotix_xs_msgs::RobotInfo group_info_srv, gripper_info_srv;
  group_info_srv.request.cmd_type = "group";
  group_info_srv.request.name = group_name;
  gripper_info_srv.request.cmd_type = "single";
  gripper_info_srv.request.name = gripper_name;
  srv_robot_info.waitForExistence();
  srv_robot_info.call(group_info_srv);
  srv_robot_info.call(gripper_info_srv);

  num_joints = group_info_srv.response.num_joints + 1;
  joint_state_indices = group_info_srv.response.joint_state_indices;
  joint_state_indices.push_back(gripper_info_srv.response.joint_state_indices.at(0));
  std::vector<std::string> joint_names = group_info_srv.response.joint_names;
  joint_names.push_back(gripper_info_srv.response.joint_names.at(0));

  // Resize vectors
  joint_positions.resize(num_joints);
  joint_velocities.resize(num_joints);
  joint_efforts.resize(num_joints);
  joint_position_commands.resize(num_joints);
  joint_commands_prev.resize(num_joints);

  ros::Rate loop_rate(loop_hz);
  while (joint_states.position.size() == 0 && ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
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

void XSHardwareInterface::update(const ros::TimerEvent& e)
{
    elapsed_time = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager->update(ros::Time::now(), elapsed_time);
    write(elapsed_time);
}

void XSHardwareInterface::read()
{
  for (int i = 0; i < num_joints; i++)
  {
    joint_positions.at(i) = joint_states.position.at(joint_state_indices.at(i));
    joint_velocities.at(i) = joint_states.velocity.at(joint_state_indices.at(i));
    joint_efforts.at(i) = joint_states.effort.at(joint_state_indices.at(i));
  }
}

void XSHardwareInterface::write(ros::Duration elapsed_time)
{
  interbotix_xs_msgs::JointGroupCommand group_msg;
  interbotix_xs_msgs::JointSingleCommand gripper_msg;
  group_msg.name = group_name;
  gripper_msg.name = gripper_name;
  gripper_msg.cmd = joint_position_commands.back() * 2;

  position_joint_saturation_interface.enforceLimits(elapsed_time);

  for (size_t i{0}; i < num_joints - 1; i++)
    group_msg.cmd.push_back(joint_position_commands.at(i));

  if (joint_commands_prev != group_msg.cmd)
  {
    pub_group.publish(group_msg);
    joint_commands_prev = group_msg.cmd;
  }
  if (gripper_cmd_prev != gripper_msg.cmd)
  {
    pub_gripper.publish(gripper_msg);
    gripper_cmd_prev = gripper_msg.cmd;
  }
}

void XSHardwareInterface::joint_state_cb(const sensor_msgs::JointState &msg)
{
  joint_states = msg;
}
