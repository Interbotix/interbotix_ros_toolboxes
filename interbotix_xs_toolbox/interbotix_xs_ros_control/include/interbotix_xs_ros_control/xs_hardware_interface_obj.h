#ifndef XS_HARDWARE_INTERFACE_OBJ_H
#define XS_HARDWARE_INTERFACE_OBJ_H

#include <ros/ros.h>
#include <urdf/model.h>
#include <boost/scoped_ptr.hpp>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <controller_manager/controller_manager.h>
#include "interbotix_xs_sdk/JointGroupCommand.h"
#include "interbotix_xs_sdk/JointSingleCommand.h"
#include "interbotix_xs_sdk/RobotInfo.h"
#include <sensor_msgs/JointState.h>

class XSHardwareInterface: public hardware_interface::RobotHW
{
public:

  XSHardwareInterface(ros::NodeHandle& nh);
  ~XSHardwareInterface();
  void init();
  void update(const ros::TimerEvent& e);
  void read();
  void write(ros::Duration elapsed_time);
  void joint_state_cb(const sensor_msgs::JointState &msg);

protected:

  // Command Interfaces
  hardware_interface::JointStateInterface joint_state_interface;
  hardware_interface::PositionJointInterface position_joint_interface;

  // Limit Interfaces
  joint_limits_interface::PositionJointSaturationInterface position_joint_saturation_interface;

  std::vector<double> joint_positions;
  std::vector<double> joint_velocities;
  std::vector<double> joint_efforts;
  std::vector<double> joint_position_commands;
  std::vector<float> joint_commands_prev;

  ros::NodeHandle nh;
  ros::Publisher pub_group;
  ros::Publisher pub_gripper;
  ros::Subscriber sub_joint_states;
  ros::ServiceClient srv_robot_info;
  ros::Timer tmr_control_loop;
  ros::Duration elapsed_time;
  float gripper_cmd_prev;
  std::string group_name;
  std::string gripper_name;
  std::vector<int16_t> joint_state_indices;

  double loop_hz;
  size_t num_joints;
  sensor_msgs::JointState joint_states;
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager;
};

#endif
