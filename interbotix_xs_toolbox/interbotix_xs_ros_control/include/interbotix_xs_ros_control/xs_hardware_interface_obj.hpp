#ifndef XS_HARDWARE_INTERFACE_OBJ_H
#define XS_HARDWARE_INTERFACE_OBJ_H

#include <rclcpp/rclcpp.hpp>
#include <urdf/model.h>
#include <boost/scoped_ptr.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_status_values.hpp>
#include <hardware_interface/visibility_control.h>

#include <xseries_msgs/msg/joint_group_command.hpp>
#include <xseries_msgs/msg/joint_single_command.hpp>
#include <xseries_msgs/srv/robot_info.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class XSHardwareInterface: public hardware_interface::SystemInterface
{
public:

  XSHardwareInterface();
  ~XSHardwareInterface();

  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type start() override;

  hardware_interface::return_type stop() override;

  hardware_interface::return_type read() override;

  hardware_interface::return_type write() override;

  void init();

  void joint_state_cb(const sensor_msgs::msg::JointState &msg);

protected:

  std::vector<double> joint_positions;
  std::vector<double> joint_velocities;
  std::vector<double> joint_efforts;
  std::vector<double> joint_position_commands;
  std::vector<float> joint_commands_prev;

  void executor_cb();

  std::thread update_thread;

  std::shared_ptr<rclcpp::Node> nh;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;

  rclcpp::Publisher<xseries_msgs::msg::JointGroupCommand>::SharedPtr pub_group;
  rclcpp::Publisher<xseries_msgs::msg::JointSingleCommand>::SharedPtr pub_gripper;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states;
  rclcpp::Client<xseries_msgs::srv::RobotInfo>::SharedPtr srv_robot_info;

  float gripper_cmd_prev;
  std::string group_name;
  std::string gripper_name;
  std::vector<int16_t> joint_state_indices;

  double loop_hz;
  size_t num_joints;
  sensor_msgs::msg::JointState joint_states;
};

#endif
