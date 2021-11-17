#ifndef XS_HARDWARE_INTERFACE_OBJ_H
#define XS_HARDWARE_INTERFACE_OBJ_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <urdf/model.h>
#include <boost/scoped_ptr.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/lifecycle_state_names.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <hardware_interface/visibility_control.h>

#include <xseries_msgs/msg/joint_group_command.hpp>
#include <xseries_msgs/msg/joint_single_command.hpp>
#include <xseries_msgs/srv/robot_info.hpp>
#include <sensor_msgs/msg/joint_state.hpp>


using hardware_interface::HardwareInfo;
using hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using hardware_interface::HW_IF_POSITION;
class XSHardwareInterface: public hardware_interface::SystemInterface
{
public:

  RCLCPP_SHARED_PTR_DEFINITIONS(XSHardwareInterface);
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CallbackReturn start();

  CallbackReturn stop();

  return_type read() override;

  return_type write() override;


  std::string get_name() const final
  {
    return info_.name;
  }


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
  HardwareInfo info_;
  std::mutex joint_state_mtx_;

  double loop_hz;
  size_t num_joints;
  sensor_msgs::msg::JointState joint_states;
};

#endif
