#pragma once

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h>
#endif

#include <QLineEdit>
#include <QPushButton>

#include "interbotix_xs_msgs/Reboot.h"
#include "interbotix_xs_msgs/RobotInfo.h"
#include "interbotix_xs_msgs/TorqueEnable.h"
#include "interbotix_xs_msgs/OperatingModes.h"
#include "interbotix_xs_msgs/RegisterValues.h"
#include "interbotix_xs_msgs/JointGroupCommand.h"
#include "interbotix_xs_rviz/xs_register_descriptions.h"

namespace Ui
{
class InterbotixControlPanelUI;
} // namespace Ui

namespace interbotix_xs_rviz
{

class InterbotixControlPanel : public rviz::Panel
{

  Q_OBJECT

public:
  InterbotixControlPanel(QWidget* parent = 0);
  ~InterbotixControlPanel();

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

public Q_SLOTS:

  void set_robot_namespace(const QString& robot_namespace);


  /** -------- Torque Tab ---------------- */

  void send_torque_enable_call(bool enable);


  /** -------- Torque Tab ---------------- */

  void send_home_sleep_call();


  /** -------- Reboot Tab ---------------- */

  void send_reboot_call();


  /** -------- Operating Modes Tab ------- */

  void send_opmodes_call();


  /** -------- Get Register Values Tab --- */

  void send_getregval_call();



protected Q_SLOTS:

  void update_robot_namespace();
  void update_robot_info();


  /** -------- Torque Tab ---------------- */

  void torque_change_cmd_type_single();
  void torque_change_cmd_type_group();
  void torque_change_name();
  void torque_enable_torque();
  void torque_disable_torque();
  void torque_init();


  /** -------- Home/Sleep Tab ------------ */

  void homesleep_go_to_home();
  void homesleep_go_to_sleep();
  void homesleep_init();


  /** -------- Reboot Tab ---------------- */

  void reboot_change_cmd_type_single();
  void reboot_change_cmd_type_group();
  void reboot_change_name();
  void reboot_change_smartreboot(bool checked);
  void reboot_change_enable(bool checked);
  void reboot_init();


  /** -------- Operating Modes Tab ------- */

  void opmodes_change_cmd_type_group();
  void opmodes_change_cmd_type_single();
  void opmodes_change_name();
  void opmodes_change_mode(int);
  void opmodes_change_profile_type(int);
  void opmodes_change_profile_vel();
  void opmodes_change_profile_acc();
  void opmodes_init();


  /** -------- Get Register Values Tab --- */

  void getregval_change_cmd_type_group();
  void getregval_change_cmd_type_single();
  void getregval_change_name();
  void getregval_change_reg_name(int);
  void getregval_display(interbotix_xs_msgs::RegisterValues&);
  void getregval_init();


  /** -------- Emergency Stop Tab -------- */

  void estop_button_pressed();


protected:

  // The GUI for this RViz panel
  Ui::InterbotixControlPanelUI* ui_;

  // ROS Node
  ros::NodeHandle nh_;

  // The robot namespace from robot_namespace_editor_;
  std::string robot_namespace_;

  // Robot info service call
  interbotix_xs_msgs::RobotInfo robot_info_call;

  // Vector containing joint info for the arm group
  std::vector<std::string> robot_arm_joints;

  // QStringList containing joint names for the arm group
  QStringList qrobot_arm_joints;

  // QStringList containing the groups in the robot
  QStringList qrobot_groups = QStringList() << "arm" << "gripper" << "all";

  // Robot info service client
  ros::ServiceClient srv_robot_info;

  // enables or disables all input fields depending on the given bool
  void enable_elements(const bool enable);

  // whether or not the panel has been loaded fully
  bool loaded = false;


  /** -------- Torque Enable Tab -------- */

  // The torque_enable service client
  ros::ServiceClient srv_torque_enable;
  
  // The torque enable service call
  interbotix_xs_msgs::TorqueEnable torque_enable_call;


  /** -------- Home/Sleep Tab ------------ */

  // The home/sleep publisher
  ros::Publisher pub_joint_group_cmd;
  
  // The home/sleep jointgroupcommand message
  interbotix_xs_msgs::JointGroupCommand joint_group_cmd;

  // The home/sleep vector storing the home positions
  std::vector<float> homesleep_homevec;

  // The home/sleep vector storing the sleep positions
  std::vector<float> homesleep_sleepvec;


  /** -------- Reboot Tab ---------------- */

  // The reboot_motors service client
  ros::ServiceClient srv_reboot_motors;

  // The reboot motors service call
  interbotix_xs_msgs::Reboot reboot_call;


  /** -------- Operating Modes Tab ------- */

  // The operating_modes service client
  ros::ServiceClient srv_operating_modes;

  // The operating modes service call
  interbotix_xs_msgs::OperatingModes opmodes_call;


  /** -------- Get Register Values Tab --- */

  // The get_motor_registers service client
  ros::ServiceClient srv_get_motor_registers;

  // The get_motor_registers service call
  interbotix_xs_msgs::RegisterValues getreg_call;


  /** -------- Emergency Stop Tab -------- */

  // sends the emergency button service call
  void send_system_call();

}; // InterbotixControlPanel

} // namespace interbotix_control_panel
