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

#include <memory>
#include <string>
#include <vector>

#include "QLineEdit"
#include "QVBoxLayout"
#include "QHBoxLayout"
#include "QLabel"

#include "interbotix_xs_rviz/interbotix_control_panel.hpp"

#include "ui_interbotix_control_panel.h"  // NOLINT

using namespace std::chrono_literals;

namespace interbotix_xs_rviz
{

InterbotixControlPanel::InterbotixControlPanel(QWidget * parent)
: rviz_common::Panel::Panel(parent),
  ui_(new Ui::InterbotixControlPanelUI())
{
}

void InterbotixControlPanel::onInitialize()
{
  // Set up the RViz panel
  ui_->setupUi(this);

  // Connect Qt signals to slots

  // robot namespace
  connect(
    ui_->pushbutton_robot_namespace_, SIGNAL(clicked()), this, SLOT(update_robot_namespace()));

  // torque
  connect(
    ui_->combobox_torque_name_, SIGNAL(currentTextChanged(const QString)), this,
    SLOT(torque_change_name()));
  connect(ui_->button_torque_enable_, SIGNAL(clicked()), this, SLOT(torque_enable_torque()));
  connect(ui_->button_torque_disable_, SIGNAL(clicked()), this, SLOT(torque_disable_torque()));
  connect(
    ui_->radiobutton_torque_group_, SIGNAL(toggled(bool)), this,
    SLOT(torque_change_cmd_type_group()));
  connect(
    ui_->radiobutton_torque_single_, SIGNAL(toggled(bool)), this,
    SLOT(torque_change_cmd_type_single()));

  // home/sleep
  connect(ui_->button_gotohome_, SIGNAL(clicked()), this, SLOT(homesleep_go_to_home()));
  connect(ui_->button_gotosleep_, SIGNAL(clicked()), this, SLOT(homesleep_go_to_sleep()));

  // reboot
  connect(
    ui_->combobox_reboot_name_, SIGNAL(currentTextChanged(const QString)), this,
    SLOT(reboot_change_name()));
  connect(
    ui_->radiobutton_reboot_group_, SIGNAL(toggled(bool)), this,
    SLOT(reboot_change_cmd_type_group()));
  connect(
    ui_->radiobutton_reboot_single_, SIGNAL(toggled(bool)), this,
    SLOT(reboot_change_cmd_type_single()));
  connect(
    ui_->checkbox_smart_reboot_, SIGNAL(toggled(bool)), this,
    SLOT(reboot_change_smartreboot(bool)));
  connect(
    ui_->checkbox_reboot_enable_, SIGNAL(toggled(bool)), this, SLOT(
      reboot_change_enable(
        bool)));
  connect(ui_->button_reboot_reboot_, SIGNAL(clicked()), this, SLOT(send_reboot_call()));

  // operating modes
  connect(
    ui_->combobox_opmodes_name_, SIGNAL(currentTextChanged(const QString)), this,
    SLOT(opmodes_change_name()));
  connect(
    ui_->radiobutton_opmodes_group_, SIGNAL(toggled(bool)), this,
    SLOT(opmodes_change_cmd_type_group()));
  connect(
    ui_->radiobutton_opmodes_single_, SIGNAL(toggled(bool)), this,
    SLOT(opmodes_change_cmd_type_single()));
  connect(
    ui_->combobox_opmodes_mode_, SIGNAL(currentIndexChanged(int)), this,
    SLOT(opmodes_change_mode(int)));
  connect(
    ui_->combobox_opmodes_profile_type_, SIGNAL(currentIndexChanged(int)), this,
    SLOT(opmodes_change_profile_type(int)));
  connect(
    ui_->lineedit_opmodes_profile_vel_, SIGNAL(editingFinished()), this,
    SLOT(opmodes_change_profile_vel()));
  connect(
    ui_->lineedit_opmodes_profile_acc_, SIGNAL(editingFinished()), this,
    SLOT(opmodes_change_profile_acc()));
  connect(ui_->button_opmodes_set_, SIGNAL(clicked()), this, SLOT(send_opmodes_call()));

  // get registers
  connect(
    ui_->combobox_getregval_name_, SIGNAL(currentTextChanged(const QString)), this,
    SLOT(getregval_change_name()));
  connect(
    ui_->radiobutton_getregval_group_, SIGNAL(toggled(bool)), this,
    SLOT(getregval_change_cmd_type_group()));
  connect(
    ui_->radiobutton_getregval_single_, SIGNAL(toggled(bool)), this,
    SLOT(getregval_change_cmd_type_single()));
  connect(
    ui_->combobox_getregval_reg_, SIGNAL(currentIndexChanged(int)), this,
    SLOT(getregval_change_reg_name(int)));
  connect(ui_->button_getregval_val_, SIGNAL(clicked()), this, SLOT(send_getregval_call()));

  // e-stop
  // connect(ui_->button_estop_, SIGNAL(clicked()), this, SLOT(estop_button_pressed()));
  // disable e-stop
  ui_->tabs_->removeTab(ui_->tabs_->indexOf(ui_->tab_estop_));

  // initialize the node and some pubs and services
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  client_node_ = std::make_shared<rclcpp::Node>("_", rclcpp::NodeOptions());
  torque_enable_req = std::make_shared<TorqueEnable::Request>();
  robot_info_req = std::make_shared<RobotInfo::Request>();
  reboot_req = std::make_shared<Reboot::Request>();
  opmodes_req = std::make_shared<OperatingModes::Request>();
  getreg_req = std::make_shared<RegisterValues::Request>();

  robot_info_req->cmd_type = CMD_TYPE_GROUP;
  robot_info_req->name = NAME_ALL;

  torque_enable_req->cmd_type = CMD_TYPE_GROUP;
  torque_enable_req->name = NAME_ARM;

  joint_group_cmd.name = NAME_ALL;

  reboot_req->cmd_type = CMD_TYPE_GROUP;
  reboot_req->name = NAME_ARM;
  reboot_req->smart_reboot = false;
  reboot_req->enable = true;

  opmodes_req->cmd_type = CMD_TYPE_GROUP;
  opmodes_req->name = NAME_ARM;
  opmodes_req->mode = "position";
  opmodes_req->profile_type = "time";
  opmodes_req->profile_velocity = 2000;
  opmodes_req->profile_acceleration = 300;

  getreg_req->cmd_type = CMD_TYPE_GROUP;
  getreg_req->name = NAME_ARM;
  getreg_req->reg = "Goal_Position";

  ui_->label_getregval_desc_->setText(
    QString(
      xs_register_descriptions::descriptions[
        ui_->combobox_getregval_reg_->currentText().toStdString()
      ].description.c_str()
  ));

  RCLCPP_INFO(LOGGER, "Successfully initialized InterbotixControlPanel!");
}

InterbotixControlPanel::~InterbotixControlPanel()
{
  delete ui_;
}

void InterbotixControlPanel::onEnable()
{
  show();
  parentWidget()->show();
}

void InterbotixControlPanel::onDisable()
{
  hide();
  parentWidget()->hide();
}

void InterbotixControlPanel::update_robot_namespace()
{
  if (set_robot_namespace(ui_->lineedit_robot_namespace_->text())) {
    enable_elements(true);
    update_robot_info();
    Q_EMIT configChanged();
  } else {
    RCLCPP_DEBUG(
      LOGGER, "Updated namespace to '%s'.",
      ui_->lineedit_robot_namespace_->text().toStdString().c_str());
    enable_elements(false);
  }
}

bool InterbotixControlPanel::set_robot_namespace(const QString & robot_namespace)
{
  robot_namespace_ = robot_namespace.toStdString();
  // start all service clients, publishers, and subscribers
  try {
    pub_joint_group_cmd = client_node_->create_publisher<JointGroupCommand>(
      "/" + robot_namespace_ + "/commands/joint_group", 1);
    srv_torque_enable = client_node_->create_client<TorqueEnable>(
      "/" + robot_namespace_ + "/torque_enable");
    srv_operating_modes = client_node_->create_client<OperatingModes>(
      "/" + robot_namespace_ + "/set_operating_modes");
    srv_reboot_motors = client_node_->create_client<Reboot>(
      "/" + robot_namespace_ + "/reboot_motors");
    srv_get_motor_registers = client_node_->create_client<RegisterValues>(
      "/" + robot_namespace_ + "/get_motor_registers");
    srv_robot_info = client_node_->create_client<RobotInfo>(
      "/" + robot_namespace_ + "/get_robot_info");
  } catch (const rclcpp::exceptions::NameValidationError & e) {
    RCLCPP_DEBUG(
      LOGGER,
      "Robot namespace '%s' is invalid.", robot_namespace_.c_str());
    return false;
  }
  if (!srv_torque_enable->wait_for_service((loaded) ? 0.1s : 2.0s)) {
    RCLCPP_DEBUG(
      LOGGER,
      "Could not find services under robot namespace '%s'.", robot_namespace_.c_str());
    return false;
  }
  return true;
}

void InterbotixControlPanel::update_robot_info()
{
  // send RobotInfo req and spin until complete
  auto result_robot_info = srv_robot_info->async_send_request(robot_info_req);
  if (rclcpp::spin_until_future_complete(
      client_node_, result_robot_info,
      100ms) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(LOGGER, "Failed to get robot_info service response.");
    return;
  }
  // fill home and sleep positions
  homesleep_homevec.resize(result_robot_info.get()->num_joints);
  std::fill(homesleep_homevec.begin(), homesleep_homevec.end(), 0.0f);
  homesleep_sleepvec.resize(result_robot_info.get()->num_joints);
  homesleep_sleepvec = result_robot_info.get()->joint_sleep_positions;

  // fill joint and group vectors
  robot_arm_joints.clear();
  qrobot_arm_joints.clear();
  robot_groups.clear();
  qrobot_groups.clear();
  for (const auto joint_name : result_robot_info.get()->joint_names) {
    robot_arm_joints.push_back(joint_name);
    qrobot_arm_joints << QString(joint_name.c_str());
  }
  for (const auto group_name : result_robot_info.get()->name) {
    RCLCPP_DEBUG(LOGGER, "Found robot group '%s'.", group_name.c_str());
    robot_groups.push_back(group_name);
    qrobot_groups << QString(group_name.c_str());
  }
}

void InterbotixControlPanel::enable_elements(const bool enable)
{
  // torque
  ui_->button_torque_enable_->setEnabled(enable);
  ui_->button_torque_disable_->setEnabled(enable);
  ui_->radiobutton_torque_single_->setEnabled(enable);
  ui_->radiobutton_torque_group_->setEnabled(enable);
  ui_->combobox_torque_name_->setEnabled(enable);

  // home/sleep
  ui_->button_gotohome_->setEnabled(enable);
  ui_->button_gotosleep_->setEnabled(enable);

  // reboot
  ui_->radiobutton_reboot_single_->setEnabled(enable);
  ui_->radiobutton_reboot_group_->setEnabled(enable);
  ui_->button_reboot_reboot_->setEnabled(enable);
  ui_->checkbox_smart_reboot_->setEnabled(enable);
  ui_->checkbox_reboot_enable_->setEnabled(enable);
  ui_->combobox_reboot_name_->setEnabled(enable);

  // operating modes
  ui_->radiobutton_opmodes_single_->setEnabled(enable);
  ui_->radiobutton_opmodes_group_->setEnabled(enable);
  ui_->combobox_opmodes_mode_->setEnabled(enable);
  ui_->combobox_opmodes_profile_type_->setEnabled(enable);
  ui_->lineedit_opmodes_profile_vel_->setEnabled(enable);
  ui_->lineedit_opmodes_profile_acc_->setEnabled(enable);
  ui_->button_opmodes_set_->setEnabled(enable);
  ui_->combobox_opmodes_name_->setEnabled(enable);

  // getregval
  ui_->radiobutton_getregval_single_->setEnabled(enable);
  ui_->radiobutton_getregval_group_->setEnabled(enable);
  ui_->combobox_getregval_reg_->setEnabled(enable);
  ui_->button_getregval_val_->setEnabled(enable);
  ui_->lineedit_getregval_val_->setEnabled(enable);
  ui_->combobox_getregval_name_->setEnabled(enable);

  // estop
  ui_->button_estop_->setEnabled(false);
}


// -------------------------- TORQUE ------------------------------------------

void InterbotixControlPanel::torque_change_cmd_type_group()
{
  if (ui_->radiobutton_torque_group_->isChecked()) {
    torque_enable_req->cmd_type = CMD_TYPE_GROUP;
    ui_->combobox_torque_name_->clear();
    ui_->combobox_torque_name_->addItems(qrobot_groups);
  }
}

void InterbotixControlPanel::torque_change_cmd_type_single()
{
  if (ui_->radiobutton_torque_single_->isChecked()) {
    torque_enable_req->cmd_type = CMD_TYPE_SINGLE;
    ui_->combobox_torque_name_->clear();
    ui_->combobox_torque_name_->addItems(qrobot_arm_joints);
  }
}

void InterbotixControlPanel::torque_change_name()
{
  torque_enable_req->name = ui_->combobox_torque_name_->currentText().toStdString();
}

void InterbotixControlPanel::torque_enable_torque()
{
  send_torque_enable_call(true);
}

void InterbotixControlPanel::torque_disable_torque()
{
  send_torque_enable_call(false);
}

void InterbotixControlPanel::send_torque_enable_call(bool enable)
{
  torque_enable_req->enable = enable;
  auto result = srv_torque_enable->async_send_request(torque_enable_req);
  if (rclcpp::spin_until_future_complete(
      client_node_, result,
      100ms) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(LOGGER, "Failed to call torque_enable.");
  }
}


// -------------------------- HOME/SLEEP --------------------------------------

void InterbotixControlPanel::homesleep_go_to_home()
{
  joint_group_cmd.cmd = homesleep_homevec;
  pub_joint_group_cmd->publish(joint_group_cmd);
}

void InterbotixControlPanel::homesleep_go_to_sleep()
{
  joint_group_cmd.cmd = homesleep_sleepvec;
  pub_joint_group_cmd->publish(joint_group_cmd);
}


// --------------------------  REBOOT -----------------------------------------

void InterbotixControlPanel::reboot_change_cmd_type_group()
{
  if (ui_->radiobutton_reboot_group_->isChecked()) {
    reboot_req->cmd_type = CMD_TYPE_GROUP;
    ui_->combobox_reboot_name_->clear();
    ui_->combobox_reboot_name_->addItems(qrobot_groups);
  }
}

void InterbotixControlPanel::reboot_change_cmd_type_single()
{
  if (ui_->radiobutton_reboot_single_->isChecked()) {
    reboot_req->cmd_type = CMD_TYPE_SINGLE;
    ui_->combobox_reboot_name_->clear();
    ui_->combobox_reboot_name_->addItems(qrobot_arm_joints);
  }
}

void InterbotixControlPanel::reboot_change_name()
{
  reboot_req->name = ui_->combobox_reboot_name_->currentText().toStdString();
}

void InterbotixControlPanel::reboot_change_smartreboot(bool checked)
{
  reboot_req->smart_reboot = checked;
}

void InterbotixControlPanel::reboot_change_enable(bool checked)
{
  reboot_req->enable = checked;
}

void InterbotixControlPanel::send_reboot_call()
{
  auto result = srv_reboot_motors->async_send_request(reboot_req);
  if (rclcpp::spin_until_future_complete(
      client_node_, result,
      100ms) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(LOGGER, "Failed to call reboot.");
  }
}


// --------------------------  OpModes ----------------------------------------

void InterbotixControlPanel::opmodes_change_cmd_type_group()
{
  if (ui_->radiobutton_opmodes_group_->isChecked()) {
    opmodes_req->cmd_type = CMD_TYPE_GROUP;
    ui_->combobox_opmodes_name_->clear();
    ui_->combobox_opmodes_name_->addItems(qrobot_groups);
  }
}

void InterbotixControlPanel::opmodes_change_cmd_type_single()
{
  if (ui_->radiobutton_opmodes_single_->isChecked()) {
    opmodes_req->cmd_type = CMD_TYPE_SINGLE;
    ui_->combobox_opmodes_name_->clear();
    ui_->combobox_opmodes_name_->addItems(qrobot_arm_joints);
  }
}

void InterbotixControlPanel::opmodes_change_name()
{
  opmodes_req->name = ui_->combobox_opmodes_name_->currentText().toStdString();
}

void InterbotixControlPanel::opmodes_change_mode(int index)
{
  opmodes_req->mode = ui_->combobox_opmodes_mode_->currentText().toStdString();
}

void InterbotixControlPanel::opmodes_change_profile_type(int index)
{
  opmodes_req->profile_type = ui_->combobox_opmodes_profile_type_->currentText().toStdString();
}

void InterbotixControlPanel::opmodes_change_profile_vel()
{
  opmodes_req->profile_velocity = ui_->lineedit_opmodes_profile_vel_->text().toInt();
}

void InterbotixControlPanel::opmodes_change_profile_acc()
{
  opmodes_req->profile_acceleration = ui_->lineedit_opmodes_profile_acc_->text().toInt();
}

void InterbotixControlPanel::send_opmodes_call()
{
  auto result = srv_operating_modes->async_send_request(opmodes_req);
  if (rclcpp::spin_until_future_complete(
      client_node_, result,
      100ms) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(LOGGER, "Failed to call set_operating_modes.");
  }
}

// --------------------------- GetReg -----------------------------------------


void InterbotixControlPanel::getregval_change_cmd_type_group()
{
  if (ui_->radiobutton_getregval_group_->isChecked()) {
    getreg_req->cmd_type = CMD_TYPE_GROUP;
    ui_->combobox_getregval_name_->clear();
    ui_->combobox_getregval_name_->addItems(qrobot_groups);
  }
}

void InterbotixControlPanel::getregval_change_cmd_type_single()
{
  if (ui_->radiobutton_getregval_single_->isChecked()) {
    getreg_req->cmd_type = CMD_TYPE_SINGLE;
    ui_->combobox_getregval_name_->clear();
    ui_->combobox_getregval_name_->addItems(qrobot_arm_joints);
  }
}

void InterbotixControlPanel::getregval_change_name()
{
  getreg_req->name = ui_->combobox_getregval_name_->currentText().toStdString();
}

void InterbotixControlPanel::getregval_change_reg_name(int index)
{
  std::string str_current_regval = ui_->combobox_getregval_reg_->currentText().toStdString();
  std::string str_current_regval_desc =
    xs_register_descriptions::descriptions[str_current_regval].description;
  getreg_req->reg = str_current_regval;
  ui_->label_getregval_desc_->setText(QString(str_current_regval_desc.c_str()));
}

void InterbotixControlPanel::send_getregval_call()
{
  auto result = srv_get_motor_registers->async_send_request(getreg_req);
  if (rclcpp::spin_until_future_complete(
      client_node_, result,
      100ms) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(LOGGER, "Failed to call get_register_values.");
    return;
  }
  getregval_display(result.get());
}

void InterbotixControlPanel::getregval_display(
  const RegisterValues::Response::SharedPtr & response)
{
  std::vector<int32_t>::const_iterator it;
  std::stringstream s;
  for (it = response->values.begin(); it != response->values.end(); it++) {
    if (it != response->values.begin()) {
      s << ",";
    }
    s << *it;
  }
  ui_->lineedit_getregval_val_->setText(s.str().c_str());
}


// --------------------------- ESTOP ------------------------------------------

void InterbotixControlPanel::estop_button_pressed()
{
  send_system_call();
}

void InterbotixControlPanel::send_system_call()
{
  // TOOD(LSinterbotix): can't easily kill ros2 nodes using system calls
  RCLCPP_WARN(LOGGER, "The e-stop is not yet implemented due to ROS2 limitations.");
  // system(("rosnode kill /" + robot_namespace_ + "/xs_sdk").c_str());
}

// --------------------------- RVIZ -------------------------------------------

void InterbotixControlPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("RobotNameSpace", QString(robot_namespace_.c_str()));
}

void InterbotixControlPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString ns;
  if (config.mapGetString("RobotNameSpace", &ns)) {
    ui_->lineedit_robot_namespace_->setText(ns);
    update_robot_namespace();
  }
  loaded = true;
}

}  // namespace interbotix_xs_rviz

#include "pluginlib/class_list_macros.hpp" // NOLINT
PLUGINLIB_EXPORT_CLASS(interbotix_xs_rviz::InterbotixControlPanel, rviz_common::Panel)
