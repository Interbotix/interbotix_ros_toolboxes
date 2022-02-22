#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>

#include "interbotix_xs_rviz/interbotix_control_panel.hpp"

#include "ui_interbotix_control_panel.h"

namespace interbotix_xs_rviz
{

InterbotixControlPanel::InterbotixControlPanel(QWidget* parent)
  : rviz::Panel(parent), 
    ui_(new Ui::InterbotixControlPanelUI())
{
  // Set up the RViz panel
  ui_->setupUi(this);

  // Connect Qt signals to slots  
  
  // robot namespace
  connect(ui_->lineedit_robot_namespace_,      SIGNAL(editingFinished()),        this, SLOT(update_robot_namespace()));
  
  // torque
  connect(ui_->combobox_torque_name_,          SIGNAL(currentTextChanged(const QString)), this, SLOT(torque_change_name()));
  connect(ui_->button_torque_enable_,          SIGNAL(clicked()),                this, SLOT(torque_enable_torque()));
  connect(ui_->button_torque_disable_,         SIGNAL(clicked()),                this, SLOT(torque_disable_torque()));
  connect(ui_->radiobutton_torque_group_,      SIGNAL(toggled(bool)),            this, SLOT(torque_change_cmd_type_group()));
  connect(ui_->radiobutton_torque_single_,     SIGNAL(toggled(bool)),            this, SLOT(torque_change_cmd_type_single()));
  
  // home/sleep
  connect(ui_->button_gotohome_,               SIGNAL(clicked()),                this, SLOT(homesleep_go_to_home()));
  connect(ui_->button_gotosleep_,              SIGNAL(clicked()),                this, SLOT(homesleep_go_to_sleep()));

  // reboot
  connect(ui_->combobox_reboot_name_,          SIGNAL(currentTextChanged(const QString)), this, SLOT(reboot_change_name()));
  connect(ui_->radiobutton_reboot_group_,      SIGNAL(toggled(bool)),            this, SLOT(reboot_change_cmd_type_group()));
  connect(ui_->radiobutton_reboot_single_,     SIGNAL(toggled(bool)),            this, SLOT(reboot_change_cmd_type_single()));
  connect(ui_->checkbox_smart_reboot_,         SIGNAL(toggled(bool)),            this, SLOT(reboot_change_smartreboot(bool)));
  connect(ui_->checkbox_reboot_enable_,        SIGNAL(toggled(bool)),            this, SLOT(reboot_change_enable(bool)));
  connect(ui_->button_reboot_reboot_,          SIGNAL(clicked()),                this, SLOT(send_reboot_call()));

  // operating modes
  connect(ui_->combobox_opmodes_name_,         SIGNAL(currentTextChanged(const QString)), this, SLOT(opmodes_change_name()));
  connect(ui_->radiobutton_opmodes_group_,     SIGNAL(toggled(bool)),            this, SLOT(opmodes_change_cmd_type_group()));
  connect(ui_->radiobutton_opmodes_single_,    SIGNAL(toggled(bool)),            this, SLOT(opmodes_change_cmd_type_single()));
  connect(ui_->combobox_opmodes_mode_,         SIGNAL(currentIndexChanged(int)), this, SLOT(opmodes_change_mode(int)));
  connect(ui_->combobox_opmodes_profile_type_, SIGNAL(currentIndexChanged(int)), this, SLOT(opmodes_change_profile_type(int)));
  connect(ui_->lineedit_opmodes_profile_vel_,  SIGNAL(editingFinished()),        this, SLOT(opmodes_change_profile_vel()));
  connect(ui_->lineedit_opmodes_profile_acc_,  SIGNAL(editingFinished()),        this, SLOT(opmodes_change_profile_acc()));
  connect(ui_->button_opmodes_set_,            SIGNAL(clicked()),                this, SLOT(send_opmodes_call()));

  // get registers
  connect(ui_->combobox_getregval_name_,      SIGNAL(currentTextChanged(const QString)), this, SLOT(getregval_change_name()));
  connect(ui_->radiobutton_getregval_group_,  SIGNAL(toggled(bool)),             this, SLOT(getregval_change_cmd_type_group()));
  connect(ui_->radiobutton_getregval_single_, SIGNAL(toggled(bool)),             this, SLOT(getregval_change_cmd_type_single()));
  connect(ui_->combobox_getregval_reg_,       SIGNAL(currentIndexChanged(int)),  this, SLOT(getregval_change_reg_name(int)));
  connect(ui_->button_getregval_val_,         SIGNAL(clicked()),                 this, SLOT(send_getregval_call()));

  // e-stop
  connect(ui_->button_estop_,                 SIGNAL(clicked()),                 this, SLOT(estop_button_pressed()));

  // initialize the node and some service calls
  nh_ = ros::NodeHandle("interbotix_control_panel");
  robot_info_call.request.cmd_type = "group";
  robot_info_call.request.name = "arm";

  // initialize all tabs
  torque_init();
  homesleep_init();
  reboot_init();
  opmodes_init();
  getregval_init();
}

InterbotixControlPanel::~InterbotixControlPanel()
{
  delete ui_;
}

void InterbotixControlPanel::update_robot_namespace()
{
  set_robot_namespace(ui_->lineedit_robot_namespace_->text());
}

void InterbotixControlPanel::set_robot_namespace(const QString& robot_namespace)
{
  robot_namespace_ = robot_namespace.toStdString();

  // start all service clients, publishers, and subscribers
  srv_torque_enable = nh_.serviceClient<interbotix_xs_msgs::TorqueEnable>(
    "/" + robot_namespace_ + "/torque_enable");
  srv_operating_modes = nh_.serviceClient<interbotix_xs_msgs::OperatingModes>(
    "/" + robot_namespace_ + "/set_operating_modes");
  srv_reboot_motors = nh_.serviceClient<interbotix_xs_msgs::Reboot>(
    "/" + robot_namespace_ + "/reboot_motors");
  srv_get_motor_registers = nh_.serviceClient<interbotix_xs_msgs::RegisterValues>(
    "/" + robot_namespace_ + "/get_motor_registers");
  srv_robot_info = nh_.serviceClient<interbotix_xs_msgs::RobotInfo>(
    "/" + robot_namespace_ + "/get_robot_info");
  pub_joint_group_cmd = nh_.advertise<interbotix_xs_msgs::JointGroupCommand>(
    "/" + robot_namespace_ + "/commands/joint_group", 1);

  if (!loaded) // wait longer if RViz is loading
  {
    if (!srv_torque_enable.waitForExistence(ros::Duration(2.0)))
      enable_elements(false);
    else
      enable_elements(true);
  }
  else
  {
    if (!srv_torque_enable.waitForExistence(ros::Duration(0.1)))
      enable_elements(false);
    else
      enable_elements(true);
  }

  update_robot_info();
  Q_EMIT configChanged();
}

void InterbotixControlPanel::update_robot_info()
{
  ROS_INFO("Updating robot_info");
  srv_robot_info.call(robot_info_call);
  homesleep_homevec.resize(robot_info_call.response.num_joints);
  std::fill(homesleep_homevec.begin(), homesleep_homevec.end(), 0.0f);
  homesleep_sleepvec.resize(robot_info_call.response.num_joints);
  homesleep_sleepvec = robot_info_call.response.joint_sleep_positions;

  robot_arm_joints.clear();
  qrobot_arm_joints.clear();
  for (const auto joint_name : robot_info_call.response.joint_names)
  {
    ROS_INFO("\n%s\n", joint_name.c_str());
    robot_arm_joints.push_back(joint_name);
    qrobot_arm_joints << QString(joint_name.c_str());
  }
}

void InterbotixControlPanel::enable_elements(const bool enable)
{
  // torque
  ui_->button_torque_enable_->setEnabled(enable);
  ui_->button_torque_disable_->setEnabled(enable);
  // ui_->lineedit_torque_name_->setEnabled(enable);
  ui_->radiobutton_torque_single_->setEnabled(enable);
  ui_->radiobutton_torque_group_->setEnabled(enable);
  ui_->combobox_torque_name_->setEnabled(enable);

  // home/sleep
  ui_->button_gotohome_->setEnabled(enable);
  ui_->button_gotosleep_->setEnabled(enable);
  
  // reboot
  // ui_->lineedit_reboot_name_->setEnabled(enable);
  ui_->radiobutton_reboot_single_->setEnabled(enable);
  ui_->radiobutton_reboot_group_->setEnabled(enable);
  ui_->button_reboot_reboot_->setEnabled(enable);
  ui_->checkbox_smart_reboot_->setEnabled(enable);
  ui_->checkbox_reboot_enable_->setEnabled(enable);
  ui_->combobox_reboot_name_->setEnabled(enable);

  // operating modes
  // ui_->lineedit_opmodes_name_->setEnabled(enable);
  ui_->radiobutton_opmodes_single_->setEnabled(enable);
  ui_->radiobutton_opmodes_group_->setEnabled(enable);
  ui_->combobox_opmodes_mode_->setEnabled(enable);
  ui_->combobox_opmodes_profile_type_->setEnabled(enable);
  ui_->lineedit_opmodes_profile_vel_->setEnabled(enable);
  ui_->lineedit_opmodes_profile_acc_->setEnabled(enable);
  ui_->button_opmodes_set_->setEnabled(enable);
  ui_->combobox_opmodes_name_->setEnabled(enable);

  // getregval
  // ui_->lineedit_getregval_name_->setEnabled(enable);
  ui_->radiobutton_getregval_single_->setEnabled(enable);
  ui_->radiobutton_getregval_group_->setEnabled(enable);
  ui_->combobox_getregval_reg_->setEnabled(enable);
  ui_->button_getregval_val_->setEnabled(enable);
  ui_->lineedit_getregval_val_->setEnabled(enable);
  ui_->combobox_getregval_name_->setEnabled(enable);

  // estop
  ui_->button_estop_->setEnabled(enable);
}


// -------------------------- TORQUE ------------------------------------------

void InterbotixControlPanel::torque_init()
{
  torque_enable_call.request.cmd_type = "group";
  torque_enable_call.request.name = "arm";
}

void InterbotixControlPanel::torque_change_cmd_type_group()
{
  if (ui_->radiobutton_torque_group_->isChecked())
  {
    torque_enable_call.request.cmd_type = "group";
    ui_->combobox_torque_name_->clear();
    ui_->combobox_torque_name_->addItems(qrobot_groups);
  }
}

void InterbotixControlPanel::torque_change_cmd_type_single()
{
  if (ui_->radiobutton_torque_single_->isChecked())
  {
    torque_enable_call.request.cmd_type = "single";
    ui_->combobox_torque_name_->clear();
    ui_->combobox_torque_name_->addItems(qrobot_arm_joints);
  }
}

void InterbotixControlPanel::torque_change_name()
{
 torque_enable_call.request.name = ui_->combobox_torque_name_->currentText().toStdString();
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
  torque_enable_call.request.enable = enable;
  if(ros::ok())
    srv_torque_enable.call(torque_enable_call);
}


// -------------------------- HOME/SLEEP --------------------------------------

void InterbotixControlPanel::homesleep_init()
{
  joint_group_cmd.name = "arm";
}

void InterbotixControlPanel::homesleep_go_to_home()
{
  joint_group_cmd.cmd = homesleep_homevec;
  pub_joint_group_cmd.publish(joint_group_cmd);
}

void InterbotixControlPanel::homesleep_go_to_sleep()
{
  joint_group_cmd.cmd = homesleep_sleepvec;
  pub_joint_group_cmd.publish(joint_group_cmd);
}


// --------------------------  REBOOT -----------------------------------------

void InterbotixControlPanel::reboot_init()
{
  reboot_call.request.cmd_type = "group";
  reboot_call.request.name = "arm";
  reboot_call.request.smart_reboot = false;
  reboot_call.request.enable = true;
}

void InterbotixControlPanel::reboot_change_cmd_type_group()
{
  if (ui_->radiobutton_reboot_group_->isChecked())
  {
    reboot_call.request.cmd_type = "group";
    ui_->combobox_reboot_name_->clear();
    ui_->combobox_reboot_name_->addItems(qrobot_groups);
  }
}

void InterbotixControlPanel::reboot_change_cmd_type_single()
{
  if (ui_->radiobutton_reboot_single_->isChecked())
  {
    reboot_call.request.cmd_type = "single";
    ui_->combobox_reboot_name_->clear();
    ui_->combobox_reboot_name_->addItems(qrobot_arm_joints);
  }
}

void InterbotixControlPanel::reboot_change_name()
{
  reboot_call.request.name = ui_->combobox_reboot_name_->currentText().toStdString();
}

void InterbotixControlPanel::reboot_change_smartreboot(bool checked)
{
  reboot_call.request.smart_reboot = checked;
}

void InterbotixControlPanel::reboot_change_enable(bool checked)
{
  reboot_call.request.enable = checked;
}

void InterbotixControlPanel::send_reboot_call()
{
  if(ros::ok())
    srv_reboot_motors.call(reboot_call);
}


// --------------------------  OpModes ----------------------------------------

void InterbotixControlPanel::opmodes_init()
{
  opmodes_call.request.cmd_type = "group";
  opmodes_call.request.name = "arm";
  opmodes_call.request.mode = "position";
  opmodes_call.request.profile_type = "time";
  opmodes_call.request.profile_velocity = 2000;
  opmodes_call.request.profile_acceleration = 300;
}

void InterbotixControlPanel::opmodes_change_cmd_type_group()
{
  if (ui_->radiobutton_opmodes_group_->isChecked())
  {
    opmodes_call.request.cmd_type = "group";
    ui_->combobox_opmodes_name_->clear();
    ui_->combobox_opmodes_name_->addItems(qrobot_groups);
  }
}

void InterbotixControlPanel::opmodes_change_cmd_type_single()
{
  if (ui_->radiobutton_opmodes_single_->isChecked())
  {
    opmodes_call.request.cmd_type = "single";
    ui_->combobox_opmodes_name_->clear();
    ui_->combobox_opmodes_name_->addItems(qrobot_arm_joints);
  }
}

void InterbotixControlPanel::opmodes_change_name()
{
  opmodes_call.request.name = ui_->combobox_opmodes_name_->currentText().toStdString();
}

void InterbotixControlPanel::opmodes_change_mode(int index)
{
  opmodes_call.request.mode = ui_->combobox_opmodes_mode_->currentText().toStdString();
}

void InterbotixControlPanel::opmodes_change_profile_type(int index)
{
  opmodes_call.request.profile_type = ui_->combobox_opmodes_profile_type_->currentText().toStdString();
}

void InterbotixControlPanel::opmodes_change_profile_vel()
{
  opmodes_call.request.profile_velocity = ui_->lineedit_opmodes_profile_vel_->text().toInt();
}

void InterbotixControlPanel::opmodes_change_profile_acc()
{
  opmodes_call.request.profile_acceleration = ui_->lineedit_opmodes_profile_acc_->text().toInt();
}

void InterbotixControlPanel::send_opmodes_call()
{
  if(ros::ok())
    srv_operating_modes.call(opmodes_call);
}

// --------------------------- GetReg -----------------------------------------

void InterbotixControlPanel::getregval_init()
{
  getreg_call.request.cmd_type = "group";
  getreg_call.request.name = "arm";
  getreg_call.request.reg = "Goal_Position";

  ui_->label_getregval_desc_->setText(QString(
    xs_register_descriptions::descriptions[
      ui_->combobox_getregval_reg_->currentText().toStdString()
    ].description.c_str()
  ));
}

void InterbotixControlPanel::getregval_change_cmd_type_group()
{
  if (ui_->radiobutton_getregval_group_->isChecked())
  {
    getreg_call.request.cmd_type = "group";
    ui_->combobox_getregval_name_->clear();
    ui_->combobox_getregval_name_->addItems(qrobot_groups);
  }
}

void InterbotixControlPanel::getregval_change_cmd_type_single()
{
  if (ui_->radiobutton_getregval_single_->isChecked())
  {
    getreg_call.request.cmd_type = "single";
    ui_->combobox_getregval_name_->clear();
    ui_->combobox_getregval_name_->addItems(qrobot_arm_joints);
  }
}

void InterbotixControlPanel::getregval_change_name()
{
  getreg_call.request.name = ui_->combobox_getregval_name_->currentText().toStdString();
}

void InterbotixControlPanel::getregval_change_reg_name(int index)
{
  std::string str_current_regval = ui_->combobox_getregval_reg_->currentText().toStdString();
  std::string str_current_regval_desc = xs_register_descriptions::descriptions[str_current_regval].description;
  getreg_call.request.reg = str_current_regval;
  ui_->label_getregval_desc_->setText(QString(str_current_regval_desc.c_str()));
}

void InterbotixControlPanel::send_getregval_call()
{
  if (ros::ok())
    if(srv_get_motor_registers.call(getreg_call))
      getregval_display(getreg_call);
}

void InterbotixControlPanel::getregval_display(interbotix_xs_msgs::RegisterValues &call)
{
  std::vector<int32_t>::const_iterator it;
  std::stringstream s;
  for (it = call.response.values.begin(); it != call.response.values.end(); it++)
  {
    if (it != call.response.values.begin())
      s << ",";
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
  if(ros::ok())
  {
    system(("rosnode kill /" + robot_namespace_ + "/xs_sdk").c_str());
  }
}

// --------------------------- RVIZ -------------------------------------------

void InterbotixControlPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
  config.mapSetValue("RobotNameSpace", QString(robot_namespace_.c_str()));
}

void InterbotixControlPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
  QString ns;
  if (config.mapGetString("RobotNameSpace", &ns));
  {
    ui_->lineedit_robot_namespace_->setText(ns);
    set_robot_namespace(ns);
  }
  loaded = true;
}

} // namespace interbotix_xs_rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(interbotix_xs_rviz::InterbotixControlPanel, rviz::Panel)
