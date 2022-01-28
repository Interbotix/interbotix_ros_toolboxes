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
  connect(ui_->button_torque_enable_,          SIGNAL(clicked()),                this, SLOT(torque_enable_torque()));
  connect(ui_->button_torque_disable_,         SIGNAL(clicked()),                this, SLOT(torque_disable_torque()));
  connect(ui_->lineedit_torque_name_,          SIGNAL(editingFinished()),        this, SLOT(torque_change_name()));
  connect(ui_->radiobutton_torque_group_,      SIGNAL(toggled(bool)),            this, SLOT(torque_change_cmd_type_group()));
  connect(ui_->radiobutton_torque_single_,     SIGNAL(toggled(bool)),            this, SLOT(torque_change_cmd_type_single()));
  
  // reboot
  connect(ui_->lineedit_reboot_name_,          SIGNAL(editingFinished()),        this, SLOT(reboot_change_name()));
  connect(ui_->radiobutton_reboot_group_,      SIGNAL(toggled(bool)),            this, SLOT(reboot_change_cmd_type_group()));
  connect(ui_->radiobutton_reboot_single_,     SIGNAL(toggled(bool)),            this, SLOT(reboot_change_cmd_type_single()));
  connect(ui_->checkbox_smart_reboot_,         SIGNAL(toggled(bool)),            this, SLOT(reboot_change_smartreboot(bool)));
  connect(ui_->checkbox_reboot_enable_,        SIGNAL(toggled(bool)),            this, SLOT(reboot_change_enable(bool)));
  connect(ui_->button_reboot_reboot_,          SIGNAL(clicked()),                this, SLOT(send_reboot_call()));

  // operating modes
  connect(ui_->radiobutton_opmodes_group_,     SIGNAL(toggled(bool)),            this, SLOT(opmodes_change_cmd_type_group()));
  connect(ui_->radiobutton_opmodes_single_,    SIGNAL(toggled(bool)),            this, SLOT(opmodes_change_cmd_type_single()));
  connect(ui_->lineedit_opmodes_name_,         SIGNAL(editingFinished()),        this, SLOT(opmodes_change_name()));
  connect(ui_->combobox_opmodes_mode_,         SIGNAL(currentIndexChanged(int)), this, SLOT(opmodes_change_mode(int)));
  connect(ui_->combobox_opmodes_profile_type_, SIGNAL(currentIndexChanged(int)), this, SLOT(opmodes_change_profile_type(int)));
  connect(ui_->lineedit_opmodes_profile_vel_,  SIGNAL(editingFinished()),        this, SLOT(opmodes_change_profile_vel()));
  connect(ui_->lineedit_opmodes_profile_acc_,  SIGNAL(editingFinished()),        this, SLOT(opmodes_change_profile_acc()));
  connect(ui_->button_opmodes_set_,            SIGNAL(clicked()),                this, SLOT(send_opmodes_call()));

  // get registers
  connect(ui_->radiobutton_getregval_group_,  SIGNAL(toggled(bool)),             this, SLOT(getregval_change_cmd_type_group()));
  connect(ui_->radiobutton_getregval_single_, SIGNAL(toggled(bool)),             this, SLOT(getregval_change_cmd_type_single()));
  connect(ui_->lineedit_getregval_name_,      SIGNAL(editingFinished()),         this, SLOT(getregval_change_name()));
  connect(ui_->combobox_getregval_reg_,       SIGNAL(currentIndexChanged(int)),  this, SLOT(getregval_change_reg_name(int)));
  connect(ui_->button_getregval_val_,         SIGNAL(clicked()),                 this, SLOT(send_getregval_call()));

  // e-stop
  connect(ui_->button_estop_,                 SIGNAL(clicked()),                 this, SLOT(estop_button_pressed()));

  // initialize the node
  nh_ = ros::NodeHandle("interbotix_control_panel");

  // initialize all tabs
  torque_init();
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

  // start all service clients
  srv_torque_enable = nh_.serviceClient<interbotix_xs_msgs::TorqueEnable>(
    "/" + robot_namespace_ + "/torque_enable");
  srv_operating_modes = nh_.serviceClient<interbotix_xs_msgs::OperatingModes>(
    "/" + robot_namespace_ + "/set_operating_modes");
  srv_reboot_motors = nh_.serviceClient<interbotix_xs_msgs::Reboot>(
    "/" + robot_namespace_ + "/reboot_motors");
  srv_get_motor_registers = nh_.serviceClient<interbotix_xs_msgs::RegisterValues>(
    "/" + robot_namespace_ + "/get_motor_registers");

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

  Q_EMIT configChanged();
}

void InterbotixControlPanel::enable_elements(const bool enable)
{
  // torque
  ui_->button_torque_enable_->setEnabled(enable);
  ui_->button_torque_disable_->setEnabled(enable);
  ui_->lineedit_torque_name_->setEnabled(enable);
  ui_->lineedit_torque_name_->setEnabled(enable);
  ui_->radiobutton_torque_single_->setEnabled(enable);
  ui_->radiobutton_torque_group_->setEnabled(enable);
  
  // reboot
  ui_->lineedit_reboot_name_->setEnabled(enable);
  ui_->radiobutton_reboot_single_->setEnabled(enable);
  ui_->radiobutton_reboot_group_->setEnabled(enable);
  ui_->button_reboot_reboot_->setEnabled(enable);
  ui_->checkbox_smart_reboot_->setEnabled(enable);
  ui_->checkbox_reboot_enable_->setEnabled(enable);

  // operating modes
  ui_->lineedit_opmodes_name_->setEnabled(enable);
  ui_->radiobutton_opmodes_single_->setEnabled(enable);
  ui_->radiobutton_opmodes_group_->setEnabled(enable);
  ui_->combobox_opmodes_mode_->setEnabled(enable);
  ui_->combobox_opmodes_profile_type_->setEnabled(enable);
  ui_->lineedit_opmodes_profile_vel_->setEnabled(enable);
  ui_->lineedit_opmodes_profile_acc_->setEnabled(enable);
  ui_->button_opmodes_set_->setEnabled(enable);

  // getregval
  ui_->lineedit_getregval_name_->setEnabled(enable);
  ui_->radiobutton_getregval_single_->setEnabled(enable);
  ui_->radiobutton_getregval_group_->setEnabled(enable);
  ui_->combobox_getregval_reg_->setEnabled(enable);
  ui_->button_getregval_val_->setEnabled(enable);
  ui_->lineedit_getregval_val_->setEnabled(enable);

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
    torque_enable_call.request.cmd_type = "group";
}

void InterbotixControlPanel::torque_change_cmd_type_single()
{
  if (ui_->radiobutton_torque_single_->isChecked())
    torque_enable_call.request.cmd_type = "single";
}

void InterbotixControlPanel::torque_change_name()
{
 torque_enable_call.request.name = ui_->lineedit_torque_name_->text().toStdString();
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
    reboot_call.request.cmd_type = "group";
}

void InterbotixControlPanel::reboot_change_cmd_type_single()
{
  if (ui_->radiobutton_reboot_single_->isChecked())
    reboot_call.request.cmd_type = "single";
}

void InterbotixControlPanel::reboot_change_name()
{
  reboot_call.request.name = ui_->lineedit_reboot_name_->text().toStdString();
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
    opmodes_call.request.cmd_type = "group";
}

void InterbotixControlPanel::opmodes_change_cmd_type_single()
{
  if (ui_->radiobutton_opmodes_single_->isChecked())
    opmodes_call.request.cmd_type = "single";
}

void InterbotixControlPanel::opmodes_change_name()
{
  opmodes_call.request.name = ui_->lineedit_opmodes_name_->text().toStdString();
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
}

void InterbotixControlPanel::getregval_change_cmd_type_group()
{
  if (ui_->radiobutton_getregval_group_->isChecked())
    getreg_call.request.cmd_type = "group";
}

void InterbotixControlPanel::getregval_change_cmd_type_single()
{
  if (ui_->radiobutton_getregval_single_->isChecked())
  getreg_call.request.cmd_type = "single";
}

void InterbotixControlPanel::getregval_change_name()
{
  getreg_call.request.name = ui_->lineedit_getregval_name_->text().toStdString();
}

void InterbotixControlPanel::getregval_change_reg_name(int index)
{
  getreg_call.request.reg = ui_->combobox_getregval_reg_->currentText().toStdString();
}

void InterbotixControlPanel::send_getregval_call()
{
  if (ros::ok())
    if(srv_get_motor_registers.call(getreg_call))
      getregval_display(getreg_call.response);
}

void InterbotixControlPanel::getregval_display(interbotix_xs_msgs::RegisterValues::Response &res)
{
  std::vector<int32_t>::const_iterator it;
  std::stringstream s;
  for (it = res.values.begin(); it != res.values.end(); it++)
  {
    if (it != res.values.begin())
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
