#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include "interbotix_rviz_interface/torque_enable_panel.hpp"

namespace interbotix_rviz_interface
{

TorqueEnablePanel::TorqueEnablePanel (QWidget* parent)
    : rviz::Panel(parent)
{
    // Robot namespace layout
    QHBoxLayout* robot_namespace_layout = new QHBoxLayout;
    robot_namespace_layout->addWidget(new QLabel("Robot Namespace:"));
    robot_namespace_editor_ = new QLineEdit;
    robot_namespace_layout->addWidget(robot_namespace_editor_);

    // Warning layout
    QHBoxLayout* warning_layout = new QHBoxLayout;
    QLabel* warning_label = new QLabel("WARNING: Disabling torque will cause the robot to collapse! Hold on to it or return it to its sleep pose before disabling!");
    warning_label->setWordWrap(true);
    warning_layout->addWidget(warning_label);

    // Buttons layout
    QHBoxLayout* torque_enable_layout = new QHBoxLayout;
    torque_enable_layout->addWidget(new QLabel("Torque State:"));
    torque_enable_button_ = new QPushButton("Enable");
    torque_disable_button_ = new QPushButton("Disable");
    torque_enable_layout->addWidget(torque_enable_button_);
    torque_enable_layout->addWidget(torque_disable_button_);

    // Connect qt signals to slots
    connect(torque_enable_button_, SIGNAL(clicked()), this, SLOT(enable_torque()));
    connect(torque_disable_button_, SIGNAL(clicked()), this, SLOT(disable_torque()));
    connect(robot_namespace_editor_, SIGNAL(editingFinished()), this, SLOT(update_robot_namespace()));
 
    // Instantiate basic torque_enable call
    torque_enable_call.request.cmd_type = "group";
    torque_enable_call.request.name = "arm";

    // Build and set final layout
    QVBoxLayout* layout = new QVBoxLayout;
    layout->addLayout(torque_enable_layout);
    layout->addLayout(robot_namespace_layout);
    layout->addLayout(warning_layout);
    setLayout(layout);

    // Start disabled - don't have robot namespace yet
    torque_enable_button_->setEnabled(false);
    torque_disable_button_->setEnabled(false);
}

void TorqueEnablePanel::update_robot_namespace()
{
    set_robot_namespace(robot_namespace_editor_->text());
}

void TorqueEnablePanel::set_robot_namespace(const QString& robot_namespace)
{
    srv_torque_enable = nh_.serviceClient<interbotix_xs_msgs::TorqueEnable>("/"+robot_namespace.toStdString()+"/torque_enable");
    if (!srv_torque_enable.waitForExistence(ros::Duration(0.01)))
    {
        torque_enable_button_->setEnabled(false);
        torque_disable_button_->setEnabled(false);
    }
    else
    {
        torque_enable_button_->setEnabled(true);
        torque_disable_button_->setEnabled(true);
    }
    Q_EMIT configChanged();
}

void TorqueEnablePanel::enable_torque()
{
    send_call(true);
}

void TorqueEnablePanel::disable_torque()
{
    send_call(false);
}

void TorqueEnablePanel::send_call(const bool enable)
{
    if(ros::ok())
    {
        torque_enable_call.request.enable = enable;
        srv_torque_enable.call(torque_enable_call);
    }
}

void TorqueEnablePanel::save(rviz::Config config) const
{
    rviz::Panel::save(config);
    config.mapSetValue("RobotNameSpace", robot_namespace_);
}

void TorqueEnablePanel::load(const rviz::Config& config)
{
    rviz::Panel::load(config);
    QString ns;
    if (config.mapGetString("RobotNameSpace", &ns));
    {
        robot_namespace_editor_->setText(ns);
        update_robot_namespace();
    }
}

} // interbotix_rviz_interface

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(interbotix_rviz_interface::TorqueEnablePanel, rviz::Panel)
