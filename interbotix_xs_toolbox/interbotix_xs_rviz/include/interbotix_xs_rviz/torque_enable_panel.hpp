#ifndef TORQUE_ENABLE_PANEL_HPP
#define TORQUE_ENABLE_PANEL_HPP

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h>
#endif

#include <QLineEdit>
#include <QPushButton>

#include "interbotix_xs_msgs/TorqueEnable.h"

namespace interbotix_xs_rviz
{

class TorqueEnablePanel: public rviz::Panel
{
Q_OBJECT // this is a Qt object
public:
  TorqueEnablePanel (QWidget* parent = 0);

  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

public Q_SLOTS:

  /**
   * @brief Calls the torque_enable service with the given boolean
   * @param enable Enables torque if true, disables torque if false
   */
  void send_call(const bool enable);
  
  /** 
   * @brief Sets the namespace of the robot as the given QString
   * @param robot_namespace the namespace to set the torque_enable service
   * client under
   */
  void set_robot_namespace(const QString& robot_namespace);

protected Q_SLOTS:

  /**
   * @brief Calls the torque_enable service with true to enable torque
   */
  void enable_torque();
  
  /**
   * @brief Calls the torque_enable service with false to disable torque
   */
  void disable_torque();

  /** 
   * @brief Updates the robot_namespace_ variable with the text in the robot_namespace_editor_
   */
  void update_robot_namespace();

protected:

  // ROS Node
  ros::NodeHandle nh_;

  // The torque_enable service client
  ros::ServiceClient srv_torque_enable;

  // The torque enable service request
  interbotix_xs_msgs::TorqueEnable torque_enable_call;

  QPushButton* torque_enable_button_;
  QPushButton* torque_disable_button_;
  QLineEdit* robot_namespace_editor_;
  QString robot_namespace_;

}; // TorqueEnablePanel

} // interbotix_xs_rviz

#endif // TORQUE_ENABLE_PANEL_HPP
