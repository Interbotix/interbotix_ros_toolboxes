#ifndef UX_HARDWARE_INTERFACE_OBJ_H
#define UX_HARDWARE_INTERFACE_OBJ_H

#include <pthread.h>
#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>
#include "xarm_ros_client.h"
#include "xarm/instruction/uxbus_cmd_config.h"

class UXHardwareInterface: public hardware_interface::RobotHW
{
public:

	UXHardwareInterface(ros::NodeHandle& nh);
	~UXHardwareInterface();
	void init();
	void update(const ros::TimerEvent& e);
	void read();
	void write();
	void joint_state_cb(const sensor_msgs::JointState::ConstPtr &msg);
	void robot_state_cb(const xarm_msgs::RobotMsg::ConstPtr& msg);

	/* get current arm status: in the order of state, mode and error_code */
	void get_status(int state_mode_err[3]);
	/* check whether the controller needs to be reset due to error or mode change */
	bool need_reset();

protected:

	// Command Interfaces
	hardware_interface::JointStateInterface    joint_state_interface;
	hardware_interface::PositionJointInterface position_joint_interface;

	std::vector<double> joint_positions;
	std::vector<double> joint_velocities;
	std::vector<double> joint_efforts;

	std::vector<double> joint_position_commands;
	std::vector<float> joint_position_commands_float;
	std::vector<float> joint_position_commands_float_prev;

	ros::NodeHandle nh;
	ros::Subscriber sub_joint_states;
	ros::Subscriber sub_robot_states;
	ros::Timer tmr_control_loop;
	sensor_msgs::JointState::ConstPtr joint_states;
	xarm_api::XArmROSClient uxarm;

	int curr_state;
	int curr_mode;
	int curr_err;

	int dof;
	double control_rate;
	double gripper_cmd_prev;
	std::string gripper_name;
	std::vector<std::string> joint_names;
	std::mutex mutex;

	bool use_gripper;
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager;

	void clientInit(const std::string& robot_ip);
	void pos_fb_cb(const sensor_msgs::JointState::ConstPtr& msg);
	void state_fb_cb(const xarm_msgs::RobotMsg::ConstPtr& msg);

};

#endif
