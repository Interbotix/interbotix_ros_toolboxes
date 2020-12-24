#include "interbotix_ux_ros_control/ux_hardware_interface_obj.h"

UXHardwareInterface::UXHardwareInterface(ros::NodeHandle& nh) : nh(nh)
{
	init();
	controller_manager.reset(new controller_manager::ControllerManager(this, nh));
	ros::Duration update_freq = ros::Duration(1.0/control_rate);
	tmr_control_loop = nh.createTimer(update_freq, &UXHardwareInterface::update, this);
}

UXHardwareInterface::~UXHardwareInterface()
{
	uxarm.setMode(XARM_MODE::POSE);
}

void UXHardwareInterface::init()
{
	uxarm.init(nh);
	std::string js_topic, robot_state_topic, robot_ip;
	nh.getParam("DOF", dof);
	nh.getParam("xarm_robot_ip", robot_ip);
	nh.getParam("joint_names", joint_names);
	nh.getParam("hardware_interface/use_gripper", use_gripper);
	nh.getParam("hardware_interface/control_rate", control_rate);
	nh.getParam("hardware_interface/joint_states_topic", js_topic);
	nh.getParam("hardware_interface/robot_state_topic", robot_state_topic);

	if (use_gripper)
	{
		ros::service::waitForService("gripper_move");
		ros::service::waitForService("gripper_config");
		float pulse_vel = 0;
		nh.getParam("hardware_interface/gripper_name", gripper_name);
		nh.getParam("hardware_interface/gripper_pulse_vel", pulse_vel);
		uxarm.gripperConfig(pulse_vel);
		joint_names.push_back(gripper_name);
	}

	curr_err = 0;
	curr_state = 0;
	joint_states = NULL;

	sub_joint_states = nh.subscribe(js_topic, 1, &UXHardwareInterface::joint_state_cb, this);
	sub_robot_states = nh.subscribe(robot_state_topic, 1, &UXHardwareInterface::robot_state_cb, this);

	ros::Rate loop_rate(control_rate);
	while (joint_states == NULL && ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	clientInit(robot_ip);
}

void UXHardwareInterface::clientInit(const std::string& robot_ip)
{
	joint_position_commands_float.resize(dof);
	joint_position_commands_float_prev.resize(dof);
	joint_position_commands.resize(joint_names.size());
	joint_positions.resize(joint_names.size());
	joint_velocities.resize(joint_names.size());
	joint_efforts.resize(joint_names.size());

	for (size_t i{0}; i < dof; i++)
	{
		joint_position_commands.at(i) = joint_states->position.at(i);
		joint_position_commands_float.at(i) = (float)joint_states->position.at(i);
		joint_position_commands_float_prev.at(i) = (float)joint_states->position.at(i);
	}

	if (use_gripper)
	{
		gripper_cmd_prev = joint_states->position.at(dof);
		joint_position_commands.at(dof) = gripper_cmd_prev;
	}

	for(size_t j=0; j < joint_names.size(); j++)
  {
		// Create joint state interface for all joints
		joint_state_interface.registerHandle(hardware_interface::JointStateHandle(joint_names.at(j), &joint_positions.at(j), &joint_velocities.at(j), &joint_efforts.at(j)));

		hardware_interface::JointHandle joint_handle;
		joint_handle = hardware_interface::JointHandle(joint_state_interface.getHandle(joint_names.at(j)), &joint_position_commands.at(j));
	  position_joint_interface.registerHandle(joint_handle);
  }

  	registerInterface(&joint_state_interface);
  	registerInterface(&position_joint_interface);

  	int ret1 = uxarm.motionEnable(1);
  	int ret2 = uxarm.setMode(XARM_MODE::SERVO);
  	int ret3 = uxarm.setState(XARM_STATE::START);

  	if(ret3)
  	{
  		ROS_ERROR("The Xarm may not be properly connected (ret = 3) or hardware Error/Warning (ret = 1 or 2) exists, PLEASE CHECK or RESTART HARDWARE!!!");
  		ROS_ERROR(" ");
  		ROS_ERROR("Did you specify the correct ros param xarm_robot_ip ? Exitting...");
  		ros::shutdown();
  		exit(1);
  	}
}

void UXHardwareInterface::joint_state_cb(const sensor_msgs::JointState::ConstPtr& msg)
{
	std::lock_guard<std::mutex> locker(mutex);
	joint_states = msg;
}

void UXHardwareInterface::robot_state_cb(const xarm_msgs::RobotMsg::ConstPtr& msg)
{
	curr_mode = msg->mode;
	curr_state = msg->state;
	curr_err = msg->err;
}

void UXHardwareInterface::update(const ros::TimerEvent& e)
{
    ros::Duration elapsed_time = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager->update(ros::Time::now(), elapsed_time, need_reset());
    write();
}

void UXHardwareInterface::read()
{
	std::lock_guard<std::mutex> locker(mutex);
	for(int j=0; j<joint_names.size(); j++)
	{
	 	joint_positions.at(j) = joint_states->position.at(j);
		joint_velocities.at(j) = joint_states->velocity.at(j);
		joint_efforts.at(j) = joint_states->effort.at(j);
	}
}

void UXHardwareInterface::write()
{
	if(need_reset())
	{
		std::lock_guard<std::mutex> locker(mutex);
		for(int k=0; k<dof; k++)
			joint_position_commands_float.at(k) = (float)joint_positions.at(k);
		return;
	}

	for(int k=0; k<dof; k++)
	{
		// make sure no abnormal command will be written into joints, check if cmd velocity > [180 deg/sec * (1+10%)]
		if(fabs(joint_position_commands_float.at(k) - (float)joint_position_commands.at(k)) * control_rate > 3.14*1.25)
			ROS_WARN("joint %d abnormal command! previous: %f, this: %f\n", k+1, joint_position_commands_float.at(k), (float)joint_position_commands.at(k));

		joint_position_commands_float.at(k) = (float)joint_position_commands.at(k);
	}

	if (joint_position_commands_float != joint_position_commands_float_prev)
	{
		uxarm.setServoJ(joint_position_commands_float);
		joint_position_commands_float_prev = joint_position_commands_float;
	}

	if (use_gripper && (gripper_cmd_prev != joint_position_commands.back()))
	{
		float pulse = 850 - (joint_position_commands.back() * 1000.0);
		uxarm.gripperMove(pulse);
		gripper_cmd_prev = joint_position_commands.back();
	}
}

void UXHardwareInterface::get_status(int state_mode_err[3])
{
	state_mode_err[0] = curr_state;
	state_mode_err[1] = curr_mode;
	state_mode_err[2] = curr_err;
}

bool UXHardwareInterface::need_reset()
{
	if(curr_state==4 || curr_state==5 || curr_err)
		return true;
	else
		return false;
}
