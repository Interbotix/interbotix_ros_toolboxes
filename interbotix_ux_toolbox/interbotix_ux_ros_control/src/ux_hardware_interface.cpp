#include "interbotix_ux_ros_control/ux_hardware_interface_obj.h"

int main(int argc, char**argv)
{
	ros::init(argc, argv, "ux_hardware_interface");
	ros::NodeHandle nh;
	ros::MultiThreadedSpinner spinner(0);
	UXHardwareInterface robot(nh);
	spinner.spin();
	return 0;
}
