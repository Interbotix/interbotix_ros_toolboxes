#include "interbotix_xs_ros_control/xs_hardware_interface_obj.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "xs_hardware_interface");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(2);
    XSHardwareInterface RobotInterface(nh);
    spinner.spin();
    return 0;
}
