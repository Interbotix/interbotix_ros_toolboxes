cmake_minimum_required(VERSION 2.8.3)
project(interbotix_ux_ros_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  joint_trajectory_controller
  roscpp
  xarm_api
  xarm_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS controller_manager hardware_interface joint_trajectory_controller roscpp xarm_api xarm_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## Specify libraries to link a library or executable target against
## Add cmake target dependencies of the executable
add_executable(ux_hardware_interface src/ux_hardware_interface.cpp src/ux_hardware_interface_obj.cpp)
target_link_libraries(ux_hardware_interface ${catkin_LIBRARIES})
add_dependencies(ux_hardware_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
