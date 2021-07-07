cmake_minimum_required(VERSION 3.0.2)
project(interbotix_perception_modules)

## Compile as C++11, supported in ROS Kinetic and newer
if("$ENV{ROS_DISTRO}" STREQUAL "noetic")
  add_compile_options(-std=c++14)
else()
  add_compile_options(-std=c++11)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  apriltag_ros
  cv_bridge
  message_generation
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  tf2_ros
  tf2_geometry_msgs
  std_srvs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ClusterInfo.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ClusterInfoArray.srv
  FilterParams.srv
  SnapPicture.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
 CATKIN_DEPENDS
  apriltag_ros
  cv_bridge
  message_runtime
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  tf2_ros
  tf2_geometry_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## Specify libraries to link a library or executable target against
## Add cmake target dependencies of the executable
add_executable(perception_pipeline src/perception_pipeline.cpp)
target_link_libraries(perception_pipeline ${catkin_LIBRARIES})
add_dependencies(perception_pipeline ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/armtag_tuner_gui
  scripts/pointcloud_tuner_gui
  scripts/static_trans_pub
  scripts/picture_snapper
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
