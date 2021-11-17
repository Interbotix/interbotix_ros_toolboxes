cmake_minimum_required(VERSION 3.5)
project(interbotix_xs_ros_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(xseries_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(joint_trajectory_controller REQUIRED)

## COMPILE
set(LIBRARY_NAME "interbotix_xs_ros_control")
add_library(
  ${LIBRARY_NAME}
  SHARED
  src/xs_hardware_interface_obj.cpp
)
target_include_directories(
  ${LIBRARY_NAME}
  PRIVATE
  include
)
ament_target_dependencies(
  ${LIBRARY_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  urdf
  xseries_msgs
  sensor_msgs
  joint_trajectory_controller
)

pluginlib_export_plugin_description_file(hardware_interface interbotix_xs_ros_control.xml)

# INSTALL
install(
  TARGETS ${LIBRARY_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

# install(DIRECTORY
#   launch
#   config
#   DESTINATION share/${PROJECT_NAME}/
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${LIBRARY_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)

ament_package()