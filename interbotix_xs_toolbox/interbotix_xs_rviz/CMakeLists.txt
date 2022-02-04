cmake_minimum_required(VERSION 3.0.2)
project(interbotix_xs_rviz)
find_package(catkin REQUIRED COMPONENTS
  rviz
  interbotix_xs_msgs
)
catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
endif()

add_definitions(-DQT_NO_KEYWORDS)

qt5_wrap_ui(UIC_FILES
  src/ui/interbotix_control_panel.ui
)

## Here we specify the list of source files.
## The generated MOC files are included automatically as headers.
set(SRC_FILES
  src/interbotix_control_panel.cpp
  include/interbotix_xs_rviz/interbotix_control_panel.hpp
)

add_library(
  ${PROJECT_NAME}
  ${SRC_FILES}
  ${HEADER_FILES}
  ${UIC_FILES}
)

target_link_libraries(
  ${PROJECT_NAME}
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(
  ${PROJECT_NAME}
  interbotix_xs_msgs_generate_messages_cpp # this fixes parallel build issue https://answers.ros.org/question/52744/how-to-specify-dependencies-with-foo_msgs-catkin-packages/
)
  
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
