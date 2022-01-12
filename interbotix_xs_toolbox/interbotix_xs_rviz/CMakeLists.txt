cmake_minimum_required(VERSION 3.0.2)
project(interbotix_xs_rviz)
catkin_package()
find_package(catkin REQUIRED COMPONENTS
    rviz
    interbotix_xs_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
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

## Here we specify the list of source files.
## The generated MOC files are included automatically as headers.
set(SRC_FILES
  src/torque_enable_panel.cpp include/interbotix_xs_rviz/torque_enable_panel.hpp
)

add_library(
  ${PROJECT_NAME} ${SRC_FILES} ${HEADER_FILES}
)

target_link_libraries(
  ${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES}
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
