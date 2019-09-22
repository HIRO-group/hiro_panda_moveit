cmake_minimum_required(VERSION 2.8.3)
project(hiro_panda_moveit)

set(CMAKE_BUILD_TYPE Release)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  kdl_parser
  roscpp
  rospy
  sensor_msgs
  std_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  tf
  franka_control
  franka_gripper
  pluginlib
  realtime_tools
  controller_interface
  dynamic_reconfigure
  franka_hw
  geometry_msgs
  hardware_interface
  message_generation
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.5.0 REQUIRED)

# generate_messages()

catkin_package(
LIBRARIES hiro_panda_controllers
CATKIN_DEPENDS
  controller_interface
  franka_hw
  geometry_msgs
  hardware_interface
  message_runtime
  pluginlib
  realtime_tools
  roscpp
DEPENDS Franka)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(hiro_panda_moveit src/hiro_panda_moveit.cpp)
target_link_libraries(hiro_panda_moveit ${catkin_LIBRARIES})

add_library(hiro_panda_controllers
  src/cartesian_velocity_controller.cpp
)

target_link_libraries(hiro_panda_controllers PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(hiro_panda_controllers SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Installation
install(TARGETS hiro_panda_controllers
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES hiro_panda_controllers_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
