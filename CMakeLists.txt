cmake_minimum_required(VERSION 2.8.3)
project(hiro_panda_moveit)

add_compile_options(-std=c++11)

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
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(hiro_panda_moveit src/hiro_panda_moveit.cpp)
target_link_libraries(hiro_panda_moveit ${catkin_LIBRARIES})