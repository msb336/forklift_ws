cmake_minimum_required(VERSION 2.8.3)
project(centralized_logic)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  message_generation
  message_runtime
)


catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
  ${catkin_INCLUDE_DIRS}
)