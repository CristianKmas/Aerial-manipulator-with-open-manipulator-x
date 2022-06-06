cmake_minimum_required(VERSION 3.0.2)
project(odometria)


find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  tf
  tf2_ros
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS nav_msgs roscpp rospy tf tf2_ros)

include_directories(${catkin_INCLUDE_DIRS})





