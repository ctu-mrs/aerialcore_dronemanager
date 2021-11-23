cmake_minimum_required(VERSION 3.1.2)
project(drone_manager)

find_package(catkin REQUIRED COMPONENTS
  rospy
  mrs_msgs
)

catkin_package(
   CATKIN_DEPENDS rospy mrs_msgs
)
