cmake_minimum_required(VERSION 2.8.3)
project(simulated_navigation_launch)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  actionlib
  geometry_msgs
  move_base_msgs
  tf
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})


catkin_package(
    DEPENDS
    CATKIN_DEPENDS roscpp actionlib tf move_base_msgs geometry_msgs
 #   INCLUDE_DIRS
 #   LIBRARIES
)

add_executable(simple_goal_publisher src/simple_goal_publisher.cpp)
target_link_libraries(simple_goal_publisher ${catkin_LIBRARIES})