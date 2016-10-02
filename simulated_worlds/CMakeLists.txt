cmake_minimum_required(VERSION 2.8.3)
project(simulated_worlds)

find_package(catkin REQUIRED)

catkin_package(
    CATKIN_DEPENDS gazebo_ros kdl_parser 
)



