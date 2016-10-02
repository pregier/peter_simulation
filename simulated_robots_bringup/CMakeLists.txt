# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(simulated_robots_bringup)

find_package(catkin REQUIRED COMPONENTS)


catkin_package(
    CATKIN_DEPENDS gazebo_ros 
    DEPENDS 
    INCLUDE_DIRS 
    LIBRARIES
)

