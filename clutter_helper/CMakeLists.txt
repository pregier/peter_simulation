cmake_minimum_required(VERSION 2.8.3)
project(clutter_helper)

set(CMAKE_BUILD_TYPE debug)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "-std=c++11")
   message(COMPILER_SUPPORTS_CXX11)
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
    message(COMPILER_SUPPORTS_CXX0X)
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package(PCL REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
)

catkin_package(
    DEPENDS
)

add_executable(set_clutter src/set_clutter.cpp)
target_link_libraries(set_clutter ${catkin_LIBRARIES} yaml-cpp)

add_executable(clustered_clutter src/clustered_clutter.cpp)
target_link_libraries(clustered_clutter ${catkin_LIBRARIES})

add_executable(odom_to_path src/odom_to_path.cpp)
target_link_libraries(odom_to_path ${catkin_LIBRARIES})