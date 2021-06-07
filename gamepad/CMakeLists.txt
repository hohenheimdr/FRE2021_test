cmake_minimum_required(VERSION 2.8.3)
project(gamepad)

#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	msgs 
	differential_ifk_lib
)	

find_package(Boost REQUIRED )

## Specify additional locations of header files  (braucht es um auch unterverzeichnisse zu finden
include_directories(include 
	${catkin_INCLUDE_DIRS} 
)

catkin_package(
  CATKIN_DEPENDS roscpp
)

SET(CMAKE_CXX_FLAGS "-std=c++11")
###########
## Build ##
###########


# build joystick_core
add_executable(joystick_core src/joystick_core.cpp src/Logitech.cpp)
target_link_libraries(joystick_core ${catkin_LIBRARIES}  )




