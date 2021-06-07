cmake_minimum_required(VERSION 2.8.3)
project(motorcontroller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	msgs 
	geometry_msgs 
	tf
	)
find_package(Boost REQUIRED )

## Specify additional locations of header files
include_directories(include 
	${catkin_INCLUDE_DIRS} 
	${Boost_INCLUDE_DIRS}
	cfg/cpp
)


catkin_package(
	
  CATKIN_DEPENDS roscpp
)


SET(CMAKE_CXX_FLAGS "-std=c++11")
###########
## Build ##
###########


# build motorcontroller core
add_executable(sdc2130_core src/sdc2130_core.cpp src/SDC2130.cpp src/RoboteqDevice.cpp)
target_link_libraries(sdc2130_core ${catkin_LIBRARIES}  )


