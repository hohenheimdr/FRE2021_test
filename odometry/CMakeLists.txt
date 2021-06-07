cmake_minimum_required(VERSION 2.8.3)
project(odometry)

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
	cfg/cpp
)

catkin_package(
	
  CATKIN_DEPENDS roscpp
)

SET(CMAKE_CXX_FLAGS "-std=c++11")
###########
## Build ##
###########

# build differential ifk
add_executable(differential_odometry src/differential_odometry_node.cpp)
target_link_libraries(differential_odometry ${catkin_LIBRARIES})

