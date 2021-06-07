cmake_minimum_required(VERSION 2.8.3)
project(mode_changer)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	msgs 
	geometry_msgs 
	)
find_package(Boost REQUIRED )

## Specify additional locations of header files
include_directories(include 
	${catkin_INCLUDE_DIRS} 
	${Boost_INCLUDE_DIRS}
	cfg/cpp
)
catkin_package(
  CATKIN_DEPENDS roscpp msgs
)


SET(CMAKE_CXX_FLAGS "-std=c++11")
###########
## Build ##
###########

# build mode changer
add_executable(mode_changer_twist src/mode_changer_twist.cpp)
target_link_libraries(mode_changer_twist ${catkin_LIBRARIES})
