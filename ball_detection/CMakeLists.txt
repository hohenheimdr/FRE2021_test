cmake_minimum_required(VERSION 2.8.3)
project(ball_detection)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  #usb_cam
  cv_bridge
  tf
  sensor_msgs
  std_msgs
  pcl_ros
  image_transport
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
${catkin_INCLUDE_DIRS} 
)

catkin_package(
  CATKIN_DEPENDS roscpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
	include
	${Boost_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS} 
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Declare a cpp executable
add_executable(ball_detection src/ball_detection.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(ball_detection ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} )
