cmake_minimum_required(VERSION 2.8.3)
project(mode_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	msgs 
	cv_bridge
	image_transport
 	dynamic_reconfigure
	geometry_msgs 
	tf
	msgs 
	
	)
find_package( PCL REQUIRED )
find_package(Boost REQUIRED )

## Specify additional locations of header files
include_directories(include 
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS} 
	#${YAML_CPP_SOURCE_DIR}
	${Boost_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	cfg/cpp
)

generate_dynamic_reconfigure_options(

)

catkin_package(
	
  CATKIN_DEPENDS roscpp dynamic_reconfigure
)


SET(CMAKE_CXX_FLAGS "-std=c++11")
###########
## Build ##
###########

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# build mode control task1
add_executable(mode_control_toggle src/mode_control_toggle.cpp)
target_link_libraries(mode_control_toggle ${catkin_LIBRARIES})

# build mode control task2
add_executable(mode_control_task2 src/mode_control_task2.cpp)
target_link_libraries(mode_control_task2 ${catkin_LIBRARIES})

# build mode control task3
add_executable(mode_control_task3 src/mode_control_task3.cpp)
target_link_libraries(mode_control_task3 ${catkin_LIBRARIES})

# build mode control task4
#add_executable(mode_control_task4 src/mode_control_task4.cpp)
#target_link_libraries(mode_control_task4 ${catkin_LIBRARIES})

