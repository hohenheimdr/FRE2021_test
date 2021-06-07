cmake_minimum_required(VERSION 2.8.3)
project(goal_manager)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  msgs 
  geometry_msgs 
  tf 
)

include_directories(
${catkin_INCLUDE_DIRS} 
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs

)

#find_package(Boost REQUIRED )

###########
## Build ##
###########

#this publish poses
add_executable(robot_goal_manager src/robot_goal_creater.cpp src/pathParser.cpp)
target_link_libraries(robot_goal_manager ${catkin_LIBRARIES})

# build path_ writer
add_executable(path_writer src/path_writer.cpp src/pathParser.cpp)
target_link_libraries(path_writer ${catkin_LIBRARIES})
