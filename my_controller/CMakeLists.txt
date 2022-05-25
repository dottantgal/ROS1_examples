cmake_minimum_required(VERSION 3.0.2)
project(my_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	message_generation
	roscpp
	std_msgs
	geometry_msgs
	sensor_msgs
	actionlib_msgs
	actionlib
)

################################################
## Declare ROS messages, services and actions ##
################################################
add_action_files(
  FILES Controller.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################
catkin_package(
	LIBRARIES my_controller
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs actionlib_msgs
)

###########
## Build ##
###########

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(my_controller src/my_controller.cpp)
target_link_libraries(my_controller ${catkin_LIBRARIES})

add_executable(my_controller_action src/my_controller_action.cpp)
target_link_libraries(my_controller_action ${catkin_LIBRARIES})

add_executable(simple_action src/simple_action.cpp)
target_link_libraries(simple_action ${catkin_LIBRARIES})
