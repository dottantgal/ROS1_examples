cmake_minimum_required(VERSION 3.0.2)
project(occupancy_grid_test)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  tf2
	tf2_ros
	tf2_geometry_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################



################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES occupacy_grid_test
	CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs tf2 tf2_ros tf2_geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(occupancy_grid_test src/occupancy_grid_test.cpp)
target_link_libraries(occupancy_grid_test ${catkin_LIBRARIES})

#############
## Install ##
#############


#############
## Testing ##
#############