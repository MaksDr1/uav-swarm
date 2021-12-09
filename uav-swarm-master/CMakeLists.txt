cmake_minimum_required(VERSION 3.0.2)
project(uav-swarm)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	mavros_msgs
	roscpp
	std_msgs
)

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIRS})
link_directories(/usr/local/include)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES uav-swarm
  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp std_msgs
)



include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)




add_executable(offb_node_waypoint_control
  src/offb_node_waypoint_control.cpp
)
target_link_libraries(offb_node_waypoint_control
  ${catkin_LIBRARIES}
)
add_executable(offb_node_formation_control
  src/offb_node_formation_control.cpp
)
target_link_libraries(offb_node_formation_control
  ${catkin_LIBRARIES}
)
