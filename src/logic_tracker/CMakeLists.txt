cmake_minimum_required(VERSION 2.8.3)
project(logic_tracker)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
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
catkin_package(
#  INCLUDE_DIRS include
# LIBRARIES opencv_core, opencv_video, opencv_highgui
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
# DEPENDS opencv_core, opencv_video, opencv_highgui
)

# Set the compiler to c++11
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(logic_tracker src/logic_tracker.cpp)

target_link_libraries(logic_tracker
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############


#############
## Testing ##
#############


