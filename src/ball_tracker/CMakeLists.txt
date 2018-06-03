cmake_minimum_required(VERSION 2.8.3)
project(ball_tracker)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
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

find_package(OpenCV REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(ball_tracker src/ball_tracker.cpp)

target_link_libraries(ball_tracker
   ${catkin_LIBRARIES} ${OpenCV_LIBS}
)

#############
## Install ##
#############


#############
## Testing ##
#############


