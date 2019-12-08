cmake_minimum_required(VERSION 2.8)
project(rvo_ros)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robohan
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy tf
  DEPENDS 
    roscpp 
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# file(GLOB RVO_LIB_DIRS "rvo_lib/nav_rvo.cpp")

add_library(rvolib SHARED rvo_lib/nav_rvo.cpp rvo_lib/RVOSimulator.cpp rvo_lib/Agent.cpp rvo_lib/KdTree.cpp rvo_lib/Obstacle.cpp)

add_executable(rvo_node src/rvo_node.cpp)
target_link_libraries(rvo_node ${catkin_LIBRARIES} rvolib)

#target_link_libraries(rvo_test RVO)
