cmake_minimum_required(VERSION 2.8)
project(rvo_ros)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  std_msgs
  message_generation
)
find_package(gazebo REQUIRED)
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

add_service_files(
  DIRECTORY srv
  FILES SetGoals.srv
)

# add_message_files(
#         FILES
#         rvo_vel.msg
# )

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robohan
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy tf
CATKIN_DEPENDS roscpp message_runtime geometry_msgs std_msgs
)

# file(GLOB RVO_LIB_DIRS "rvo_lib/nav_rvo.cpp")

add_subdirectory(simulation/plugins)
add_library(rvolib SHARED rvo_lib/nav_rvo.cpp rvo_lib/RVOSimulator.cpp rvo_lib/Agent.cpp rvo_lib/KdTree.cpp rvo_lib/Obstacle.cpp)

add_executable(rvo_node src/rvo_node.cpp)
target_link_libraries(rvo_node ${catkin_LIBRARIES} rvolib)
add_dependencies(rvo_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(set_goals_client src/set_goals_client.cpp)
target_link_libraries(set_goals_client ${catkin_LIBRARIES})
add_dependencies(set_goals_client ${${PROJECT_NAME}_EXPORTED_TARGETS})
# add_dependencies(set_goals_client )

add_executable(formation src/formation.cpp)
target_link_libraries(formation ${catkin_LIBRARIES})
add_dependencies(formation ${${PROJECT_NAME}_EXPORTED_TARGETS})

#target_link_libraries(rvo_test RVO)
