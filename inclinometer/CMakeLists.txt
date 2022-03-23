cmake_minimum_required(VERSION 3.0.2)
project(inclinometer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  uavcan_msgs
  tf2_ros
)

catkin_package(
 CATKIN_DEPENDS std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tf_broadcaster_node src/tf_broadcaster.cpp)
add_dependencies(tf_broadcaster_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tf_broadcaster_node
  ${catkin_LIBRARIES}
)
