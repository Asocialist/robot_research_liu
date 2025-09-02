cmake_minimum_required(VERSION 3.0.2)
project(liu_formation_track)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  gnd_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(robot_person_following_node src/robot_person_following_node.cpp)
add_dependencies(robot_person_following_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_person_following_node ${catkin_LIBRARIES})
