cmake_minimum_required(VERSION 3.0.2)
project(lidar_people_cluster)

# Use C++14 for ROS Noetic (safe for std::unique_ptr, unordered_map, etc.)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    geometry_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    message_filters
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Build node executable
add_executable(lidar_people_cluster_node
  src/lidar_people_cluster_node.cpp
)

target_link_libraries(lidar_people_cluster_node
  ${catkin_LIBRARIES}
)

add_dependencies(lidar_people_cluster_node
  ${catkin_EXPORTED_TARGETS}
)

# Optional install rules (uncomment if you need)
# install(TARGETS lidar_people_cluster_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
#
# install(DIRECTORY launch config
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
