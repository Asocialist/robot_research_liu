cmake_minimum_required(VERSION 3.0.2)
project(lidar_people_cluster_viewer)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  visualization_msgs
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(opencv_viewer_node src/opencv_viewer_node.cpp)
target_link_libraries(opencv_viewer_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_dependencies(opencv_viewer_node
  ${catkin_EXPORTED_TARGETS}
)
