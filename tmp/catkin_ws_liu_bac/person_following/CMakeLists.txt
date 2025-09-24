cmake_minimum_required(VERSION 3.0.2)
project(person_following)

# 设置C++标准为C++14，ROS Noetic推荐
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找Catkin和必要的ROS包
# 添加了 cv_bridge
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  gndlib
)

# 查找OpenCV 4，并明确指定需要的组件
# opencv_video_legacy 对于使用 CvConDensation 至关重要！
find_package(OpenCV 4 REQUIRED COMPONENTS
  opencv_core
  opencv_highgui
  opencv_imgproc
  opencv_video
  
)

# Catkin包的配置
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs cv_bridge gndlib
)

# 包含头文件目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# === 我们重点关注的节点 ===
# 确保包含了所有的 .cpp 源文件
add_executable(person_following_OR
  src/person_following_multiple_OR.cpp
  src/MyCondensation.cpp
  src/MyEllipseNormalEvaluation_2LS.cpp
  src/json11.cpp
)

# 链接库，${OpenCV_LIBRARIES} 是由新的 find_package(OpenCV) 命令提供的
target_link_libraries(person_following_OR
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(TARGETS person_following_OR
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# === 其他节点（您可以按照上面的新模式自行添加回来） ===
# 为了清晰起见，我已将其他节点的定义注释掉。
# 如果您还需要它们，请仿照上面的例子进行修改。
#
# add_executable(person_following
#   src/person_following.cpp
# )
# target_link_libraries(person_following ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
# install(TARGETS person_following RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})