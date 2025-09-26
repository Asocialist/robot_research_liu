cmake_minimum_required(VERSION 3.0.2)
project(node_review)

# 强制使用 C++14 标准 (ROS Noetic 推荐)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找所有依赖的包，这是现代且推荐的方式
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

# 声明 Catkin 包信息 (用于生成消息、服务等)
## Declare ROS messages and services
# add_message_files(FILES Num.msg)
# add_service_files(FILES AddTwoInts.srv)

# 1. 声明你的消息和服务文件
# add_message_files(FILES
#   Num.msg
# )
add_service_files(
  FILES
  AddTwoInts.srv
)
#add_service_files(FILES
#   AddTwoInts.srv
# )
# 2. 调用此命令来实际生成代码
generate_messages(
  DEPENDENCIES
    std_msgs # 如果你的.msg/.srv文件内部使用了std_msgs类型
)
# 3. 为你的包“贴上标签”，声明依赖关系
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_awesome_library
  CATKIN_DEPENDS roscpp std_msgs message_runtime
#  DEPENDS system_lib
)


###########
#  构建   #
###########

# 指定头文件目录
# 这使得你在代码中可以直接 #include "my_awesome_package/my_header.h"
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## ------------------ 构建可执行文件 (节点) ------------------
# 为目标链接 Catkin 库
add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
# add_dependencies(talker ${PROJECT_NAME}_generate_messages_cpp)
add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
# add_dependencies(listener ${PROJECT_NAME}_generate_messages_cpp)



add_executable(add_two_ints_server src/add_two_ints_server.cpp)
target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})
add_dependencies(add_two_ints_server ${PROJECT_NAME}_gencpp)

add_executable(add_two_ints_client src/add_two_ints_client.cpp)
target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})
add_dependencies(add_two_ints_client ${PROJECT_NAME}_gencpp)





## ------------------ 构建库 (可选) ------------------
# add_library(my_awesome_library src/my_library.cpp)
#
# # 为目标链接 Catkin 库
# target_link_libraries(my_awesome_library
#   ${catkin_LIBRARIES}
# )

############
#  安装   #
############
# install(...)
catkin_install_python(
  PROGRAMS
    scripts/talker.py
    scripts/listener.py
    scripts/add_two_ints_server.py
    scripts/add_two_ints_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# # 安装 C++ 节点
# install(TARGETS
#   talker
#   listener
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )