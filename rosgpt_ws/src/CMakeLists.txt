cmake_minimum_required(VERSION 3.0.2)
project(rosgpt)

# 查找 catkin，对于纯 Python 包，甚至可以不列出 rospy 和 std_msgs
find_package(catkin REQUIRED)

# 对于纯 Python 项目，这个 catkin_package() 调用是必须的，
# 但内部可以为空。它能确保工作区设置正确。
catkin_package()

# 安装 Python 脚本
# 这会将你的脚本安装到 setup.bash 加载的环境路径中
# 这样你就可以使用 rosrun 来执行它
install(PROGRAMS
  scripts/rosgpt.py  # 假设你的 rosgpt.py 文件在 'scripts' 目录下
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装其他文件，例如 launch 文件 [cite: 10]
# install(FILES
#   launch/my_launch_file.launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )