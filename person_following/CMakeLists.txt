cmake_minimum_required(VERSION 2.8.3)
project(person_following)

# CatkinパッケージとOpenCVを探す
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  cv_bridge
  image_transport
)

# OpenCV 4が必須
find_package(OpenCV 4 REQUIRED)

# メッセージ生成
generate_messages(
  DEPENDENCIES
    std_msgs
    sensor_msgs
)

# Catkinパッケージ情報
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime cv_bridge image_transport
)

# インクルードディレクトリの設定
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# --- 実行ファイルのビルド設定 ---
# [修正]: json11.cpp をコンパイル対象に戻します。CVLegacy.cpp は不要なため除外したままです。

add_executable(person_following src/person_following.cpp src/MyCondensation.cpp src/MyEllipseNormalEvaluation.cpp src/json11.cpp)
target_link_libraries(person_following ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
install(TARGETS person_following
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(person_following_multi src/person_following_multiple.cpp src/MyCondensation.cpp src/MyEllipseNormalEvaluation.cpp src/json11.cpp)
target_link_libraries(person_following_multi ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
install(TARGETS person_following_multi
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(person_following_OR src/person_following_multiple_OR.cpp src/MyCondensation.cpp src/MyEllipseNormalEvaluation_2LS.cpp src/json11.cpp)
target_link_libraries(person_following_OR ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
install(TARGETS person_following_OR
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
