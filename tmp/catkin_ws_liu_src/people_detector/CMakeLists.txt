cmake_minimum_required(VERSION 3.0.2)
project(people_detector)

# --- Compiler flags (safe defaults) ---
# NOTE: You can relax warnings or optimization if needed.
add_compile_options(-std=c++14 -O2 -Wall -Wextra)

# --- Dependencies (catkin + msgs) ---
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
)

# --- Catkin package: export include dirs / libs if any ---
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs visualization_msgs
)

# --- Include paths ---
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# --- Nodes / Executables ---
# Build the LiDAR-based people detector node from your source file.
add_executable(people_detector_lidar
  src/people_detector_lidar.cpp
)

# Link against catkin libraries.
target_link_libraries(people_detector_lidar
  ${catkin_LIBRARIES}
)

# --- Install (optional but recommended for deployment) ---
# Install the node binary into devel/install spaces.
install(TARGETS people_detector_lidar
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# If you later add headers or a launch folder, you can uncomment and adapt:
# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )
