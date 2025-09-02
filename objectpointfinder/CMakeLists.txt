cmake_minimum_required(VERSION 2.8.3)
project(hdk_objectpoint_finder)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  gndlib
  gnd_rosutil
  gnd_msgs
  sensor_msgs
)
find_package(GTest)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   ObjectPointInfo.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  srv_find_objectpoint.srv
  srv_is_in_travelable_area.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp std_msgs message_runtime gnd_msgs geometry_msgs sensor_msgs gndlib gnd_rosutil
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})


set(CMAKE_CXX_FLAGS "-std=c++11")

# hdk_objectpoint_finder
add_executable(hdk_objectpoint_finder src/hdk_objectpoint_finder.cpp)
add_dependencies(hdk_objectpoint_finder ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(hdk_objectpoint_finder ${catkin_LIBRARIES})

# find_objectpoint_by_orientation
add_executable(find_objectpoint_by_orientation src/find_objectpoint_by_orientation.cpp src/json11.cpp)
add_dependencies(find_objectpoint_by_orientation ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(find_objectpoint_by_orientation ${catkin_LIBRARIES})

# hdk_objectpoint_finder_check_function
add_executable(hdk_objectpoint_finder_check_function src/hdk_objectpoint_finder_check_function.cpp)
add_dependencies(hdk_objectpoint_finder_check_function ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(hdk_objectpoint_finder_check_function ${catkin_LIBRARIES})
set_target_properties(hdk_objectpoint_finder_check_function PROPERTIES OUTPUT_NAME check_function)

# hdk_objectpoint_finder_check_service
add_executable(hdk_objectpoint_finder_check_service src/hdk_objectpoint_finder_check_service.cpp)
add_dependencies(hdk_objectpoint_finder_check_service ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(hdk_objectpoint_finder_check_service ${catkin_LIBRARIES})
set_target_properties(hdk_objectpoint_finder_check_service PROPERTIES OUTPUT_NAME check_service)

install(TARGETS
  hdk_objectpoint_finder
  find_objectpoint_by_orientation
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#if (GTEST_FOUND)
#  add_executable(hdk_objectpoint_finder_test test/test_hdk_objectpoint_finder.cpp)
#  target_link_libraries(hdk_objectpoint_finder_test ${GTEST_BOTH_LIBRARIES} ${catkin_LIBRARIES})
#  if (WIN32)
#    set_target_properties(hdk_objectpoint_finder_test PROPERTIES COMPILE_FLAGS "/MT")
#  endif (WIN32)
#  add_test(
#    NAME hdk_objectpoint_finder_test
#    COMMAND hdk_objectpoint_finder_test
#    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
#  )
#endif (GTEST_FOUND)