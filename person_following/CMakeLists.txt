##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(person_following)

##############################################################################
# Catkin
##############################################################################
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  std_msgs 
  sensor_msgs
  geometry_msgs
  gndlib
)

#add_service_files(
#  FILES
#  AddDestination.srv
#)

#add_message_files(
#  FILES
#    SpurDriveControl.msg
#)

# generate_messages(
#   DEPENDENCIES
#     std_msgs
#     sensor_msgs
# )

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES person_following
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs gndlib
)


find_package(OpenCV REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS} )


##############################################################################
# Sources
##############################################################################

#add_library(person_following src/lib/person_following.cpp)
#target_link_libraries(person_following ${catkin_LIBRARIES})

#install(TARGETS person_following
#  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

#install(DIRECTORY include/person_following/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h")

add_executable(person_following
  src/person_following.cpp
)
target_link_libraries(person_following ${catkin_LIBRARIES} ${OpenCV_LIBS})
install(TARGETS person_following 
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(person_following_multi
  src/person_following_multiple.cpp
  src/json11.cpp
)
target_link_libraries(person_following_multi ${catkin_LIBRARIES} ${OpenCV_LIBS})
install(TARGETS person_following_multi 
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(person_following_OR
  src/person_following_multiple_OR.cpp
  src/json11.cpp
)
target_link_libraries(person_following_OR ${catkin_LIBRARIES} ${OpenCV_LIBS})
install(TARGETS person_following_OR 
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


##############################################################################
# Test
##############################################################################

#if(CATKIN_ENABLE_TESTING)
#  find_package(catkin COMPONENTS rostest rosunit)
#  include_directories(${GTEST_INCLUDE_DIRS})
#  catkin_add_gtest(${PROJECT_NAME}-test test/test_${PROJECT_NAME}.cpp)
#  if(TARGET ${PROJECT_NAME}-test)
#    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${GTEST_LIBRARIES})
#  endif()
#endif()

