cmake_minimum_required(VERSION 3.0.2)
project(mbf_utility)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mbf_msgs
  roscpp
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mbf_utility
  CATKIN_DEPENDS geometry_msgs mbf_msgs roscpp tf tf2 tf2_ros tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
   src/navigation_utility.cpp
   src/robot_information.cpp
   src/odometry_helper.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
