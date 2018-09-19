cmake_minimum_required(VERSION 2.8.3)
project(mbf_costmap_core)

find_package(catkin REQUIRED
        COMPONENTS
            std_msgs
            geometry_msgs
            mbf_abstract_core
            tf
            costmap_2d
            nav_core
        )

catkin_package(
    INCLUDE_DIRS
        include
    CATKIN_DEPENDS
            std_msgs
            geometry_msgs
            mbf_abstract_core
            tf
            costmap_2d
            nav_core
)

if("$ENV{ROS_DISTRO}" STREQUAL "melodic")
  add_definitions(-DCOSTMAP_HAS_TF2)
  message("melodic compatible build")
endif()

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)
