cmake_minimum_required(VERSION 3.0.2)
project(mbf_simple_nav)

find_package(catkin REQUIRED
  COMPONENTS
  actionlib
  actionlib_msgs
  dynamic_reconfigure
  geometry_msgs
  mbf_abstract_nav
  mbf_msgs
  mbf_abstract_core
  nav_msgs
  pluginlib
  roscpp
  std_msgs
  std_srvs
  tf
  tf2
  tf2_ros
  )

find_package(Boost COMPONENTS thread chrono REQUIRED)

set(MBF_SIMPLE_SERVER_LIB mbf_simple_server)
set(MBF_SIMPLE_SERVER_NODE mbf_simple_nav)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${MBF_SIMPLE_SERVER_LIB}
  CATKIN_DEPENDS
  actionlib
  actionlib_msgs
  dynamic_reconfigure
  geometry_msgs
  mbf_abstract_nav
  mbf_msgs
  mbf_abstract_core
  nav_msgs
  pluginlib
  roscpp
  std_msgs
  std_srvs
  tf
  tf2
  tf2_ros
  DEPENDS Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(${MBF_SIMPLE_SERVER_LIB}
  src/simple_navigation_server.cpp
)

add_dependencies(${MBF_SIMPLE_SERVER_LIB} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${MBF_SIMPLE_SERVER_LIB}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(${MBF_SIMPLE_SERVER_NODE} src/simple_server_node.cpp)
add_dependencies(${MBF_SIMPLE_SERVER_NODE} ${MBF_SIMPLE_SERVER_LIB})
target_link_libraries(${MBF_SIMPLE_SERVER_NODE}
  ${MBF_SIMPLE_SERVER_LIB}
  ${catkin_LIBRARIES})

install(TARGETS
  ${MBF_SIMPLE_SERVER_LIB} ${MBF_SIMPLE_SERVER_NODE}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

