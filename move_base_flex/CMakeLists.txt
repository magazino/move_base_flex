cmake_minimum_required(VERSION 2.8.3)
project(move_base_flex)

find_package(catkin REQUIRED
  COMPONENTS
  actionlib
  actionlib_msgs
  base_local_planner
  dynamic_reconfigure
  geometry_msgs
  move_base_flex_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  std_msgs
  std_srvs
  tf
  )

find_package(Boost COMPONENTS thread chrono REQUIRED)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/MoveBaseFlex.cfg
)


set(MBF_UTILITY_LIB mbf_navigation_util)
set(MBF_MOVE_BASE_SERVER_LIB mbf_2d_nav_server)
set(MBF_MOVE_BASE_SERVER_NODE mbf_2d_nav)
set(MBF_SIMPLE_SERVER_LIB mbf_simple_server)
set(MBF_SIMPLE_SERVER_NODE mbf_simple)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${MBF_UTILITY_LIB} ${MBF_MOVE_BASE_SERVER_LIB} ${MBF_SIMPLE_SERVER_LIB}
  CATKIN_DEPENDS
  actionlib
  actionlib_msgs
  base_local_planner
  dynamic_reconfigure
  geometry_msgs
  move_base_flex_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  std_msgs
  std_srvs
  tf
  DEPENDS Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(${MBF_UTILITY_LIB} src/navigation_utility.cpp)

add_library(${MBF_MOVE_BASE_SERVER_LIB}
  src/move_base_server/move_base_navigation_server.cpp
  src/move_base_server/move_base_planner_execution.cpp
  src/move_base_server/move_base_controller_execution.cpp
  src/move_base_server/move_base_recovery_execution.cpp
  )
add_dependencies(${MBF_MOVE_BASE_SERVER_LIB} ${MBF_UTILITY_LIB})
add_dependencies(${MBF_MOVE_BASE_SERVER_LIB} ${PROJECT_NAME}_gencfg)
add_dependencies(${MBF_MOVE_BASE_SERVER_LIB} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${MBF_MOVE_BASE_SERVER_LIB}
  ${MBF_UTILITY_LIB}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )

add_executable(${MBF_MOVE_BASE_SERVER_NODE} src/move_base_server_node.cpp)
add_dependencies(${MBF_MOVE_BASE_SERVER_NODE} ${MBF_MOVE_BASE_SERVER_LIB})
target_link_libraries(${MBF_MOVE_BASE_SERVER_NODE} ${MBF_MOVE_BASE_SERVER_LIB})

add_library(${MBF_SIMPLE_SERVER_LIB}
  src/simple_server/simple_navigation_server.cpp
  src/simple_server/simple_planner_execution.cpp
  src/simple_server/simple_controller_execution.cpp
  src/simple_server/simple_recovery_execution.cpp
  )

add_dependencies(${MBF_SIMPLE_SERVER_LIB} ${MBF_UTILITY_LIB})
add_dependencies(${MBF_SIMPLE_SERVER_LIB} ${PROJECT_NAME}_gencfg)
add_dependencies(${MBF_SIMPLE_SERVER_LIB} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${MBF_SIMPLE_SERVER_LIB}
  ${MBF_UTILITY_LIB}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )

add_executable(${MBF_SIMPLE_SERVER_NODE} src/simple_server_node.cpp)
add_dependencies(${MBF_SIMPLE_SERVER_NODE} ${MBF_SIMPLE_SERVER_LIB})
target_link_libraries(${MBF_SIMPLE_SERVER_NODE} ${MBF_SIMPLE_SERVER_LIB})

install(TARGETS
  ${MBF_UTILITY_LIB}
  ${MBF_MOVE_BASE_SERVER_LIB} ${MBF_MOVE_BASE_SERVER_NODE}
  ${MBF_SIMPLE_SERVER_LIB} ${MBF_SIMPLE_SERVER_NODE}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(PROGRAMS scripts/move_base_legacy_relay.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
