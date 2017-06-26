cmake_minimum_required(VERSION 2.8.3)
project(move_base_flex)

find_package(catkin REQUIRED
  COMPONENTS
  actionlib
  actionlib_msgs
  base_local_planner
  dynamic_reconfigure
  geometry_msgs
  nav_core
  nav_msgs
  move_base_flex_msgs
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

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  actionlib
  actionlib_msgs
  base_local_planner
  dynamic_reconfigure
  geometry_msgs
  nav_core
  nav_msgs
  move_base_flex_msgs
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

set(MBF_MOVE_BASE_SERVER mbf_2d_nav_server)
add_executable(${MBF_MOVE_BASE_SERVER}
  src/move_base_server_node.cpp
  src/move_base_server/move_base_navigation_server.cpp
  src/move_base_server/move_base_planner_execution.cpp
  src/move_base_server/move_base_controller_execution.cpp
  src/move_base_server/move_base_recovery_execution.cpp
  src/navigation_utility.cpp
  )

add_dependencies(${MBF_MOVE_BASE_SERVER} ${PROJECT_NAME}_gencfg)
add_dependencies(${MBF_MOVE_BASE_SERVER} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${MBF_MOVE_BASE_SERVER}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )

set(MBF_SIMPLE_SERVER mbf_simple_server)
add_executable(${MBF_SIMPLE_SERVER}
  src/simple_server/simple_navigation_server.cpp
  src/simple_server/simple_planner_execution.cpp
  src/simple_server/simple_controller_execution.cpp
  src/simple_server/simple_recovery_execution.cpp
  src/navigation_utility.cpp
  src/simple_server_node.cpp
  )
add_dependencies(${MBF_SIMPLE_SERVER} ${PROJECT_NAME}_gencfg)
add_dependencies(${MBF_SIMPLE_SERVER} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${MBF_SIMPLE_SERVER}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )

install(TARGETS ${MBF_MOVE_BASE_SERVER} ${MBF_SIMPLE_SERVER}
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
