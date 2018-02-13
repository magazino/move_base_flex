cmake_minimum_required(VERSION 2.8.3)
project(mbf_abstract_nav)

find_package(catkin REQUIRED
  COMPONENTS
  actionlib
  actionlib_msgs
  dynamic_reconfigure
  geometry_msgs
  mbf_msgs
  mbf_abstract_core
  mbf_utility
  nav_msgs
  roscpp
  std_msgs
  std_srvs
  tf
  xmlrpcpp
  )

find_package(Boost COMPONENTS thread chrono exception REQUIRED)

# dynamic reconfigure: we provide the abstract configuration common to all MBF-based navigation
# frameworks in a python module, so it can easily be included in particular navigation flavours
catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/MoveBaseFlex.cfg
)

set(MBF_ABSTRACT_SERVER_LIB mbf_abstract_server)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${MBF_ABSTRACT_SERVER_LIB}
  CATKIN_DEPENDS
      actionlib
      actionlib_msgs
      dynamic_reconfigure
      geometry_msgs
      mbf_msgs
      mbf_abstract_core
      mbf_utility
      nav_msgs
      roscpp
      std_msgs
      std_srvs
      tf
      xmlrpcpp
  DEPENDS Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(${MBF_ABSTRACT_SERVER_LIB}
  src/abstract_navigation_server.cpp
  src/abstract_planner_execution.cpp
  src/abstract_controller_execution.cpp
  src/abstract_recovery_execution.cpp
  )
add_dependencies(${MBF_ABSTRACT_SERVER_LIB} ${PROJECT_NAME}_gencfg)
add_dependencies(${MBF_ABSTRACT_SERVER_LIB} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${MBF_ABSTRACT_SERVER_LIB}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )

install(TARGETS
  ${MBF_ABSTRACT_SERVER_LIB}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

