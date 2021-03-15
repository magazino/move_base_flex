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

find_package(Boost COMPONENTS thread chrono REQUIRED)

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
  src/controller_action.cpp
  src/planner_action.cpp
  src/recovery_action.cpp
  src/move_base_action.cpp
  src/abstract_execution_base.cpp
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

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  # gtests
  catkin_add_gtest(${MBF_ABSTRACT_SERVER_LIB}_gtest test/abstract_execution_base.cpp)
  target_link_libraries(${MBF_ABSTRACT_SERVER_LIB}_gtest ${MBF_ABSTRACT_SERVER_LIB})
  
  # ros-tests
  add_rostest_gmock(abstract_action_base_test
    test/abstract_action_base.launch
    test/abstract_action_base.cpp)
  target_link_libraries(abstract_action_base_test ${MBF_ABSTRACT_SERVER_LIB})

  add_rostest_gmock(abstract_controller_execution_test
    test/abstract_controller_execution.launch
    test/abstract_controller_execution.cpp)
  target_link_libraries(abstract_controller_execution_test ${MBF_ABSTRACT_SERVER_LIB})

  add_rostest_gmock(abstract_planner_execution_test
    test/abstract_planner_execution.launch
    test/abstract_planner_execution.cpp)
  target_link_libraries(abstract_planner_execution_test ${MBF_ABSTRACT_SERVER_LIB})

  add_rostest_gmock(planner_action_test
    test/planner_action.launch
    test/planner_action.cpp)
  target_link_libraries(planner_action_test ${MBF_ABSTRACT_SERVER_LIB})
endif()
