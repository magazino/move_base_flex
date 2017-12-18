cmake_minimum_required(VERSION 2.8.3)
project(move_base_flex)

find_package(catkin REQUIRED COMPONENTS
  mbf_abstract_core
  mbf_abstract_nav
  mbf_costmap_core
  mbf_costmap_nav
  mbf_msgs
  mbf_simple_nav
)

catkin_package(CATKIN_DEPENDS 
  mbf_abstract_core
  mbf_abstract_nav
  mbf_costmap_core
  mbf_costmap_nav
  mbf_msgs
  mbf_simple_nav
)
