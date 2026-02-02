cmake_minimum_required(VERSION 3.0.2)
project(elderly_bot)

## Compile as C++11
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  tf2_ros
  cv_bridge
  image_transport
  diagnostic_msgs
  actionlib
  move_base_msgs
)

###################################
## Catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf
    tf2_ros
    cv_bridge
    image_transport
    diagnostic_msgs
    actionlib
    move_base_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# ─────────────────────────────────────────────
# C++ Nodes
# Only add executables that ACTUALLY exist
# and are meant to be compiled.
# ─────────────────────────────────────────────

# Example (keep commented until confirmed):
# add_executable(example_node src/example_node.cpp)
# target_link_libraries(example_node ${catkin_LIBRARIES})

#############
## Install ##
#############

## Install Python scripts
catkin_install_python(
  PROGRAMS
    scripts/*
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Install config files
install(
  DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Install URDF files
install(
  DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
