cmake_minimum_required(VERSION 2.8.3)
project(elderly_bot)

## Compile as C++11
add_compile_options(-std=c++11)

## Find catkin and all required components
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf
  tf2_ros
  move_base
  move_base_msgs
  actionlib
  actionlib_msgs
  amcl
  map_server
  navfn
  dwa_local_planner
  costmap_2d
  gmapping
  explore_lite
  robot_localization
  rplidar_ros
  rosserial_python
  rosserial_arduino
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    rospy
    roscpp
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    tf
    tf2_ros
    move_base
    move_base_msgs
    actionlib
    actionlib_msgs
    amcl
    map_server
    navfn
    dwa_local_planner
    costmap_2d
    gmapping
    explore_lite
    robot_localization
    rplidar_ros
    rosserial_python
    rosserial_arduino
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/patrol_client.py
  scripts/mpu9250_node.py
  scripts/sensors_actuators_node.py
  scripts/cloud_bridge_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)

# Install maps directory
install(DIRECTORY maps/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps
)

# Install firmware
install(DIRECTORY firmware/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/firmware
)

# Install scripts
install(PROGRAMS
  install_dependencies.sh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


