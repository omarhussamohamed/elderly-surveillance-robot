cmake_minimum_required(VERSION 3.10)
project(elderly_bot)

add_compile_options(-std=c++11)

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
  dwa_local_planner
  costmap_2d
  gmapping
  explore_lite
  robot_localization
  rplidar_ros
  rosserial_python
  rosserial_arduino
  cv_bridge
  image_transport
  compressed_image_transport
  diagnostic_aggregator
)

catkin_package(
  CATKIN_DEPENDS
    rospy roscpp std_msgs geometry_msgs sensor_msgs nav_msgs tf tf2_ros
    move_base move_base_msgs actionlib actionlib_msgs amcl map_server
    dwa_local_planner costmap_2d gmapping explore_lite robot_localization
    rplidar_ros rosserial_python rosserial_arduino cv_bridge image_transport
    compressed_image_transport diagnostic_aggregator
)

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/camera_node.py
  scripts/cloud_bridge_node.py
  scripts/kvs_streamer_node.py
  scripts/mpu9250_node.py
  scripts/patrol_client.py
  scripts/sensors_actuators_node.py
  scripts/system_health_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install directories
install(DIRECTORY launch/ config/ maps/ firmware/ urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch" PATTERN "*.yaml" PATTERN "*.pgm" PATTERN "*.ino" PATTERN "*.urdf"
)

install(PROGRAMS install_dependencies.sh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)