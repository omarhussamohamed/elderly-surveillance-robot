# System Architecture Overview — Current Truth

**Last Updated**: January 27, 2026

This is the **only authoritative source** for system architecture. Do not trust other documents.

**Primary Visualization**: Foxglove Bridge (ws://<jetson_ip>:8765) — /map, /scan, /tf, /odom, /gas_detected, /jetson_temperature

---

## Mission

Autonomous indoor monitoring robot for elderly care with:
- SLAM mapping & autonomous exploration
- Waypoint-based patrol navigation
- Real-time gas leak detection with buzzer alerts
- Live video streaming to AWS Kinesis Video Streams
- Cloud telemetry, alerts, and remote commands via AWS IoT Core

---

## ROS Nodes by Category

### Perception
- camera_node.py → /camera/image_raw (USB /dev/video0, 1280×720 @15 fps)
- mpu9250_node.py → /imu/data_raw (I2C bus 1, address 0x68)
- imu_filter_madgwick → /imu/data (orientation fusion)
- rplidar_node → /scan (rear-mounted, backward-facing)

### Localization & Mapping
- ekf_localization_node (robot_localization) → odom → base_footprint  
  (single EKF, no wheel vyaw fusion, high yaw covariance for skid-steer slip)
- gmapping → map → odom TF, /map topic (1.0 s update interval, 500 particles, likelihood_field model)
- amcl → map → odom TF (500 particles, tf_broadcast: true)
- explore_lite → autonomous frontier exploration (mapping mode)

### Navigation & Control
- move_base → /cmd_vel (Navfn global planner + DWA local planner)
- patrol_client.py → cycles waypoints from patrol_goals.yaml (3 retries on failure, success rate logging)
- esp32_serial_node (rosserial_python) → /odom (20 Hz) from ESP32, /cmd_vel to motors

### Monitoring & Actuators
- sensors_actuators_node.py
  - MQ-6 gas sensor (GPIO BOARD pin 18) → /gas_detected (Bool)
  - Buzzer (pin 16) auto-trigger on gas, 5 s timeout
  - Jetson stats → /jetson_temperature, /jetson_power
- system_health_monitor.py → /diagnostics (checks every 10 s, auto-respawn critical nodes)

### Cloud & Video
- cloud_bridge_node.py → AWS IoT Core (telemetry, alerts, commands; certificates via env vars)
- kvs_streamer_node.py → AWS KVS (720p @15 fps, nvh264enc hardware encoding)

---

## TF Tree
- map → odom (AMCL or GMapping)
- odom → base_footprint (EKF)
- base_footprint → base_link (fixed joint, z = 0.0825 m)
- base_link → laser (LiDAR)
- base_link → imu_link (IMU)
- base_link → camera_link (camera, xyz="0.15 0 0.25")

---

## Key Configurations
- Costmaps: merged in costmap_params.yaml (inflation_radius: 0.30 m, resolution: 0.05 m)
- EKF: ekf.yaml (two_d_mode: true, no wheel vyaw, high yaw covariance)
- AMCL: amcl.yaml (500 particles, tf_broadcast: true)
- GMapping: gmapping.yaml (1.0 s map_update_interval, 500 particles)
- DWA: dwa_local_planner.yaml (max_vel_x: 0.25 m/s, holonomic_robot: false)
- Patrol: patrol_goals.yaml (example waypoints; adjust via RViz 2D Pose Estimate if needed)

---

## Dependencies
- **ROS Melodic packages** (apt): navigation, robot_localization, gmapping, amcl, move_base, dwa_local_planner, rplidar_ros, rosserial_python, imu_filter_madgwick, diagnostic_aggregator, cv_bridge, image_transport, compressed_image_transport
- **Python 2 packages** (pip): paho-mqtt, smbus2, Jetson.GPIO, jetson-stats, pyyaml, opencv-python
- **Other**: GStreamer (KVS streaming), Arduino IDE (ESP32 firmware)
- **One-command install**: `./install_dependencies.sh` (idempotent, includes udev rules for RPLidar)

---

## Simplified Launch Architecture
- bringup.launch: Hardware bringup + EKF + sensors + cloud (respawn enabled on critical nodes)
- mapping.launch: bringup + GMapping + explore_lite
- navigation.launch: bringup + map_server + AMCL + move_base + Foxglove Bridge
- kvs_stream.launch: camera_node + kvs_streamer_node (respawn enabled)

---

## File Tree (Current Structure)
elderly_bot/
├── aws_certs/                  # Certificates (loaded via env vars)
├── config/                     # YAML configs (merged costmap_params.yaml)
├── firmware/                   # ESP32 .ino
├── launch/                     # bringup, mapping, navigation, kvs_stream
├── maps/                       # my_house.yaml, .pgm
├── rviz/                       # Deprecated configs only
├── scripts/                    # All Python nodes
├── urdf/                       # elderly_bot.urdf
├── docs/                       # TROUBLESHOOTING.md
├── CHANGELOG.md
├── CMakeLists.txt
├── HARDWARE.md
├── install_dependencies.sh
├── package.xml
├── README.md
└── SYSTEM_OVERVIEW.md