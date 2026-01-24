# System Architecture Overview — Current Truth

**Last Updated**: January 24, 2026

This is the **only authoritative source** for system architecture. Do not trust other documents.

---

## Mission

Indoor autonomous monitoring robot with SLAM mapping, waypoint navigation, and gas detection alerts.

---

## ROS Nodes by Category

### Perception
- **rplidar_node** (rplidar_ros): 2D laser scans → `/scan`
- **mpu9250_node** (scripts/): Raw IMU data → `/imu/data_raw`
- **imu_filter_madgwick** (imu_filter_madgwick): IMU sensor fusion → `/imu/data`

### Localization & Mapping
- **robot_localization/ekf_localization_node**: Fuses wheel odom + IMU → `odom→base_footprint` TF
- **gmapping** (SLAM mode): Creates map from /scan → `map→odom` TF, publishes `/map`
- **amcl** (Navigation mode): Localizes on known map → `map→odom` TF

### Navigation
- **move_base**: Path planning and obstacle avoidance
  - Global planner: NavfnROS (Dijkstra)
  - Local planner: DWA (Dynamic Window Approach)
  - Subscribes: `/scan`, `/odom`, `/map`, `/tf`
  - Publishes: `/cmd_vel`
- **patrol_client** (scripts/): Autonomous waypoint patrol using move_base action client

### Motor Control
- **ESP32 firmware** (Arduino): Receives `/cmd_vel`, controls motors, publishes `/odom`
- **rosserial_server_node** (rosserial_server): TCP bridge between Jetson and ESP32

### Monitoring & Alerts
- **sensors_actuators_node** (scripts/): Gas sensor, buzzer, Jetson stats
  - Publishes: `/gas_detected`, `/jetson_temperature`, `/jetson_power`
  - Subscribes: `/buzzer_command`
- **kvs_streamer_node** (scripts/): AWS Kinesis Video Streams integration
  - Subscribes: `/camera/image_raw`
  - Publishes: Encoded H.264 video to AWS KVS (eu-west-1, stream: RobotStream)
  - Status: **Production Ready** (kvssink plugin compiled at `/home/omar/amazon-kinesis-video-streams-producer-sdk-cpp/build/libgstkvssink.so`)
  - Note: Requires Python 2.7 (ROS Melodic), GStreamer 1.0, kvssink plugin in GST_PLUGIN_PATH
- **cloud_bridge_node** (scripts/, optional): AWS IoT Core integration
  - Region: eu-north-1 (endpoint: a1k8itxfx77i0w-ats.iot.eu-north-1.amazonaws.com)
  - Publishes telemetry to `elderly_bot/telemetry`, alerts to `elderly_bot/alerts`
  - Subscribes to `elderly_bot/commands`
  - Note: Different region from KVS (eu-west-1) — service-specific regional deployment

### Transforms
- **robot_state_publisher**: Broadcasts URDF-defined static TFs (base_link, laser, imu_link)

---

## Core Topics

| Topic | Type | Publisher | Subscriber(s) | Purpose |
|-------|------|-----------|---------------|---------|
| `/cmd_vel` | Twist | move_base | ESP32 firmware | Motor velocity commands |
| `/odom` | Odometry | ESP32 firmware | ekf, amcl, move_base | Wheel encoder odometry |
| `/scan` | LaserScan | rplidar_node | gmapping, amcl, move_base | 2D laser scans |
| `/imu/data_raw` | Imu | mpu9250_node | imu_filter_madgwick | Raw gyro + accel |
| `/imu/data` | Imu | imu_filter_madgwick | ekf | Fused IMU with orientation |
| `/map` | OccupancyGrid | map_server or gmapping | amcl, move_base | Occupancy grid map |
| `/camera/image_raw` | Image | camera_node | kvs_streamer_node | Raw camera frames (1280x720 BGR) |
| `/gas_detected` | Bool | sensors_actuators_node | cloud_bridge (optional) | Gas detection status |
| `/buzzer_command` | Bool | cloud_bridge or manual | sensors_actuators_node | Buzzer control |

---

## Coordinate Frames (TF Tree)

```
map                          [gmapping or amcl, only when map exists]
 └── odom                    [ekf_localization_node or amcl]
     └── base_footprint      [ekf_localization_node]
         └── base_link       [robot_state_publisher]
             ├── laser       [robot_state_publisher, 180° yaw rotation]
             └── imu_link    [robot_state_publisher]
```

**TF Publishers**:
- `robot_state_publisher`: All URDF-defined transforms (static or from robot_description)
- `ekf_localization_node`: `odom→base_footprint` (fused wheel odom + IMU)
- `gmapping` or `amcl`: `map→odom` (localization in map frame)

**Critical Rules**:
- Only ONE node publishes `odom→base_footprint` (currently: ekf)
- Only ONE node publishes `map→odom` (gmapping in SLAM mode, amcl in nav mode)
- ESP32 firmware publishes `/odom` topic but NOT the `odom→base_footprint` TF

---

## Parameter Namespaces

All configurations in `~/catkin_ws/src/elderly_bot/config/`:

- `amcl.yaml` — AMCL localization params
- `cloud_config.yaml` — AWS IoT Core settings
- `costmap_common_params.yaml` — Shared costmap config
- `dwa_local_planner.yaml` — Local planner (DWA)
- `ekf.yaml` — robot_localization EKF fusion config
- `global_costmap.yaml` — Global planner costmap
- `gmapping.yaml` — GMapping SLAM parameters
- `local_costmap.yaml` — Local planner costmap
- `patrol_goals.yaml` — Waypoints for patrol mode
- `sensors_actuators.yaml` — Gas sensor, buzzer, Jetson stats config

---

## Launch Entry Points

### Main Launch Files

| Launch File | Purpose | Required Hardware |
|-------------|---------|-------------------|
| `bringup.launch` | Full system with sensors, IMU, optional cloud | All |
| `mapping.launch` | SLAM mode (create new map) | LiDAR, odom, IMU |
| `navigation.launch` | Autonomous navigation (use existing map) | All + map file |
| `imu_nav.launch` | IMU + sensor fusion only | IMU only |
| `kvs_stream.launch` | AWS KVS video streaming (production: 640x480@15fps) | USB camera |
| `camera_streaming.launch` | Camera + KVS (legacy, use kvs_stream.launch) | USB camera |

**Typical Usage**:
```bash
# SLAM (create map)
roslaunch elderly_bot mapping.launch

# Save map
rosrun map_server map_saver -f ~/maps/my_map

# Navigation (use map)
roslaunch elderly_bot navigation.launch map_file:=/path/to/my_map.yaml

# Autonomous patrol
roslaunch elderly_bot navigation.launch
rosrun elderly_bot patrol_client.py
```

---

## Operational Modes

### 1. Mapping Mode
- **Launch**: `mapping.launch`
- **Active**: gmapping, rplidar, ekf, imu_filter_madgwick, mpu9250_node
- **Output**: `/map` topic (OccupancyGrid)
- **Control**: Teleop or manual joystick to drive robot
- **Save Map**: `rosrun map_server map_saver -f map_name`

### 2. Navigation Mode
- **Launch**: `navigation.launch map_file:=...`
- **Active**: amcl, move_base, rplidar, ekf, imu_filter_madgwick, mpu9250_node
- **Input**: 2D Nav Goal in RViz or patrol_client
- **Output**: `/cmd_vel` to motors

### 3. Patrol Mode
- **Launch**: `navigation.launch` + `patrol_client.py`
- **Behavior**: Cycles through waypoints from `patrol_goals.yaml`
- **Recovery**: Auto-retry failed goals (max 3 retries)

---

## Known Limitations

### Not Implemented
- **Battery voltage monitoring**: Manual check with multimeter required
- **Dynamic obstacle avoidance**: Limited to static/slow-moving obstacles (DWA local planner constraints)
- **Multi-floor mapping**: Single-floor only
- **Outdoor operation**: Not weather-sealed, WiFi range limited

### Hardware Constraints
- **Magnetometer disabled**: No absolute heading, relies on gyro integration (drift over time)
- **2D LiDAR only**: Cannot detect overhangs, low obstacles (e.g., table edges)
- **Skid-steer kinematics**: Odometry error on slippery surfaces
- **WiFi dependency**: ESP32-Jetson communication requires stable WiFi (no wired fallback)

### Software Limitations
- **No collision recovery**: Robot will get stuck if physically wedged
- **AMCL requires good initial pose**: Manual 2D Pose Estimate in RViz needed after large movements
- **Gas sensor burn-in**: MQ-6 requires 24-48 hours of continuous power for stable operation

### Resource Constraints (AWS KVS)
- **CPU-intensive encoding**: KVS H.264 encoding constrained to protect navigation stack
  - Resolution: 640x480 (downscaled from camera's 1280x720)
  - Encoder preset: ultrafast (minimal CPU impact on move_base/DWA)
  - Bitrate: 512 kbps (low-bandwidth mode)
- **Memory overhead**: kvssink plugin + dependencies ~150MB RSS
- **Network dependency**: Requires stable WiFi for AWS KVS streaming (stream will auto-restart on reconnect)

---

## Debugging Tools

```bash
# View TF tree
rosrun tf view_frames && evince frames.pdf

# Monitor topics
rostopic hz /scan /odom /imu/data

# Check node status
rosnode list
rosnode info /move_base

# RViz visualization
rviz -d $(rospack find elderly_bot)/rviz/navigation.rviz

# ROS logs
cat ~/.ros/log/latest/rosout.log

# AWS KVS debugging
gst-inspect-1.0 kvssink  # Verify plugin is loaded
rostopic hz /camera/image_raw  # Verify camera feed
rostopic echo /kvs/streaming  # Check KVS stream status
```

---

## Safety Notes

- Gas sensor alarm is **advisory only** — not suitable for life-safety applications
- Robot will not stop automatically if gas is detected (requires external monitoring)
- No emergency stop button on robot (power switch only)
- IMU calibration requires stationary robot for 5-10 seconds at startup


