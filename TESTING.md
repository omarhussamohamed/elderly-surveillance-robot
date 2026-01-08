# Testing Guide - Elderly Robot

This document provides comprehensive testing procedures for the Elderly Robot system.

## Quick Test Checklist

- [ ] ESP32 powers on and connects to WiFi
- [ ] rosserial connection established (`rostopic list` shows ESP32 topics)
- [ ] RPLidar publishes `/scan` topic
- [ ] Motor control responds to `/cmd_vel` commands
- [ ] Odometry data is published and reasonable
- [ ] IMU data is published (if connected)
- [ ] EKF fusion working (`/odom` topic available)
- [ ] Navigation stack loads without errors
- [ ] RViz displays all transforms correctly

## Component-Level Testing

### ESP32 Firmware Testing

```bash
# 1. Check rosserial connection
rosrun rosserial_python socket_node.py _host:=192.168.1.16 _port:=11411

# 2. Verify topics
rostopic list | grep wheel_odom
rostopic hz /wheel_odom

# 3. Test motor control
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.1}" -r 5
```

### Sensor Testing

```bash
# RPLidar
roslaunch rplidar_ros view_rplidar.launch
rostopic hz /scan

# IMU (if connected to Jetson)
rosrun rqt_plot rqt_plot /imu/data/linear_acceleration/x /imu/data/angular_velocity/z
```

### Navigation Testing

```bash
# Bringup test
roslaunch elderly_bot bringup.launch

# Check all transforms
rosrun tf view_frames
evince frames.pdf

# Test EKF
rostopic echo /odom
```

## Performance Benchmarks

### Expected Performance
- **rosserial latency**: < 10ms
- **Odometry update rate**: 10 Hz
- **Motor response time**: < 50ms
- **Lidar scan rate**: 5-10 Hz
- **Navigation planning**: < 2s for 10m path

### Diagnostic Commands

```bash
# Monitor system performance
rostopic hz /wheel_odom /scan /odom

# Check for dropped messages
rostopic echo /rosout | grep "dropped"

# Monitor CPU usage
top -p $(pgrep -f "ros")

# Check network connectivity
ping 192.168.1.16
```

## Troubleshooting Tests

### Motor Issues
```bash
# Test PWM directly (ESP32 Serial Monitor)
# Send commands and observe motor response

# Check encoder counts
rostopic echo /wheel_odom | grep "position"
```

### Navigation Issues
```bash
# Test localization
roslaunch elderly_bot navigation.launch
# Use RViz "2D Pose Estimate" to set pose

# Check costmaps
rostopic echo /move_base/global_costmap/costmap | head -20
```

### Communication Issues
```bash
# Test rosserial stability
timeout 30 rosrun rosserial_python socket_node.py _host:=192.168.1.16 _port:=11411

# Check for reconnection
rostopic list | grep -c "wheel_odom"
```

## Integration Testing

### Full System Test
1. Power on all components
2. Launch bringup
3. Verify all topics are publishing
4. Test teleoperation
5. Test autonomous navigation (if map available)

### Stress Testing
1. High-frequency command sending
2. Long-duration operation (>1 hour)
3. Temperature stress (if applicable)
4. Battery voltage variations

## Data Validation

### Odometry Validation
- Check for realistic velocities (< 0.5 m/s)
- Verify heading consistency
- Compare with known distances

### Sensor Validation
- Lidar: Check for 360° coverage
- IMU: Verify gravity vector (9.81 m/s² downward)
- Encoders: Check for monotonic counting

## Reporting Issues

When reporting test failures, include:
1. **Test environment** (hardware versions, ROS distro)
2. **Commands executed**
3. **Expected vs actual behavior**
4. **Log output** (`roscd elderly_bot && cat ~/.ros/log/latest/*.log`)
5. **Topic data samples**

---
*This testing guide ensures systematic validation of all Elderly Robot components.*
