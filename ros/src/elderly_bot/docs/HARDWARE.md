# Hardware Specification — Elderly Bot (Authoritative)

This document describes the **actual physical hardware**, **pin mappings**, and
**electrical connections** used in the Elderly Bot system.

Only hardware that is **verified by code or explicit confirmation** is listed.
No planned or hypothetical components are included.

---

## Compute & Control

### NVIDIA Jetson
- Role: High-level processing
- OS: Ubuntu 18.04
- ROS: Melodic
- Responsibilities:
  - SLAM (GMapping)
  - Sensor processing
  - EKF localization
  - Cloud communication (optional)
  - GPIO access (gas sensor & buzzer)

---

### ESP32
- Role: Low-level motion interface
- Communication with ROS: **WiFi (rosserial over TCP)**
- Responsibilities:
  - Wheel encoder reading
  - Encoder tick publishing
- Does **NOT**:
  - Publish TF
  - Compute robot pose
  - Control motors directly from ROS navigation

---

## Locomotion System

### Drive Type
- Differential / skid-steer base
- Motors are driven **outside ROS navigation**
- ROS does **not** issue velocity commands during mapping

### Wheel Encoders (Connected to ESP32)

| Encoder | ESP32 Pin | Signal |
|------|-----------|--------|
Left wheel encoder | GPIO 34 | Encoder pulses |
Right wheel encoder | GPIO 35 | Encoder pulses |

Notes:
- Encoders are read via hardware interrupts
- Encoder ticks are published to ROS
- Odometry computation is handled on the ROS side

---

## Sensors

### LiDAR
- Model: **RPLidar A1**
- Interface: USB
- ROS topic:
  - `/scan` (`sensor_msgs/LaserScan`)
- Frame:
  - `laser` (static TF from URDF)

---

### IMU
- Model: **MPU9250**
- Interface: I2C
- ROS topic:
  - `/imu/data_raw`
- Frame:
  - `imu_link`
- Notes:
  - Raw IMU only (no onboard fusion)
  - Orientation estimation handled by EKF

---

### Gas Sensor
- Type: **Digital gas sensor**
- Output used: **D0 (digital HIGH/LOW)**
- Interface: Jetson GPIO
- ROS topic:
  - `/gas_detected` (`std_msgs/Bool`)

#### GPIO Mapping (Jetson – BOARD numbering)

| Function | Pin |
|-------|-----|
Gas sensor D0 | 18 |

Notes:
- No ADC used
- No analog thresholding
- LOW = gas detected (active low logic)

---

### Camera
- Type: USB or CSI camera
- Interface: Direct to Jetson
- ROS topic:
  - `/camera/image_raw`
- Frame:
  - `camera_link`
- Used for:
  - Monitoring
  - Cloud streaming (optional)

---

## Actuators

### Buzzer
- Type: Active buzzer
- Interface: Jetson GPIO
- ROS control:
  - `/buzzer_command` (`std_msgs/Bool`)

#### GPIO Mapping (Jetson – BOARD numbering)

| Function | Pin |
|-------|-----|
Buzzer control | 16 |

Activation conditions:
- Gas detected locally
- OR cloud command (if cloud enabled)

---

## Power & Wiring Summary

- ESP32:
  - Powered independently
  - Handles encoder inputs only
- Jetson:
  - Handles sensors, logic, and GPIO
- Motors:
  - Controlled by motor driver outside ROS navigation stack

---

## TF & Frame Consistency

- URDF defines **static frames only**
- No `map` or `odom` frames in URDF
- EKF publishes `odom → base_link`
- GMapping publishes `map → odom`
- ESP32 publishes **no TF**

---

## Verified Hardware Checklist

- RPLidar A1 ✔
- Digital gas sensor (D0) ✔
- MPU9250 IMU ✔
- ESP32 encoder interface ✔
- Buzzer via GPIO ✔
- Camera connected to Jetson ✔
- Foxglove used for visualization ✔

---

## Notes

- All axes and sensor orientations are hardware-validated
- Software does not modify physical orientation assumptions
- Hardware configuration is stable and mapping-validated
