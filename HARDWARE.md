# Hardware — Current Reality

**Last Updated**: January 23, 2026

This is the **only authoritative source** for hardware configuration. Do not trust other documents.

---

## Computing

- **Main Controller**: NVIDIA Jetson Nano (4GB)
  - OS: Ubuntu 18.04 LTS
  - ROS: Melodic
  - JetPack: 4.6.x
  
- **Motor Controller**: ESP32 38-pin DevKit
  - Firmware: Arduino rosserial
  - Connection: WiFi (SSID configured in firmware)
  - Protocol: rosserial over TCP

---

## Sensors

### LiDAR
- **Model**: RPLidar A1
- **Interface**: USB Serial (/dev/ttyUSB0, 115200 baud)
- **Driver**: rplidar_ros package
- **Frame ID**: `laser`
- **Mount**: Rear of robot, 180° rotated (backward-facing)
- **Height**: 0.30m above ground
- **Scan Rate**: 8 Hz, 360°

### IMU
- **Model**: MPU-9250 (9-axis, using only 6-axis)
- **Interface**: I2C bus 1, address 0x68
- **Connections**:
  - SDA → Jetson Pin 3 (I2C1 SDA)
  - SCL → Jetson Pin 5 (I2C1 SCL)
  - VCC → 3.3V (Pin 1)
  - GND → Ground (Pin 6, 9, 14, or 20)
- **Driver Node**: `mpu9250_node`
- **Frame ID**: `imu_link`
- **Sensors Used**: Gyroscope + Accelerometer only
- **Magnetometer**: **DISABLED** (indoor EMI from motors/PSU/battery)
- **Calibration**: Dynamic gyro zero-drift calibration on startup (robot must be stationary)
- **Fusion**: imu_filter_madgwick provides orientation

### Gas Sensor
- **Model**: MQ-6 (LPG/natural gas)
- **Interface**: GPIO digital input (default mode)
- **Connections**:
  - D0 → Jetson Pin 18 (GPIO)
  - VCC → 3.3V
  - GND → Ground
  - External pull-up: 2.2-4.7kΩ resistor to 3.3V
- **Polarity**: Active-low (LM393 comparator pulls low when gas detected)
- **Driver Node**: `sensors_actuators_node`
- **Debouncing**: 500ms software debounce
- **Alternative Mode**: A0 analog via ADS1115 I2C ADC (not currently used)

### Jetson System Monitor
- **Interface**: jtop library (jetson-stats package)
- **Metrics**: Temperature (°C), Power consumption (W)
- **Driver Node**: `sensors_actuators_node`

---

## Actuators

### Motors
- **Model**: JGB37-520
- **Specs**: 12V DC, 110 RPM, 90:1 gearbox
- **Quantity**: 4 (4WD skid-steer)
- **Drivers**: 2× L298N dual H-bridge
- **Control**: PWM from ESP32 (motor_control firmware)

### Encoders
- **Type**: Hall effect quadrature
- **Resolution**: 11 PPR (pulses per revolution)
- **After Gearbox**: 11 × 90 = 990 pulses/rev
- **Counting Edges**: 4× (A rising/falling, B rising/falling)
- **Effective Resolution**: 990 × 4 = **3960 ticks/revolution**
- **Debouncing**: 100µs software filter in ESP32 firmware
- **Interface**: GPIO interrupts on ESP32
- **Published**: Via /odom topic (nav_msgs/Odometry)

### Buzzer
- **Type**: Active buzzer (5V)
- **Interface**: Jetson GPIO Pin 16
- **Driver**: 2N2222 NPN transistor
- **Control Circuit**:
  - Jetson Pin 16 → 1kΩ resistor → 2N2222 base
  - 5V rail → Buzzer → 2N2222 collector
  - 2N2222 emitter → Ground
- **Driver Node**: `sensors_actuators_node`
- **Pattern**: Continuous beeping (0.1s ON/OFF) when activated
- **Trigger**: Automatic on gas detection, or manual via `/buzzer_command`

---

## Power

- **Battery**: 12V LiPo or lead-acid (capacity varies)
- **Voltage Rail**: 12V (motors), 5V (Jetson + ESP32 via buck converters)
- **Monitoring**: Jetson power consumption via jtop library
- **Battery Voltage Monitoring**: Not currently implemented

---

## Kinematics

- **Robot Type**: 4WD skid-steer (differential drive)
- **Track Width**: 0.26m (center-to-center of left/right wheels)
- **Wheelbase**: 0.17m (front-to-rear axle distance)
- **Wheel Diameter**: 0.065m (65mm)
- **Effective Wheel Radius**: 0.0325m (calibrated for 1:1 odometry mapping)
- **Distance per Encoder Tick**: 0.0000515m

---

## Wiring Summary

**Critical I2C Devices (Bus 1)**:
- MPU-9250 IMU @ 0x68
- (Optional) ADS1115 ADC @ 0x48 (if using I2C gas sensor mode)

**USB Devices**:
- RPLidar A1 → /dev/ttyUSB0

**GPIO (Jetson Nano BOARD Pin Numbering)**:
- Pin 3: I2C1 SDA (MPU-9250)
- Pin 5: I2C1 SCL (MPU-9250)
- Pin 16: Buzzer control (via transistor)
- Pin 18: MQ-6 gas sensor D0 (digital input, active-low)

**Network**:
- ESP32 WiFi: Connected to same LAN as Jetson
- ROS Master: Running on Jetson (http://jetson-hostname:11311)

---

## Removed / Deprecated Hardware

**Magnetometer (MPU-9250)**:
- **Reason**: Indoor electromagnetic interference renders data unusable
- **Date Removed**: Early 2026 (configuration change, hardware still present)
- **Replacement**: Madgwick fusion uses only gyro + accel for orientation

**External Battery Monitor**:
- **Status**: Planned but not implemented
- **Current Workaround**: None (manual battery voltage check with multimeter)

---

## Physical Dimensions

- **Length**: 35cm
- **Width**: 25cm
- **Height**: 14cm (excluding LiDAR, ~20cm with LiDAR)
- **Weight**: ~2.5kg (approximate, varies with battery)

---

## Maintenance Notes

- IMU requires stationary robot during startup for gyro calibration
- Gas sensor requires 24-48 hour burn-in period for stable readings
- Encoder debouncing critical for noise-free odometry
- WiFi connection between ESP32 and Jetson must be stable for motor control
