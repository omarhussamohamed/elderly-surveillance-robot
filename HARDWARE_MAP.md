# Hardware Map - Source of Truth

This document contains the **exact hardware configuration** extracted from `elderly_bot_esp32_wifi.ino`, which is the current source of truth for the Elderly Robot Project.

## Power Supply

- **ESP32 Power Source**: Jetson Nano 5V rail
- **WARNING**: High WiFi transmission power or additional sensors on ESP32 must not exceed Jetson's 5V output capacity (~2A max)

## WiFi Configuration

- **SSID**: `"ShellBack"`
- **Password**: `"hhmo@1974"`
- **Server IP**: `192.168.1.16`
- **Server Port**: `11411`

## Motor Control Pins

### Front Left Motor (FL)
- **PWM Pin**: `13`
- **Direction Pin 1 (IN1)**: `12`
- **Direction Pin 2 (IN2)**: `14`
- **Encoder A**: `34`
- **Encoder B**: `35`

### Front Right Motor (FR)
- **PWM Pin**: `27`
- **Direction Pin 1 (IN1)**: `26`
- **Direction Pin 2 (IN2)**: `25`
- **Encoder A**: `36`
- **Encoder B**: `39`

### Rear Left Motor (RL)
- **PWM Pin**: `21`
- **Direction Pin 1 (IN1)**: `32`
- **Direction Pin 2 (IN2)**: `15`
- **Encoder A**: `18`
- **Encoder B**: `19`

### Rear Right Motor (RR)
- **PWM Pin**: `22`
- **Direction Pin 1 (IN1)**: `16`
- **Direction Pin 2 (IN2)**: `17`
- **Encoder A**: `23`
- **Encoder B**: `5`

## PWM Configuration

- **PWM Frequency**: `5000 Hz`
- **PWM Resolution**: `8-bit (0-255)`
- **PWM Channels**:
  - FL: `0`
  - FR: `1`
  - RL: `4`
  - RR: `5`
- **Minimum Movement PWM**: `152` (stall prevention threshold)

## Physical Constants

- **Wheel Radius**: `0.0325 meters`
- **Track Width**: `0.26 meters`
- **Encoder Ticks per Revolution**: `4900`

## Timing Constants

- **Control Loop Interval**: `20ms (50Hz)`
- **Odometry Publish Interval**: `100ms (10Hz)`
- **Safety Timeout**: `800ms`
- **ROS Serial Buffer Size**: `1024 bytes`

## Encoder Pin Modes

- **FL Encoder**: `INPUT` (external pullups assumed)
- **FR Encoder**: `INPUT` (external pullups assumed)
- **RL Encoder**: `INPUT_PULLUP` (internal pullups)
- **RR Encoder**: `INPUT_PULLUP` (internal pullups)

## Sensors/Analog Input

- **Differential Input Setup**:
  - **VP Pin (ADC1_CH0)**: `36`
  - **VN Pin (ADC1_CH3)**: `39`
  - **Resistor**: `1kΩ` between VP (Pin 36) and VN (Pin 39)

## ROS Configuration

- **Node Handle Buffer Sizes**: `25 subscribers, 25 publishers, 1024 input buffer, 1024 output buffer`
- **Topics**:
  - Subscriber: `cmd_vel` (geometry_msgs/Twist)
  - Publisher: `wheel_odom` (nav_msgs/Odometry)
- **Frame IDs**:
  - Odometry: `"odom"`
  - Base: `"base_footprint"`

## Firmware Architecture

- **Dual Core Design**:
  - **Core 0**: ROS/WiFi communication, odometry publishing
  - **Core 1**: Motor control task with independent left/right wheel control
- **Thread Safety**: Mutex-protected shared cmd_vel structure
- **Motor Control**: Implements differential drive kinematics (left = linear.x - angular.z, right = linear.x + angular.z)

## USB Port Assignments

- **RPLidar A1**: `/dev/ttyUSB0`
- **ESP32**: WiFi connection (192.168.1.16:11411) - no USB required

## Network Configuration

### Jetson Nano Firewall
**CRITICAL**: Port 11411 must be open for rosserial TCP communication

```bash
# Allow rosserial port (required for ESP32 WiFi connection)
sudo ufw allow 11411/tcp

# Verify firewall status
sudo ufw status
```

### WiFi Network Requirements
- **SSID**: ShellBack
- **ESP32 and Jetson must be on the same WiFi network**
- **Jetson IP Address**: 192.168.1.16 (static, matches ESP32 firmware)

## Dependencies

- **ESP32 Arduino Core**: Compatible with WiFi and FreeRTOS
- **rosserial**: For ROS communication over TCP
- **ROS Messages**: geometry_msgs/Twist, nav_msgs/Odometry

## Important Notes

- This configuration is **EXACTLY** from `motor_and_encoder_HW_test.ino` for motor behavior
- Encoder ISRs ensure forward motion = positive encoder counts
- PWM scaling prevents motor stalling with minimum threshold
- Safety timeout stops motors if no cmd_vel received within 800ms
