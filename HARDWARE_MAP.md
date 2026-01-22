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

- **Wheel Radius**: `0.0325 meters` (32.5mm, from 65mm diameter wheels)
- **Track Width**: `0.26 meters`
- **Encoder Ticks per Revolution**: `3960`

## Motor Specifications (JGB37-520)

- **Model**: JGB37-520
- **Voltage**: 12V DC
- **No-Load Speed**: 110 RPM
- **Gear Ratio**: 90:1
- **Encoder**: Hall effect quadrature encoder
  - **PPR (Pulses Per Revolution)**: 11
  - **Edges per Revolution**: 4 (quadrature decoding)
  - **Total Ticks per Revolution**: 11 × 90 × 4 = **3960**
- **Calculation**: `ticks_to_m = (2.0 × π × 0.0325) / 3960.0 = 0.0000515 m/tick`

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

## Sensors/Analog Input (ESP32)

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

---

## MPU9250 IMU (Direct Jetson I2C Connection)

### Hardware Wiring

**Jetson Nano J21 Header:**
```
Pin 1:  3.3V        ← MPU9250 VCC
Pin 3:  I2C2_SDA    ← MPU9250 SDA
Pin 5:  I2C2_SCL    ← MPU9250 SCL
Pin 6:  GND         ← MPU9250 GND
```

### Physical Mounting
- **IMU Height**: 7cm from ground
- **Relative to base_link**: z = -0.0125m (1.25cm below base_link at 8.25cm height)
- **Axes**: MPU9250 native NED (X=forward, Y=right, Z=down) remapped to ROS ENU in software

### I2C Configuration
- **Bus**: I2C-1 (`/dev/i2c-1`)
- **Address**: `0x68` (AD0 pin LOW)
- **Verify connection**: `sudo i2cdetect -y 1` (should show device at 0x68)

### Software Setup

**Install I2C tools:**
```bash
sudo apt-get install -y i2c-tools python3-smbus
```

**Test connection:**
```bash
sudo i2cdetect -y 1
# MPU9250 should appear at address 0x68
```

**Enable node:**
```bash
cd ~/catkin_ws/src/elderly_bot/scripts
chmod +x mpu9250_node.py
```

### Topics Published
- `/imu/data_raw` - Raw accelerometer + gyroscope (sensor_msgs/Imu)
- `/imu/data` - Fused orientation from imu_filter_madgwick (sensor_msgs/Imu)

**Note**: Magnetometer DISABLED - poisoned by indoor EMI (motors, PSU, battery)

### Calibration
- **Gyroscope**: Auto-calibrates on startup (robot must be stationary for 1 second)
- **Magnetometer**: Hard-iron bias already configured in `imu_nav.launch`
- **No manual calibration needed** - dynamic gyro calibration adapts to temperature

### NED to ENU Remapping
MPU9250 outputs NED convention (Y=right, Z=down). Software remapping applied in `mpu9250_node.py`:
```python
# Transform: X_enu=X_ned, Y_enu=-Y_ned, Z_enu=-Z_ned
accel_y = -accel_y
accel_z = -accel_z
gyro_y = -gyro_y
gyro_z = -gyro_z
```

**Result**: URDF uses identity rotation (rpy="0 0 0"), axes match ROS ENU standard
---

## Sensors/Actuators on Jetson Nano GPIO

### MQ-6 Gas Sensor (GPIO Mode)

**Hardware Wiring (Jetson J41 Header):**
```
Pin 2:  5V          ← MQ-6 VCC
Pin 6:  GND         ← MQ-6 GND
Pin 18: GPIO 24     ← MQ-6 D0 (digital output)
```

**Configuration:**
- **Mode**: GPIO (binary detection, no ADC required)
- **Pin**: 18 (BOARD numbering = GPIO 24)
- **Detection**: Digital HIGH = gas detected, LOW = no gas
- **Sensitivity**: Adjustable via on-board potentiometer
- **Alternative Mode**: I2C via ADS1115 ADC for analog voltage readings (optional)
- **Wiring Guide**: [docs/GAS_SENSOR_WIRING.md](docs/GAS_SENSOR_WIRING.md)

**Topics Published:**
- `/gas_level` (std_msgs/Float32) - Voltage level (0.0 in GPIO mode)
- `/gas_detected` (std_msgs/Bool) - Gas detection status

**Detection Logic:**
```python
# GPIO mode: digital on/off only
pin_state = GPIO.input(18)  # Read Pin 18
detected = (pin_state == GPIO.HIGH)
```

### Active Buzzer (Transistor-Driven)

**Hardware Wiring (Jetson J41 Header):**
```
Pin 2:  5V          ← Buzzer (+) positive terminal
Pin 6:  GND         ← 2N2222 Emitter
Pin 12: GPIO 18     ← 1kΩ resistor → 2N2222 Base
                      Buzzer (-) → 2N2222 Collector
```

**Configuration:**
- **Pin**: 12 (BOARD numbering = GPIO 18)
- **Driver Circuit**: 2N2222 NPN transistor with 1kΩ base resistor
- **Power**: 5V, 30-50mA (switched by transistor, not GPIO direct)
- **Safety**: Auto-shutoff after 5 seconds if no commands received
- **Wiring Guide**: [docs/BUZZER_WIRING.md](docs/BUZZER_WIRING.md)

**Topic Subscribed:**
- `/buzzer_command` (std_msgs/Bool) - true = ON, false = OFF

**Control Circuit:**
```
GPIO HIGH (3.3V) → 1kΩ → Base → Transistor ON → Buzzer sounds
GPIO LOW (0V)    → No base current → Transistor OFF → Buzzer silent
```

**Integration:**
- Buzzer automatically activates when gas sensor detects gas
- Manual control available via `/buzzer_command` topic
- Thread-safe operation with mutex-protected GPIO access

### Jetson Stats Monitoring

**Software Monitoring:**
- **Library**: jetson-stats (jtop)
- **Metrics**: CPU temperature, GPU temperature, power consumption
- **Topics Published**:
  - `/jetson/temperature` (std_msgs/Float32) - CPU temp in °C
  - `/jetson/power` (std_msgs/Float32) - Power consumption in watts

**Safety Thresholds:**
- Temperature warning: >70°C
- Power monitoring: Real-time system load tracking