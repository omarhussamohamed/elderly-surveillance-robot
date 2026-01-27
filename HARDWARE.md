# Hardware — Current Reality

**Last Updated**: January 27, 2026

This is the **only authoritative source** for hardware configuration. Do not trust other documents.

**Primary Visualization**: Foxglove Bridge (ws://<jetson_ip>:8765) — displays /map, /scan, /tf, /odom, /gas_detected in real time.

---

## Computing

- **Main Controller**: NVIDIA Jetson Nano (4GB)
  - OS: Ubuntu 18.04 LTS
  - ROS: Melodic
  - JetPack: 4.6.x
  - Monitoring: jetson-stats (pip3 install jetson-stats) for temperature, power, CPU/GPU usage

- **Motor Controller**: NodeMCU ESP32s
  - Firmware: elderly_bot_esp32_wifi.ino (Arduino IDE, WiFiManager library + Preferences.h for persistent config)
  - Connection: WiFi (2.4 GHz only recommended; configured via captive portal AP "ElderlyBotESP32")
  - Protocol: rosserial over TCP (port 11411)
  - Core usage: Core 0 → ROS communication & odometry publishing (20 Hz); Core 1 → Motor control loop (100 Hz)
  - Encoder debouncing: 100 μs (prevents noise from magnetic encoders)

---

## Sensors & Actuators

### LiDAR
- Model: RPLidar A1
- Interface: USB Serial (/dev/ttyUSB0, 115200 baud)
- Driver: rplidar_ros package
- Frame ID: laser
- Mount: Rear of robot, physically rotated 180° (backward-facing to improve mapping field of view)
- Power: 5V from Jetson USB (~500 mA)
- udev rule: SYMLINK+="rplidar" (installed via install_dependencies.sh)

### IMU
- Model: MPU-9250 (9-axis: accel + gyro + magnetometer)
- Interface: I2C bus 1, address 0x68
- Driver: mpu9250_node.py (uses smbus2 library)
- Frame ID: imu_link
- Mount: Centered on chassis for accurate yaw estimation
- Fusion: Madgwick filter (imu_filter_madgwick package) → /imu/data
- Power: 3.3V from Jetson I2C pins (SCL = pin 3, SDA = pin 5, GND)
- Calibration note: Robot must be stationary during startup for gyro offset calibration

### Gas Sensor
- Model: MQ-6 (sensitive to LPG, propane, butane)
- Interface: Digital GPIO mode (D0 output → Jetson BOARD pin 18 / BCM 24)
- Driver: sensors_actuators_node.py (uses Jetson.GPIO library)
- Threshold: Digital high (1) on gas detection
- Mount: Front of robot for fast detection in living spaces
- Power: 5V VCC, GND; 3.3V pull-up resistor (2.2–4.7 kΩ) on D0
- Calibration: Adjust potentiometer clockwise if stuck low in clean air; 24–48 hour burn-in recommended
- Alert: Auto-triggers buzzer on detection; publishes /gas_detected (Bool)

### Buzzer
- Model: Active buzzer module (5V)
- Interface: GPIO pin 16 (BOARD numbering / BCM 23), controlled via 2N2222 transistor + 1 kΩ base resistor
- Driver: sensors_actuators_node.py
- Behavior: Auto-activates on gas detection; 5-second auto-shutoff for safety
- Power: 5V VCC, GND (transistor protects Jetson GPIO)

### Camera
- Model: USB webcam (e.g., Logitech C270 or similar)
- Interface: /dev/video0
- Driver: camera_node.py (uses cv_bridge + OpenCV)
- Resolution & FPS: 1280×720 @ 15 fps (production setting for stability)
- Streaming: kvs_streamer_node.py → AWS KVS (hardware encoding with nvh264enc)
- Mount: Front, elevated 25 cm above base_link (URDF origin xyz="0.15 0 0.25")

### Encoders & Motors
- Motors: 4× DC geared motors with magnetic quadrature encoders (3960 ticks/rev)
- Wheel radius: 6.5 cm
- Track width: 26 cm
- ESP32 Pinout (BOARD numbering):
  - Front Left: PWM=13, IN1=12, IN2=14, ENC_A=34, ENC_B=35
  - Front Right: PWM=27, IN1=26, IN2=25, ENC_A=36, ENC_B=39
  - Rear Left: PWM=21, IN1=32, IN2=15, ENC_A=18, ENC_B=19 (internal pullups)
  - Rear Right: PWM=22, IN1=16, IN2=17, ENC_A=23, ENC_B=5 (internal pullups)
- Odometry: Published as /odom at 20 Hz with slip-compensated covariances

### Battery & Power
- Type: 7.4V LiPo (2S)
- Regulators: Step-down to 5V/3.3V for Jetson, sensors, ESP32
- Monitoring: Planned via ADC (future); current workaround — manual multimeter check
- Safety: Avoid deep discharge

---

## Physical Dimensions
- Length: 35 cm
- Width: 25 cm
- Height: 14 cm (base chassis) + 6 cm (LiDAR) ≈ 20 cm total
- Weight: ≈ 2.5 kg (including battery)

---

## Maintenance & Calibration Notes
- IMU: Keep robot completely stationary during startup for gyro calibration
- Gas sensor: 24–48 hour burn-in; adjust potentiometer clockwise if digital output stuck low in clean air
- Encoders: 100 μs debounce; verify /odom for smooth motion (check for slip on carpet)
- WiFi: Stable 2.4 GHz connection required for real-time motor control
- Firmware: Update ESP32 via Arduino IDE (requires rosserial_arduino library)
- Power: Use 7.4V 2S LiPo with at least 2200 mAh; monitor Jetson temperature (<80°C)
- RPLidar: Clean lens regularly; ensure udev rule is active