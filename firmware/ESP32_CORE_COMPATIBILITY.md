# ESP32 Arduino Core Compatibility Guide

## Problem Summary

The elderly_bot_esp32.ino firmware experiences compatibility issues when upgrading from ESP32 Arduino Core 2.x to 3.x:

### Issues Encountered:

1. **Compilation Errors**:
   - `'ledcSetup' was not declared in this scope`
   - `'ledcAttachPin' was not declared in this scope`
   - `'ledcWrite' was not declared in this scope`

2. **Runtime Crashes**:
   - `assert failed: tcpip_send_msg_wait_sem (Invalid mbox)`
   - Continuous reboot loop
   - LWIP (Lightweight IP) stack crashes

3. **Serial Output Issues**:
   - Unreadable boot messages
   - Application logs not appearing
   - Repeated boot sequences

## Root Cause Analysis

### 1. LEDC API Changes (Core 3.x)

**Core 2.x**: LEDC functions were available as simple C-style functions:
```cpp
ledcSetup(channel, freq, resolution);
ledcAttachPin(pin, channel);
ledcWrite(channel, duty);
```

**Core 3.x**: LEDC API moved to low-level driver interface requiring explicit configuration:
```cpp
#include "driver/ledc.h"
// Must use ledc_timer_config_t and ledc_channel_config_t structs
```

### 2. TCP/IP Stack Initialization Order (Core 3.x)

**Core 2.x**: Network stack initialized lazily, rosserial could start immediately

**Core 3.x**: Network stack requires explicit initialization before any socket operations. When rosserial_arduino attempts to initialize serial communication, it triggers LWIP operations before the TCP/IP adapter is ready, causing:
```
assert failed: tcpip_send_msg_wait_sem IDF/components/lwip/lwip/src/api/tcpip.c:455 (Invalid mbox)
```

### 3. Serial Timing

**Core 3.x** requires longer delays after `Serial.begin()` for proper initialization.

## Solution

### RECOMMENDED: Use ESP32 Arduino Core 2.0.17

**Why?**
- Last stable release of 2.x branch
- Proven compatibility with rosserial
- No code changes required
- Stable LWIP implementation
- Well-tested with ROS applications

**Installation**:

#### Arduino IDE:
1. Open Arduino IDE
2. Go to **Tools → Board → Boards Manager**
3. Search for "esp32"
4. Click on "esp32 by Espressif Systems"
5. Select version **2.0.17** from dropdown
6. Click **Install**

#### arduino-cli:
```bash
arduino-cli core install esp32:esp32@2.0.17
```

#### platformio.ini (if using PlatformIO):
```ini
[env:esp32dev]
platform = espressif32@5.4.0  ; Maps to Arduino Core 2.0.17
board = esp32dev
framework = arduino
```

### Board Configuration:
```
Board: ESP32 Dev Module
Upload Speed: 115200
CPU Frequency: 240MHz
Flash Frequency: 80MHz
Flash Mode: QIO
Flash Size: 4MB (32Mb)
Partition Scheme: Default 4MB with spiffs
Core Debug Level: None
Arduino Core: 2.0.17
```

### Alternative: Core 3.x Compatibility Layer

If you **must** use Core 3.x (not recommended for ROS applications), the firmware now includes a compatibility layer that:

1. **Detects Core Version**:
```cpp
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  #define ESP32_CORE_3X
```

2. **Provides LEDC Wrapper Functions**:
```cpp
bool ledcSetup(uint8_t channel, double freq, uint8_t resolution_bits);
bool ledcAttachPin(uint8_t pin, uint8_t channel);
void ledcWrite(uint8_t channel, uint32_t duty);
```

3. **Fixes Initialization Order**:
```cpp
void setup() {
  Serial.begin(115200);
  delay(500);  // CRITICAL: Wait for TCP/IP stack
  
  setupMotors();    // Hardware first
  setupEncoders();
  setupIMU();
  
  delay(100);       // Additional delay
  
  nh.initNode();    // ROS last
  // ...
}
```

## Technical Details

### LEDC API Changes Explained

**Core 2.x Implementation**:
- Simple wrapper functions in `esp32-hal-ledc.c`
- Automatic timer and channel management
- User-friendly API

**Core 3.x Implementation**:
- Direct access to ESP-IDF driver layer
- Manual configuration of timers and channels
- More control but requires explicit setup

**Compatibility Layer Mapping**:
```cpp
ledcSetup(channel, freq, resolution)
  ↓
ledc_timer_config_t + ledc_timer_config()

ledcAttachPin(pin, channel)
  ↓
ledc_channel_config_t + ledc_channel_config()

ledcWrite(channel, duty)
  ↓
ledc_set_duty() + ledc_update_duty()
```

### TCP/IP Crash Explanation

**The Problem**:
```
rosserial_arduino → Serial operations → LWIP socket calls → TCP/IP not ready → CRASH
```

**The Fix**:
```cpp
// Initialize hardware BEFORE ROS
setupMotors();
setupEncoders();
setupIMU();

delay(100);  // Let TCP/IP stack initialize

// NOW safe to initialize ROS
nh.initNode();
```

**Why This Works**:
- Core 3.x initializes TCP/IP stack asynchronously
- The delays allow FreeRTOS tasks to complete initialization
- Hardware setup first ensures no premature network calls
- By the time `nh.initNode()` is called, LWIP is ready

## Verification

### Test Core 2.0.17 Installation:
```bash
# In Arduino IDE, open Tools → Board → Board Manager
# Verify "esp32 by Espressif Systems" shows version 2.0.17

# Or check via arduino-cli:
arduino-cli core list
# Should show: esp32:esp32  2.0.17
```

### Test Compilation:
```bash
# Compile firmware
arduino-cli compile --fqbn esp32:esp32:esp32 elderly_bot_esp32.ino

# Should complete without errors
```

### Test Upload and Run:
```bash
# Upload firmware
arduino-cli upload -p /dev/ttyUSB1 --fqbn esp32:esp32:esp32 elderly_bot_esp32.ino

# Monitor serial output
arduino-cli monitor -p /dev/ttyUSB1 -c baudrate=115200

# Expected output:
# ...
# Elderly Bot ESP32 Firmware Starting...
# Calibrating IMU - keep robot still...
# IMU calibration complete!
# Elderly Bot Ready!
```

### Test with ROS:
```bash
# Terminal 1: Start roscore
roscore

# Terminal 2: Connect rosserial
rosrun rosserial_python serial_node.py /dev/ttyUSB1

# Terminal 3: Check topics
rostopic list
# Should see:
# /wheel_odom
# /imu/data

rostopic hz /wheel_odom
# Should show ~100 Hz

rostopic hz /imu/data
# Should show ~100 Hz
```

## Troubleshooting

### Still Getting LEDC Errors?
- Verify you're using Core 2.0.17 (not 3.x)
- Clean build: Delete build cache and recompile
- Restart Arduino IDE

### Still Getting TCP/IP Crashes?
- Increase delays in setup():
  ```cpp
  delay(1000);  // After Serial.begin()
  delay(500);   // Before nh.initNode()
  ```
- Check serial baud rate matches (115200)
- Verify USB cable and connection

### Serial Output Garbled?
- Check baud rate: 115200
- Try different USB cable
- Check USB-to-serial chip drivers

### ROS Not Connecting?
```bash
# Check ESP32 is on correct port
ls -l /dev/ttyUSB*

# Test serial connection
screen /dev/ttyUSB1 115200
# Should see periodic output

# Check rosserial
rosrun rosserial_python serial_node.py /dev/ttyUSB1 _baud:=115200
```

## Version Compatibility Matrix

| Component | Core 2.0.17 | Core 3.x |
|-----------|-------------|----------|
| LEDC API | ✅ Native | ⚠️ Compatibility layer |
| rosserial | ✅ Stable | ⚠️ Requires delays |
| LWIP Stack | ✅ Stable | ⚠️ Initialization issues |
| Wire (I2C) | ✅ Works | ✅ Works |
| Interrupts | ✅ Works | ✅ Works |
| Serial | ✅ Works | ⚠️ Needs longer delays |
| **Overall** | ✅ **RECOMMENDED** | ⚠️ Use only if necessary |

## Migration Path

### If Currently on Core 2.x:
**DO NOT UPGRADE** - Stay on 2.0.17 for maximum stability

### If Already on Core 3.x:
**Option 1 (Recommended)**: Downgrade to 2.0.17
```bash
# Arduino IDE: Boards Manager → esp32 → Select 2.0.17
# No code changes needed
```

**Option 2**: Use compatibility layer
- Use the updated firmware with compatibility layer
- Test thoroughly before deployment
- Monitor for stability issues

## References

- ESP32 Arduino Core 2.0.17: https://github.com/espressif/arduino-esp32/releases/tag/2.0.17
- ESP32 Arduino Core 3.x Changes: https://github.com/espressif/arduino-esp32/releases/tag/3.0.0
- LEDC Driver Documentation: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/ledc.html
- rosserial_arduino: http://wiki.ros.org/rosserial_arduino

## Summary

✅ **RECOMMENDED SOLUTION**: Use ESP32 Arduino Core 2.0.17
- No code changes required
- Proven stability with ROS
- No LWIP issues
- Simple installation

⚠️ **Alternative**: Use Core 3.x with compatibility layer
- Requires firmware with compatibility layer (provided)
- May have subtle timing issues
- More complex debugging
- Only use if Core 3.x features are required

**Bottom Line**: For ROS robotics applications, **ESP32 Arduino Core 2.0.17 is the stable, tested choice**.

