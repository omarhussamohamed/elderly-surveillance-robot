# GPIO Transport Alternative Evaluation

## Overview

Since USB rosserial has proven unreliable, this document evaluates GPIO-based communication alternatives for ESP32-to-Jetson communication.

## Communication Options

### Option 1: UART over GPIO (Recommended)

**Hardware:**
- ESP32: Hardware UART pins (GPIO 16/17, or any free GPIO)
- Jetson: Available UART pins (e.g., J21 header pins)

**Advantages:**
- ✅ Lower latency than USB (direct hardware)
- ✅ More reliable (no USB drivers/hubs)
- ✅ Still works with rosserial (rosserial_arduino supports UART)
- ✅ Easy to implement
- ✅ Good throughput (up to 1Mbps)

**Disadvantages:**
- ❌ Requires physical wiring (GPIO to GPIO)
- ❌ Limited to ~3-5 meters without level shifting
- ❌ Need 3.3V level shifter if Jetson uses 1.8V GPIO
- ❌ One less USB port available (but you keep USB0/USB1)

**Implementation:**
```cpp
// ESP32 side - use HardwareSerial
#include <HardwareSerial.h>
HardwareSerial Serial1(1); // UART1

void setup() {
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  nh.getHardware()->setPort(&Serial1);
}
```

**Jetson side:**
```bash
# Enable UART on Jetson (check device tree)
# Connect to /dev/ttyTHS1 or similar
rosrun rosserial_python serial_node.py _port:=/dev/ttyTHS1 _baud:=115200
```

**Wiring:**
- ESP32 TX → Jetson RX (with level shifter if needed)
- ESP32 RX → Jetson TX (with level shifter if needed)
- GND → GND (shared ground essential!)

---

### Option 2: SPI over GPIO

**Hardware:**
- ESP32: SPI pins (MOSI, MISO, SCK, CS)
- Jetson: SPI pins (SPI1, SPI2, etc.)

**Advantages:**
- ✅ Highest throughput (multiple Mbps)
- ✅ Full-duplex communication
- ✅ Can multiplex multiple devices

**Disadvantages:**
- ❌ Requires custom protocol (rosserial doesn't support SPI directly)
- ❌ More complex implementation
- ❌ Need to write custom ROS node on Jetson side
- ❌ More wiring (4+ wires)

**Implementation Complexity:** HIGH
- Would need custom ROS node using `spidev` on Jetson
- Custom packet protocol on ESP32
- Not compatible with rosserial

**Recommendation:** Only if UART fails and high throughput is critical

---

### Option 3: I2C over GPIO

**Hardware:**
- ESP32: I2C pins (SDA, SCL) - BUT THESE ARE USED FOR IMU!
- Jetson: I2C pins

**Advantages:**
- ✅ Only 2 wires (SDA, SCL)
- ✅ Can share bus with IMU (if addressable)

**Disadvantages:**
- ❌ Lower throughput than UART (~400kHz)
- ❌ Not suitable for high-rate data
- ❌ Requires custom protocol (no rosserial support)
- ❌ Conflicts with IMU I2C if same bus
- ❌ Limited message size

**Recommendation:** NOT RECOMMENDED - Too slow for odometry + IMU data

---

## Recommended Solution: UART over GPIO

### Why UART?

1. **Compatibility**: Works with existing rosserial_arduino
2. **Reliability**: Direct hardware connection, no USB stack issues
3. **Performance**: 115200-921600 baud sufficient for robot data
4. **Simplicity**: Minimal code changes needed
5. **Proven**: Many ROS robots use UART for microcontroller communication

### Required Hardware

**Level Shifter (if Jetson GPIO is 1.8V):**
- TXB0104 or similar bidirectional level shifter
- Or 2x 10kΩ resistors for simple voltage divider (not recommended for production)

**Wiring:**
```
ESP32          Jetson
GPIO 17 (TX) → UART RX (via level shifter)
GPIO 16 (RX) → UART TX (via level shifter)
GND          → GND
```

### Jetson GPIO Configuration

**Jetson Nano J21 Header:**
- Pin 8: UART2 TX (3.3V)
- Pin 10: UART2 RX (3.3V)
- Pin 14: GND

**Enable UART:**
```bash
# Check if UART is enabled
ls /dev/ttyTHS*

# If not visible, may need to enable in device tree
# Edit /boot/extlinux/extlinux.conf or use jetson-io tool
```

### Code Changes Required

**ESP32 Firmware (minimal changes):**
```cpp
// Use HardwareSerial instead of Serial
#include <HardwareSerial.h>
HardwareSerial uart_serial(1);

void setup() {
  // Initialize UART
  uart_serial.begin(115200, SERIAL_8N1, 16, 17); // RX=16, TX=17
  
  // Point rosserial to UART instead of Serial
  nh.getHardware()->setPort(&uart_serial);
  nh.getHardware()->setBaud(115200);
  
  // Keep Serial for debugging (USB)
  Serial.begin(115200);
  // ... rest of setup
}
```

**Jetson Launch File:**
```xml
<node name="esp32_serial_node" 
      pkg="rosserial_python" 
      type="serial_node.py">
  <param name="port" value="/dev/ttyTHS2" />  <!-- UART2 -->
  <param name="baud" value="115200" />
</node>
```

---

## Performance Comparison

| Method | Latency | Throughput | Reliability | Complexity |
|--------|---------|------------|-------------|------------|
| USB (Current) | 5-20ms | High | ❌ Unreliable | Low |
| UART GPIO | 1-5ms | Medium | ✅ Excellent | Low |
| SPI GPIO | <1ms | Very High | ✅ Excellent | High |
| I2C GPIO | 2-10ms | Low | ⚠️ Medium | Medium |

---

## Migration Path

### Phase 1: Fix USB (Current)
1. ✅ Regenerate ros_lib on Jetson
2. ✅ Fix firmware with FreeRTOS
3. ✅ Test USB connection

### Phase 2: If USB Still Fails, Move to UART
1. Wire ESP32 GPIO to Jetson UART
2. Modify firmware to use HardwareSerial
3. Update launch file port
4. Test and verify

### Phase 3: Optimization (If Needed)
1. Increase baud rate (230400, 460800, 921600)
2. Optimize message sizes
3. Add error correction if needed

---

## Decision Matrix

**Use UART over GPIO if:**
- ✅ USB continues to fail after fixing ros_lib
- ✅ You can wire GPIO connections
- ✅ You need reliable communication
- ✅ Latency < 5ms is acceptable

**Stick with USB if:**
- ✅ USB works after ros_lib fix
- ✅ You prefer plug-and-play
- ✅ Cable length > 3 meters needed

**Use SPI if:**
- ✅ UART bandwidth insufficient
- ✅ You need < 1ms latency
- ✅ You can implement custom protocol

---

## Next Steps

1. **First**: Try USB fix (regenerate ros_lib on Jetson)
2. **If USB fails**: Implement UART over GPIO
3. **If UART insufficient**: Consider SPI (custom implementation)

Would you like me to implement the UART GPIO solution now, or wait to see if the USB fix works?

