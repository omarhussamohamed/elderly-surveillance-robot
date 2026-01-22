# MQ-6 Gas Sensor - Direct GPIO Connection (No ADC)

## Simple 3-Wire Connection

Your MQ-6 module should have a **D0** (digital output) pin. This gives ON/OFF detection without needing an ADC.

### What You Need
- MQ-6 Gas Sensor Module (with D0 pin)
- 3 jumper wires (female-to-female)

### Connections

**Power Off Jetson First!**

| MQ-6 Pin | → | Jetson Pin | Wire Color | Description |
|----------|---|------------|------------|-------------|
| VCC | → | Pin 2 (5V) | Red | Power |
| GND | → | Pin 6 (GND) | Black | Ground |
| D0 | → | Pin 18 (GPIO 24) | Yellow | Digital signal |

### Pin Conflict Check
✅ **No conflicts!**
- Pin 18 (GPIO 24): Free, not used by anything else
- Pin 2 (5V): Shared with other 5V devices - okay, Jetson can supply ~2A
- Pin 6 (GND): Common ground - okay

### Wiring Diagram
```
MQ-6 Module          Jetson Nano J41 Header
┌─────────┐          ┌────────────────┐
│ VCC  ●──┼── Red ──→│ Pin 2 (5V)     │
│ GND  ●──┼─ Black ─→│ Pin 6 (GND)    │
│ D0   ●──┼─Yellow ─→│ Pin 18 (GPIO24)│
│ A0   ○  │          └────────────────┘
└─────────┘
```

### How It Works
- **D0 pin**: Goes HIGH when gas detected, LOW when no gas
- **Threshold**: Set by turning the small potentiometer on the MQ-6 module
- **No voltage readings**: Only binary on/off detection

---

## Configuration

Edit these files:

**File**: `config/sensors_actuators.yaml`

```yaml
# Gas Sensor Configuration
enable_gas_sensor: true
gas_sensor_mode: "gpio"        # Use GPIO mode (not I2C)
gas_sensor_gpio_pin: 18        # GPIO 24 = Pin 18 (BOARD numbering)
publish_rate: 1.0              # 1 Hz update rate

# These are not used in GPIO mode:
# adc_i2c_address: 0x48
# adc_i2c_bus: 1
# gas_threshold_voltage: 1.0
```

---

## Testing

```bash
# 1. Launch the node
roslaunch elderly_bot bringup.launch enable_sensors:=true

# 2. Monitor detection status
rostopic echo /gas_detected
# Will show: True (gas detected) or False (no gas)

# Note: /gas_level will always show 0.0 in GPIO mode
```

### Adjusting Sensitivity

Turn the small potentiometer (screw) on the MQ-6 module:
- **Clockwise**: Less sensitive (only strong gas triggers)
- **Counter-clockwise**: More sensitive (weak gas triggers)

Test by releasing gas from a lighter near the sensor and adjust until it reliably detects.

---

## Limitations of GPIO Mode

❌ **No voltage readings** - can't see analog gas levels  
❌ **Fixed threshold** - adjusted manually with potentiometer, not via software  
❌ **Binary only** - just on/off, no gradual detection  
✅ **No extra hardware needed** - works with what you have  
✅ **Simple and reliable** - fewer points of failure  

---

## If You Want Voltage Readings Later

You can buy the ADS1115 module later (~$5-8) to get:
- Full analog voltage readings (0-5V)
- Software-adjustable threshold
- See gradual gas level changes
- Better for calibration

Then just switch `gas_sensor_mode: "i2c"` in the config and connect A0 pin instead of D0.

---

**Last Updated**: January 22, 2026
