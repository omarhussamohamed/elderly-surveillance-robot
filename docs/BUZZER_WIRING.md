# Buzzer Wiring Guide (with Transistor)

## Overview
This guide shows how to connect an active buzzer to the Jetson Nano using a 2N2222 NPN transistor for switching. The transistor allows the low-power GPIO pin to control a higher-current buzzer safely.

---

## Components Needed
- **Active Buzzer** (5V, typically 30-50mA)
- **2N2222 NPN Transistor** (switching transistor)
- **1kΩ Resistor** (base current limiting)
- **Jumper wires** (3 wires minimum)
- Optional: Small breadboard for prototyping

---

## Circuit Diagram

```
Jetson Pin 12 (GPIO 18) ----[1kΩ]---- Base (2N2222)
                                        |
Jetson Pin 6 (GND) ----------------  Emitter
                                        |
                                    Collector
                                        |
Jetson Pin 2 (5V) ---- Buzzer (+) ---- Buzzer (-) ---- Collector
```

### 2N2222 Pinout (Bottom View)
```
   ___
  |   |
E B C     E = Emitter (GND)
          B = Base (GPIO via 1kΩ)
          C = Collector (Buzzer negative)
```

---

## Wiring Steps

### 1. Identify Components
- **2N2222**: Flat side facing you, pins down → Left=Emitter, Middle=Base, Right=Collector
- **Buzzer**: Usually marked with "+" on positive lead (longer leg is typically positive)
- **Resistor**: 1kΩ (color bands: Brown-Black-Red-Gold)

### 2. Connections

| From                | To                    | Purpose                |
|---------------------|-----------------------|------------------------|
| **Jetson Pin 2**    | Buzzer **positive (+)** | 5V power supply       |
| **Jetson Pin 6**    | 2N2222 **Emitter**    | Ground reference       |
| **Jetson Pin 12**   | 1kΩ → 2N2222 **Base** | GPIO control signal    |
| Buzzer **negative (-)** | 2N2222 **Collector** | Switched ground path  |

### 3. Pin Locations on Jetson J41 Header

```
    3.3V  [1] [2]  5V       ← Connect Buzzer (+) here
   I2C_SDA [3] [4]  5V
   I2C_SCL [5] [6]  GND     ← Connect Emitter here
           [7] [8]
       GND [9] [10]
          [11] [12] GPIO 18 ← Connect Resistor here (default buzzer pin)
          [13] [14] GND
          [15] [16]
     3.3V [17] [18] GPIO 24 (Gas sensor D0)
          [19] [20] GND
     ...
```

---

## How It Works

1. **GPIO LOW (0V)**: 
   - No current flows through base → transistor OFF
   - Collector-Emitter circuit open → buzzer OFF

2. **GPIO HIGH (3.3V)**:
   - Current flows: GPIO → 1kΩ → Base → Emitter → GND
   - Base current: ~2.3mA (safe for GPIO)
   - Transistor ON → Collector-Emitter conducts
   - Current flows: 5V → Buzzer → Collector → Emitter → GND
   - Buzzer ON (can draw 30-50mA, safe for Jetson 5V rail)

**Why transistor?**
- Jetson GPIO can only safely source ~10mA
- Buzzer typically needs 30-50mA
- Transistor acts as a switch controlled by low-power GPIO

---

## Configuration

### In `config/sensors_actuators.yaml`:
```yaml
sensors_actuators_node:
  enable_buzzer: true      # Set to true after wiring
  buzzer_pin: 12           # BOARD numbering (GPIO 18)
```

### Alternative Pins
Any available GPIO pin works. Common choices:
- **Pin 12** (GPIO 18) - Recommended, default in config
- Pin 11 (GPIO 17)
- Pin 13 (GPIO 27)
- Pin 15 (GPIO 22)
- Pin 16 (GPIO 23)

**Note**: Avoid pins already in use:
- Pins 3/5: I2C (MPU9250)
- Pin 18: Gas sensor D0 (if enabled)

---

## Testing

### 1. Launch the node:
```bash
roslaunch elderly_bot bringup.launch enable_sensors:=true
```

### 2. Manual test:
```bash
# Turn buzzer ON
rostopic pub /buzzer_command std_msgs/Bool "data: true"

# Turn buzzer OFF
rostopic pub /buzzer_command std_msgs/Bool "data: false"
```

### 3. Expected behavior:
- Buzzer sounds when `data: true` is published
- Buzzer stops when `data: false` is published
- Auto-shutoff after 5 seconds if no commands received (safety feature)

---

## Troubleshooting

### Buzzer doesn't sound:
1. **Check polarity**: Swap buzzer leads if using polarized buzzer
2. **Check transistor**: Ensure correct pinout (E-B-C)
3. **Check resistor connection**: Should connect GPIO → Base, not directly
4. **Check 5V power**: Verify Pin 2 has 5V with multimeter
5. **Test GPIO**: Manually toggle pin: `sudo echo 18 > /sys/class/gpio/export`

### Buzzer always ON:
1. **Check transistor orientation**: Collector/Emitter may be swapped
2. **Check for shorts**: Ensure no wires touching
3. **Check GPIO state**: `gpio readall` shows pin state

### Weak/distorted sound:
1. **Voltage drop**: Check 5V rail under load (should be >4.75V)
2. **Poor connections**: Secure all wire connections
3. **Wrong resistor**: Verify 1kΩ (not 10kΩ or 100Ω)

### "GPIO already in use" error:
1. **Pin conflict**: Check `buzzer_pin` doesn't conflict with other hardware
2. **Cleanup**: Run `sudo chmod 666 /sys/class/gpio/unexport` then retry

---

## Safety Notes

- ⚠️ **Do not** connect buzzer directly to GPIO (may damage Jetson)
- ⚠️ Always use transistor for buzzer switching
- ⚠️ 1kΩ resistor is critical - protects GPIO from overcurrent
- ⚠️ Double-check polarity before powering on
- ✅ This circuit is safe - GPIO current ~2.3mA, well below 10mA limit

---

## Theory: Current Calculations

**Base current** (GPIO → Base):
```
I_base = (V_gpio - V_be) / R_base
       = (3.3V - 0.7V) / 1000Ω
       = 2.6mA  (safe, GPIO limit is 10mA)
```

**Collector current** (5V → Buzzer):
```
I_collector = I_buzzer ≈ 30-50mA
h_fe (gain) = I_collector / I_base ≈ 12-19
(2N2222 has h_fe ≥ 100, so we're well within saturation)
```

**Result**: GPIO safely controls buzzer that draws 10x more current than GPIO can provide directly.

---

## Next Steps

1. Wire according to diagram above
2. Set `enable_buzzer: true` in config
3. Launch bringup with `enable_sensors:=true`
4. Test with `rostopic pub /buzzer_command`
5. Integrate with gas detection: buzzer activates when gas detected

**Integration**: The `sensors_actuators_node.py` automatically activates the buzzer when gas is detected (if both are enabled).
