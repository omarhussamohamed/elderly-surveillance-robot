# MQ-6 Gas Sensor Debug & Testing Guide

## ✅ VOLTAGE CONFIRMED: 3.3V Operation (SAFE)

**Your current setup:**
- MQ-6 powered with 3.3V
- D0 signal: 3.3V logic (safe for Jetson GPIO)
- No level shifter needed
- Expected polarity: `active_high` (D0 HIGH when gas detected)

**Characteristics of 3.3V operation:**
- ✅ Completely safe for Jetson Nano GPIO
- ⚠️ Slightly less sensitive than 5V operation
- 💡 May require closer gas source for detection
- 🔧 Potentiometer adjustment more critical

---

## 🔧 Quick Rebuild & Test

```bash
# Rebuild with new code
cd ~/catkin_ws
catkin_make

# Restart node
rosnode kill /sensors_actuators_node

# Watch startup diagnostics (IMPORTANT!)
rostopic echo /rosout | grep -i gas
```

---

## 🧪 Testing Procedure

### Step 1: Check Startup Logs

The node will now show extensive diagnostics:
```
✓ GAS SENSOR INITIALIZED
  Hardware: MQ-6 Digital Gas Sensor
  Pin: 24 (BOARD) = GPIO 8 (BCM)
  Polarity: ACTIVE_LOW (D0 LOW when gas detected)
  Initial pin state: HIGH
  Initial detection: NO GAS (safe)
  
  Monitoring pin stability for 2 seconds...
  ✓ Pin stable (no changes in 2s) - good!
```

**Look for:**
- ✅ "Pin stable" = Good, potentiometer tuned correctly
- ⚠️ "Pin changed X times" = Near threshold, adjust potentiometer
- ❌ "Pin very unstable" = 5V signal issue or bad tuning

---

### Step 2: Test Gas Detection

```bash
# Terminal 1: Watch topic
rostopic echo /gas_detected

# Terminal 2: Watch detailed logs
rostopic echo /rosout | grep "GAS"

# Test:
# 1. Wave gas source near sensor (lighter WITHOUT flame)
# 2. LED should turn ON
# 3. Wait 0.5 seconds (software debounce)
# 4. Should see: data: true
# 5. Remove gas
# 6. LED turns OFF
# 7. Wait 0.5 seconds
# 8. Should see: data: false
```

---

## 🐛 Troubleshooting

### Issue: `/gas_detected` never changes

**Check startup logs for:**
```
[GAS] Interrupt #1: pin=HIGH, dt=0.523s
```

**If you see interrupts:**
- ✅ GPIO working, check if polarity matches behavior
- With 3.3V power, should be `gas_polarity: "active_high"`
- **If LED ON shows "NO GAS":** Change to `"active_low"` in config

**If NO interrupts appear:**
- ❌ Pin not receiving signal
- Possible causes:
  1. D0 not connected to Pin 24
  2. Sensor not powered (check 3.3V connection)
  3. Wrong GPIO pin number
  4. MQ-6 not warmed up (wait 60 seconds)

---

### Issue: Rapid toggling (data: true/false/true/false)

**Cause:** Pin oscillating near threshold

**Solutions:**
1. Adjust MQ-6 potentiometer (turn clockwise for less sensitive)
2. Increase stability time in config:
   ```yaml
   gas_stability_time: 1.0  # Increase to 1 second
   ```
3. Let sensor warm up (30-60 seconds)

---

### Issue: Works but delayed response

**Expected:** 0.5-1.0 second delay (this is NORMAL and CORRECT)

**Why:** Software debouncing prevents false alarms. Real gas detection physics takes seconds, not milliseconds.

**If too slow:**
```yaml
gas_stability_time: 0.3  # Reduce to 300ms
```

---

## 🔍 Detailed Debug Mode

**Enable verbose logging:**
```bash
# Edit bringup.launch or run manually:
rosrun elderly_bot sensors_actuators_node.py _gas_stability_time:=0.5 __log_level:=debug

# You'll see EVERY interrupt:
[GAS] Interrupt #1: pin=HIGH, dt=0.000s
[GAS] Interrupt #2: pin=LOW, dt=0.523s
[GAS] Pin changed to LOW (interpreted: GAS DETECTED), waiting 0.5s for stability...
```

---

## ⚙️ Configuration Parameters

Edit `config/sensors_actuators.yaml`:

```yaml
# Test different polarities
gas_polarity: "active_low"   # Try "active_high" if not working

# Adjust stability time
gas_stability_time: 0.5      # 0.3-1.0 seconds recommended

# If using different pin
gas_sensor_gpio_pin: 24      # Physical pin number (BOARD mode)
```

---

## 📊 Expected Behavior (3.3V Active-High)

**Normal Operation:**
1. Clean air: LED OFF, D0 = LOW, `/gas_detected` = false
2. Gas present: LED ON, D0 = HIGH
3. **Wait 500ms** (software debounce)
4. `/gas_detected` = true
5. Gas removed: LED OFF, D0 = LOW
6. **Wait 500ms**
7. `/gas_detected` = false

**Each state change will log:**
```
⚠⚠⚠ GAS DETECTED! ⚠⚠⚠
  Pin: HIGH (polarity: active_high) → ALARM
  Stable for: 0.52 seconds
  Total interrupts: 3
```

---

## ✅ Success Checklist

- [x] Verified D0 voltage (3.3V - SAFE)
- [x] Config set to `gas_polarity: "active_high"`
- [ ] Sensor warmed up (wait 60 seconds after power-on)
- [ ] Startup shows "Pin stable" or minimal changes
- [ ] Interrupts appear in debug logs
- [ ] LED ON (gas) → D0 HIGH → wait 0.5s → `/gas_detected` true
- [ ] LED OFF (clear) → D0 LOW → wait 0.5s → `/gas_detected` false
- [ ] No rapid toggling (stable states)
- [ ] Logs show "⚠⚠⚠ GAS DETECTED!" when expected

---

## 🆘 Still Not Working?

**Post this information:**
1. Startup log (first 50 lines) - shows polarity and stability
2. MQ-6 module model/brand
3. Confirm 3.3V connection (VCC → 3.3V pin on Jetson)
4. Debug log showing interrupt count
5. Does LED turn ON when you test with gas?
6. Photo of wiring (if possible)

**Most common fixes for 3.3V operation:**
1. Wrong polarity: Try changing `gas_polarity: "active_low"`
2. Sensor needs warm-up: Wait 60 seconds
3. Potentiometer needs adjustment: Turn clockwise to reduce sensitivity
4. Gas source too far: 3.3V MQ-6 less sensitive, bring gas closer
