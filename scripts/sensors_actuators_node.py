#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Sensors and Actuators Node for Elderly Bot
Handles MQ-6 gas sensor, active buzzer, and Jetson monitoring.
"""

import rospy
import time
import threading
import subprocess
import os
import warnings
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Temperature

# Python 2.7 compatibility
if not hasattr(subprocess, 'DEVNULL'):
    subprocess.DEVNULL = open(os.devnull, 'wb')

# === HARDWARE IMPORTS ===
GPIO_AVAILABLE = False
try:
    import Jetson.GPIO as GPIO
    GPIO.setwarnings(False)
    GPIO_AVAILABLE = True
except ImportError:
    GPIO = None

ADS_AVAILABLE = False
try:
    import smbus2
    ADS_AVAILABLE = True
except ImportError:
    smbus2 = None

JTOP_AVAILABLE = False
try:
    from jtop import jtop
    JTOP_AVAILABLE = True
except ImportError:
    jtop = None


def setup_gpio_permissions(pin_number):
    """Setup GPIO pin permissions."""
    try:
        subprocess.call(['sudo', 'chmod', '666', '/sys/class/gpio/export'], 
                       stderr=subprocess.DEVNULL)
        subprocess.call(['sudo', 'bash', '-c', 
                        'echo {} > /sys/class/gpio/export 2>/dev/null || true'.format(pin_number)],
                       stderr=subprocess.DEVNULL)
        subprocess.call(['sudo', 'chmod', '-R', '666', 
                        '/sys/class/gpio/gpio{}'.format(pin_number)],
                       stderr=subprocess.DEVNULL)
        time.sleep(0.1)
        return True
    except:
        return False


class SensorsActuatorsNode:
    """ROS node for gas sensor, buzzer, and Jetson monitoring."""
    
    def __init__(self):
        rospy.init_node('sensors_actuators_node', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        
        # Parameters
        self.enable_gas_sensor = rospy.get_param('~enable_gas_sensor', False)
        self.enable_buzzer = rospy.get_param('~enable_buzzer', False)
        self.enable_jetson_stats = rospy.get_param('~enable_jetson_stats', True)
        
        self.gas_sensor_mode = rospy.get_param('~gas_sensor_mode', 'gpio')
        self.gas_sensor_gpio_pin = rospy.get_param('~gas_sensor_gpio_pin', 24)
        self.gas_threshold_voltage = rospy.get_param('~gas_threshold_voltage', 1.0)
        self.buzzer_pin = rospy.get_param('~buzzer_pin', 0)
        self.adc_i2c_address = rospy.get_param('~adc_i2c_address', 0x48)
        self.adc_i2c_bus = rospy.get_param('~adc_i2c_bus', 1)
        self.publish_rate = rospy.get_param('~publish_rate', 1.0)
        
        # Hardware state
        self.i2c_bus = None
        self.gas_gpio_initialized = False
        self.buzzer_initialized = False
        self.jtop_handle = None
        
        # Gas sensor state (polling-based)
        self.last_gas_detected = False
        self.gas_detection_lock = threading.Lock()
        
        # One-time warnings
        self.voltage_warning_shown = False
        
        # Polarity configuration: active_high (hardware verified)
        # MQ-6 with voltage divider: GPIO.HIGH = gas detected, LED ON
        self.gas_polarity = rospy.get_param('~gas_polarity', 'active_high')
        
        # Buzzer warning pattern state (MANUAL CONTROL ONLY)
        self.buzzer_lock = threading.Lock()
        self.buzzer_warning_active = False
        self.buzzer_warning_thread = None
        
        # ADS1115 registers
        self.ADS1115_REG_CONVERSION = 0x00
        self.ADS1115_REG_CONFIG = 0x01
        
        # Initialize hardware
        if self.enable_gas_sensor:
            self._init_gas_sensor()
        
        if self.enable_buzzer:
            self._init_buzzer()
        
        if self.enable_jetson_stats:
            self._init_jetson_stats()
        
        # ROS interface
        self.gas_detected_pub = rospy.Publisher('/gas_detected', Bool, queue_size=1)
        self.jetson_temp_pub = rospy.Publisher('/jetson_temperature', Temperature, queue_size=1)
        self.jetson_power_pub = rospy.Publisher('/jetson_power', Float32, queue_size=1)
        rospy.Subscriber('/buzzer_command', Bool, self.buzzer_command_callback)
        
        rospy.loginfo("=== Sensors & Actuators Node Ready ===")
        rospy.loginfo("  Gas: {} | Buzzer: {} | Stats: {}".format(
            'OK' if (self.gas_gpio_initialized or self.i2c_bus) else 'OFF',
            'OK' if self.buzzer_initialized else 'OFF',
            'OK' if self.jtop_handle else 'OFF'
        ))
    
    def _init_gas_sensor(self):
        """Initialize gas sensor (GPIO or I2C mode)."""
        rospy.loginfo("Initializing gas sensor in {} mode...".format(self.gas_sensor_mode))
        if self.gas_sensor_mode == 'gpio':
            self._init_gas_sensor_gpio()
        elif self.gas_sensor_mode == 'i2c':
            self._init_gas_sensor_i2c()
        else:
            rospy.logerr("Invalid gas_sensor_mode: {}".format(self.gas_sensor_mode))
    
    def _init_gas_sensor_gpio(self):
        """Initialize gas sensor in GPIO mode with interrupt detection."""
        if not GPIO_AVAILABLE:
            rospy.logerr("Gas sensor: Jetson.GPIO not available")
            return
        
        if self.gas_sensor_gpio_pin == 0:
            rospy.logerr("Gas sensor: pin not configured")
            return
        
        try:
            setup_gpio_permissions(self.gas_sensor_gpio_pin)
            GPIO.setmode(GPIO.BOARD)
            # Note: Jetson.GPIO ignores pull_up_down, so removed to suppress warning
            # MQ-6 DO has strong output, no pull needed
            GPIO.setup(self.gas_sensor_gpio_pin, GPIO.IN)
            
            # Initialize state
            initial_state = GPIO.input(self.gas_sensor_gpio_pin)
            initial_detected = (initial_state == GPIO.HIGH)  # active_high: HIGH = gas
            
            with self.gas_detection_lock:
                self.last_gas_detected = initial_detected
            
            rospy.loginfo("MQ-6 gas sensor initialized: pin {} (BOARD), polarity: active_high".format(
                self.gas_sensor_gpio_pin))
            rospy.loginfo("Polling-based detection (no interrupts), 20Hz rate")
            
            self.gas_gpio_initialized = True
            
            # Show voltage warning ONCE only (user has divider installed)
            if not self.voltage_warning_shown:
                rospy.logwarn("⚠️ VOLTAGE NOTE: Ensure 10k+2k divider installed on DO pin")
                self.voltage_warning_shown = True
            
            rospy.loginfo("Gas sensor ready - monitoring started")
            rospy.loginfo("="*60)
            
        except Exception as e:
            rospy.logerr("✗ Gas sensor failed: {}".format(e))
            self.gas_gpio_initialized = False
    
    def _init_gas_sensor_i2c(self):
        """Initialize gas sensor in I2C mode."""
        if not ADS_AVAILABLE:
            rospy.logerr("Gas sensor: smbus2 not available")
            return
        
        try:
            self.i2c_bus = smbus2.SMBus(self.adc_i2c_bus)
            config = 0xC183
            config_bytes = [(config >> 8) & 0xFF, config & 0xFF]
            self.i2c_bus.write_i2c_block_data(self.adc_i2c_address, self.ADS1115_REG_CONFIG, config_bytes)
            rospy.loginfo("✓ Gas sensor ready: I2C 0x{:02X}".format(self.adc_i2c_address))
        except Exception as e:
            rospy.logerr("✗ Gas sensor failed: {}".format(e))
            if self.i2c_bus:
                try:
                    self.i2c_bus.close()
                except:
                    pass
            self.i2c_bus = None
    
    def _init_buzzer(self):
        """Initialize buzzer."""
        rospy.loginfo("Initializing buzzer on pin {}...".format(self.buzzer_pin))
        
        if not GPIO_AVAILABLE:
            rospy.logerr("Buzzer: Jetson.GPIO not available")
            return
        
        if self.buzzer_pin == 0:
            rospy.logerr("Buzzer: pin not configured")
            return
        
        try:
            setup_gpio_permissions(self.buzzer_pin)
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(self.buzzer_pin, GPIO.OUT, initial=GPIO.LOW)
            self.buzzer_initialized = True
            rospy.loginfo("✓ Buzzer ready: GPIO pin {}".format(self.buzzer_pin))
        except Exception as e:
            rospy.logerr("✗ Buzzer failed: {}".format(e))
            self.buzzer_initialized = False
    
    def _init_jetson_stats(self):
        """Initialize Jetson monitoring."""
        if not JTOP_AVAILABLE:
            rospy.logwarn("Jetson stats not available - install jetson-stats: sudo -H pip install jetson-stats")
            return
        
        try:
            self.jtop_handle = jtop()
            self.jtop_handle.start()
            # Wait for jtop to initialize
            time.sleep(0.5)
            if self.jtop_handle.ok():
                rospy.loginfo("✓ Jetson stats ready")
            else:
                rospy.logwarn("Jetson stats initialized but not ready")
        except (OSError, IOError) as e:
            # Python 2.7 doesn't have PermissionError
            if hasattr(e, 'errno') and e.errno == 13:  # Permission denied
                rospy.logerr("✗ Jetson stats failed: Permission denied. Run: sudo usermod -aG jtop $USER")
            else:
                rospy.logerr("✗ Jetson stats failed: {}".format(e))
            self.jtop_handle = None
        except Exception as e:
            rospy.logerr("✗ Jetson stats failed: {}".format(e))
            self.jtop_handle = None
    
    def read_gas_sensor(self):
        """Read gas sensor. Returns (voltage, detected) tuple."""
        if self.gas_sensor_mode == 'gpio':
            return self.read_gas_sensor_gpio()
        elif self.gas_sensor_mode == 'i2c':
            return self.read_gas_sensor_i2c()
        return (0.0, False)
    
    def read_gas_sensor_gpio(self):
        """Read gas sensor in GPIO mode (returns cached interrupt-driven value).
        Polarity configured via gas_polarity parameter (active_low or active_high).
        """
        if not self.gas_gpio_initialized:
            return (0.0, False)
        try:
            with self.gas_detection_lock:
                detected = self.last_gas_detected
            return (0.0, detected)
        except Exception as e:
            rospy.logwarn_throttle(10.0, "Gas sensor read error: {}".format(e))
            return (0.0, False)
    
    def read_gas_sensor_i2c(self):
        """Read gas sensor in I2C mode."""
        if self.i2c_bus is None:
            return (0.0, False)
        try:
            config = 0xC183
            config_bytes = [(config >> 8) & 0xFF, config & 0xFF]
            self.i2c_bus.write_i2c_block_data(self.adc_i2c_address, self.ADS1115_REG_CONFIG, config_bytes)
            time.sleep(0.01)
            data = self.i2c_bus.read_i2c_block_data(self.adc_i2c_address, self.ADS1115_REG_CONVERSION, 2)
            raw_adc = (data[0] << 8) | data[1]
            if raw_adc > 32767:
                raw_adc -= 65536
            voltage = abs(raw_adc * 4.096 / 32768.0)
            detected = voltage > self.gas_threshold_voltage
            return (voltage, detected)
        except Exception as e:
            rospy.logwarn_throttle(10.0, "Gas sensor read error: {}".format(e))
            return (0.0, False)
    
    def set_buzzer(self, state):
        """Set buzzer state (True=ON, False=OFF)."""
        if not self.buzzer_initialized:
            return
        with self.buzzer_lock:
            try:
                GPIO.output(self.buzzer_pin, GPIO.HIGH if state else GPIO.LOW)
            except Exception as e:
                rospy.logerr_throttle(5.0, "Buzzer error: {}".format(e))
    
    def start_buzzer_warning(self):
        """Start buzzer (continuous beeping until manually stopped).
        MANUAL CONTROL ONLY - runs indefinitely until /buzzer_command false.
        """
        if not self.buzzer_initialized:
            rospy.logwarn("Cannot start buzzer - not initialized")
            return
        
        with self.buzzer_lock:
            # If already running, don't restart
            if self.buzzer_warning_active:
                rospy.loginfo("Buzzer already active - ignoring duplicate start")
                return
            
            # Set active flag FIRST
            self.buzzer_warning_active = True
            rospy.loginfo("========== BUZZER START (MANUAL) ==========")
        
        def warning_pattern():
            """Continuous beep: 0.1s ON, 0.1s OFF, runs forever until stopped."""
            rospy.loginfo("[BUZZER] Started - manual control (indefinite)")
            beep_count = 0
            
            try:
                while not rospy.is_shutdown():
                    # Check if we should stop
                    with self.buzzer_lock:
                        if not self.buzzer_warning_active:
                            rospy.loginfo("[BUZZER] Stop command received")
                            break
                    
                    # Beep ON
                    self.set_buzzer(True)
                    time.sleep(0.1)
                    
                    # Beep OFF
                    self.set_buzzer(False)
                    time.sleep(0.1)
                    
                    beep_count += 1
                    if beep_count % 50 == 0:  # Log every 10 seconds
                        rospy.loginfo("[BUZZER] Still beeping (count: {})".format(beep_count))
                        
            except Exception as e:
                rospy.logerr("[BUZZER] ERROR: {}".format(e))
            finally:
                self.set_buzzer(False)
                rospy.loginfo("[BUZZER] Stopped (total beeps: {})".format(beep_count))
        
        # Start thread
        self.buzzer_warning_thread = threading.Thread(target=warning_pattern, name="BuzzerThread")
        self.buzzer_warning_thread.daemon = True
        self.buzzer_warning_thread.start()
        rospy.loginfo("========== BUZZER THREAD LAUNCHED ==========")
    
    def stop_buzzer_warning(self):
        """Stop buzzer."""
        rospy.loginfo("========== BUZZER STOP (MANUAL) ==========")
        
        with self.buzzer_lock:
            if not self.buzzer_warning_active:
                rospy.loginfo("Buzzer already stopped")
                return
            
            # Clear the active flag
            self.buzzer_warning_active = False
        
        # Wait for thread to finish (max 1 second)
        if self.buzzer_warning_thread and self.buzzer_warning_thread.is_alive():
            rospy.loginfo("Waiting for buzzer thread to exit...")
            self.buzzer_warning_thread.join(timeout=1.0)
            
            if self.buzzer_warning_thread.is_alive():
                rospy.logwarn("Buzzer thread did not exit cleanly")
        
        # Ensure buzzer is OFF
        self.set_buzzer(False)
        rospy.loginfo("========== BUZZER STOPPED ==========")
    
    def buzzer_command_callback(self, msg):
        """Handle buzzer commands."""
        if not self.buzzer_initialized:
            rospy.logwarn_throttle(5.0, "Buzzer not initialized")
            return
        
        if msg.data:
            self.start_buzzer_warning()
        else:
            self.stop_buzzer_warning()
    
    def read_jetson_stats(self):
        """Read Jetson temperature and power with robust dictionary parsing."""
        if self.jtop_handle is None:
            return (0.0, 0.0)
        
        try:
            if not self.jtop_handle.ok():
                return (0.0, 0.0)
            
            # Read temperature - handle nested dictionaries
            temp = 0.0
            if hasattr(self.jtop_handle, 'temperature'):
                temps = self.jtop_handle.temperature
                if temps and isinstance(temps, dict):
                    # Try CPU first
                    if 'CPU' in temps:
                        try:
                            cpu_val = temps['CPU']
                            # Handle nested dict: {'temp': 33.5, 'online': True}
                            if isinstance(cpu_val, dict):
                                temp = float(cpu_val.get('temp', 0.0))
                            else:
                                temp = float(cpu_val)
                        except (ValueError, TypeError) as e:
                            rospy.logwarn_throttle(30.0, "Invalid CPU temp: {} (error: {})".format(temps['CPU'], e))
                    # Try thermal second
                    elif 'thermal' in temps:
                        try:
                            thermal_val = temps['thermal']
                            if isinstance(thermal_val, dict):
                                temp = float(thermal_val.get('temp', 0.0))
                            else:
                                temp = float(thermal_val)
                        except (ValueError, TypeError) as e:
                            rospy.logwarn_throttle(30.0, "Invalid thermal temp: {} (error: {})".format(temps['thermal'], e))
                    # Average all as fallback
                    elif temps:
                        try:
                            valid_temps = []
                            for k, v in temps.items():
                                try:
                                    if isinstance(v, dict):
                                        valid_temps.append(float(v.get('temp', 0.0)))
                                    else:
                                        valid_temps.append(float(v))
                                except (ValueError, TypeError):
                                    pass
                            if valid_temps:
                                temp = sum(valid_temps) / len(valid_temps)
                        except Exception:
                            pass
            
            # Read power - handle nested dictionaries
            power = 0.0
            if hasattr(self.jtop_handle, 'power'):
                power_data = self.jtop_handle.power
                if isinstance(power_data, dict):
                    # Try different power keys
                    for key in ['tot', 'total', 'ALL', 'cur']:
                        if key in power_data:
                            try:
                                pwr_val = power_data[key]
                                # Handle nested dict: {'power': 5.2, 'unit': 'mW'}
                                if isinstance(pwr_val, dict):
                                    power = float(pwr_val.get('power', pwr_val.get('val', 0.0)))
                                else:
                                    power = float(pwr_val)
                                break
                            except (ValueError, TypeError) as e:
                                rospy.logwarn_throttle(30.0, "Invalid power value for key '{}': {} (error: {})".format(
                                    key, power_data[key], e))
                elif isinstance(power_data, (int, float)):
                    try:
                        power = float(power_data)
                    except (ValueError, TypeError):
                        pass
            
            return (temp, power)
            
        except AttributeError as e:
            rospy.logwarn_throttle(30.0, "Jetson stats attribute error: {}".format(e))
            return (0.0, 0.0)
        except ValueError as e:
            rospy.logwarn_throttle(30.0, "Jetson stats float conversion error: {}".format(e))
            return (0.0, 0.0)
        except Exception as e:
            rospy.logwarn_throttle(30.0, "Jetson stats unexpected error: {}".format(e))
            return (0.0, 0.0)
    
    def publish_sensor_data(self):
        """Publish sensor data - direct GPIO polling only."""
        # === MQ-6 GAS SENSOR (POLLING ONLY) ===
        if self.gas_gpio_initialized:
            # Read GPIO pin directly
            raw = GPIO.input(self.gas_sensor_gpio_pin)
            
            # Active-high: GPIO.HIGH = gas detected
            detected = (raw == GPIO.HIGH)
            
            # Update state
            with self.gas_detection_lock:
                self.last_gas_detected = detected
            
            # Publish every cycle
            self.gas_detected_pub.publish(Bool(data=detected))
            
            # Throttled debug logging (1 Hz)
            rospy.loginfo_throttle(
                1.0,
                "MQ-6 GPIO {} = {} → detected={}".format(
                    self.gas_sensor_gpio_pin,
                    "HIGH" if raw == GPIO.HIGH else "LOW",
                    detected
                )
            )
    
    def run(self):
        """Main loop - 10Hz polling rate for stable gas detection."""
        rate = rospy.Rate(10.0)  # 10 Hz = 100ms cycle
        rospy.loginfo("Main loop running at 10 Hz")
        while not rospy.is_shutdown():
            self.publish_sensor_data()
            rate.sleep()
    
    def shutdown(self):
        """Clean shutdown."""
        rospy.loginfo("Shutting down sensors node...")
        self.stop_buzzer_warning()
        
        # Cleanup GPIO
        if self.gas_gpio_initialized or self.buzzer_initialized:
            try:
                GPIO.cleanup()
            except Exception as e:
                rospy.logwarn("GPIO cleanup warning: {}".format(e))
        
        if self.i2c_bus:
            try:
                self.i2c_bus.close()
            except:
                pass
        if self.jtop_handle:
            try:
                self.jtop_handle.close()
            except:
                pass


if __name__ == '__main__':
    try:
        node = SensorsActuatorsNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
