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
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Temperature

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
        self.gas_sensor_gpio_pin = rospy.get_param('~gas_sensor_gpio_pin', 18)
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
        
        # Buzzer warning pattern state
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
        self.gas_voltage_pub = rospy.Publisher('/gas_level', Float32, queue_size=1)
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
        """Initialize gas sensor in GPIO mode."""
        if not GPIO_AVAILABLE:
            rospy.logerr("Gas sensor: Jetson.GPIO not available")
            return
        
        if self.gas_sensor_gpio_pin == 0:
            rospy.logerr("Gas sensor: pin not configured")
            return
        
        try:
            setup_gpio_permissions(self.gas_sensor_gpio_pin)
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(self.gas_sensor_gpio_pin, GPIO.IN)
            self.gas_gpio_initialized = True
            rospy.loginfo("✓ Gas sensor ready: GPIO pin {}".format(self.gas_sensor_gpio_pin))
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
            rospy.logwarn("Jetson stats not available")
            return
        
        try:
            self.jtop_handle = jtop()
            self.jtop_handle.start()
            rospy.loginfo("Jetson stats ready")
        except Exception as e:
            rospy.logerr("Jetson stats failed: {}".format(e))
            self.jtop_handle = None
    
    def read_gas_sensor(self):
        """Read gas sensor. Returns (voltage, detected) tuple."""
        if self.gas_sensor_mode == 'gpio':
            return self.read_gas_sensor_gpio()
        elif self.gas_sensor_mode == 'i2c':
            return self.read_gas_sensor_i2c()
        return (0.0, False)
    
    def read_gas_sensor_gpio(self):
        """Read gas sensor in GPIO mode."""
        if not self.gas_gpio_initialized:
            return (0.0, False)
        try:
            pin_state = GPIO.input(self.gas_sensor_gpio_pin)
            detected = (pin_state == GPIO.HIGH)
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
        """Start buzzer warning pattern (fast beeping)."""
        if not self.buzzer_initialized:
            return
        
        self.buzzer_warning_active = True
        
        def warning_pattern():
            """Fast beep pattern: 0.1s ON, 0.1s OFF, repeat."""
            while self.buzzer_warning_active and not rospy.is_shutdown():
                self.set_buzzer(True)
                time.sleep(0.1)
                self.set_buzzer(False)
                time.sleep(0.1)
        
        if self.buzzer_warning_thread is None or not self.buzzer_warning_thread.is_alive():
            self.buzzer_warning_thread = threading.Thread(target=warning_pattern)
            self.buzzer_warning_thread.daemon = True
            self.buzzer_warning_thread.start()
    
    def stop_buzzer_warning(self):
        """Stop buzzer warning pattern."""
        self.buzzer_warning_active = False
        self.set_buzzer(False)
    
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
        """Read Jetson temperature and power."""
        if self.jtop_handle is None:
            return (0.0, 0.0)
        try:
            temps = self.jtop_handle.temperature
            avg_temp = sum(temps.values()) / len(temps) if temps else 0.0
            power = self.jtop_handle.power.get('total', 0.0)
            return (avg_temp, power)
        except Exception as e:
            rospy.logwarn_throttle(10.0, "Jetson stats error: {}".format(e))
            return (0.0, 0.0)
    
    def publish_sensor_data(self):
        """Publish sensor data and handle automatic gas alarm."""
        voltage, detected = self.read_gas_sensor()
        self.gas_voltage_pub.publish(Float32(data=voltage))
        self.gas_detected_pub.publish(Bool(data=detected))
        
        # Auto-activate buzzer warning when gas detected
        if detected and self.buzzer_initialized:
            if not self.buzzer_warning_active:
                rospy.logwarn("GAS DETECTED! Activating alarm")
                self.start_buzzer_warning()
        elif not detected and self.buzzer_warning_active:
            rospy.loginfo("Gas cleared, stopping alarm")
            self.stop_buzzer_warning()
        
        temp, power = self.read_jetson_stats()
        temp_msg = Temperature()
        temp_msg.header.stamp = rospy.Time.now()
        temp_msg.temperature = temp
        self.jetson_temp_pub.publish(temp_msg)
        self.jetson_power_pub.publish(Float32(data=power))
    
    def run(self):
        """Main loop."""
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            self.publish_sensor_data()
            rate.sleep()
    
    def shutdown(self):
        """Clean shutdown."""
        rospy.loginfo("Shutting down sensors node...")
        self.stop_buzzer_warning()
        if self.buzzer_initialized:
            try:
                GPIO.cleanup()
            except:
                pass
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
