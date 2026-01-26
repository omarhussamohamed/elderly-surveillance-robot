#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Sensors and Actuators Node - WORKING VERSION
"""

import rospy
import time
import threading
import subprocess
import os
import re
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

class SensorsActuatorsNode:
    """ROS node for gas sensor, buzzer, and Jetson monitoring."""
    
    def __init__(self):
        rospy.init_node('sensors_actuators_node', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        
        # Parameters
        self.enable_gas_sensor = rospy.get_param('~enable_gas_sensor', True)
        self.enable_buzzer = rospy.get_param('~enable_buzzer', True)
        self.enable_jetson_stats = rospy.get_param('~enable_jetson_stats', True)
        
        # GPIO pins
        self.gas_sensor_pin = rospy.get_param('~gas_sensor_gpio_pin', 18)
        self.buzzer_pin = rospy.get_param('~buzzer_pin', 16)
        
        # Polarity configuration
        self.gas_polarity = rospy.get_param('~gas_polarity', 'active_low')
        
        # State variables
        self.current_temp = 0.0
        self.current_power = 0.0
        self.gas_detected = False
        
        # Buzzer control
        self.buzzer_active = False
        self.buzzer_thread = None
        self.buzzer_lock = threading.Lock()
        
        # Initialize hardware
        self.gpio_initialized = False
        
        if self.enable_gas_sensor or self.enable_buzzer:
            self._init_gpio()
        
        # ROS Publishers
        self.gas_pub = rospy.Publisher('/gas_detected', Bool, queue_size=1)
        self.temp_pub = rospy.Publisher('/jetson_temperature', Temperature, queue_size=1)
        self.power_pub = rospy.Publisher('/jetson_power', Float32, queue_size=1)
        
        # ROS Subscriber
        rospy.Subscriber('/buzzer_command', Bool, self.buzzer_callback)
        
        rospy.loginfo("Sensors Node Ready")
        rospy.loginfo("  Gas: %s | Buzzer: %s | Stats: %s",
                     "ON" if self.enable_gas_sensor else "OFF",
                     "ON" if self.enable_buzzer else "OFF",
                     "ON" if self.enable_jetson_stats else "OFF")
    
    def _init_gpio(self):
        """Initialize GPIO pins."""
        if not GPIO_AVAILABLE:
            rospy.logerr("Jetson.GPIO not available - hardware disabled")
            return
        
        try:
            GPIO.setmode(GPIO.BOARD)
            
            # Gas sensor pin (input)
            if self.enable_gas_sensor:
                GPIO.setup(self.gas_sensor_pin, GPIO.IN)
                rospy.loginfo("Gas sensor on pin %d", self.gas_sensor_pin)
            
            # Buzzer pin (output)
            if self.enable_buzzer:
                GPIO.setup(self.buzzer_pin, GPIO.OUT)
                GPIO.output(self.buzzer_pin, GPIO.LOW)
                rospy.loginfo("Buzzer on pin %d", self.buzzer_pin)
            
            self.gpio_initialized = True
            
        except Exception as e:
            rospy.logerr("GPIO init failed: %s", str(e))
            self.gpio_initialized = False
    
    def read_jetson_stats(self):
        """Read Jetson temperature and power using multiple methods."""
        temp = 0.0
        power = 0.0
        
        # Method 1: Use tegrastats (most reliable on Jetson)
        try:
            # Run tegrastats once
            result = subprocess.check_output(['tegrastats', '--interval', '1000', '--count', '1'], 
                                           stderr=subprocess.STDOUT)
            output = result.decode('utf-8')
            
            # Parse temperature (look for patterns like CPU@37.5C or thermal@36.7C)
            temp_patterns = [
                r'CPU@(\d+\.?\d*)C',
                r'thermal@(\d+\.?\d*)C',
                r'GPU@(\d+\.?\d*)C',
                r'AO@(\d+\.?\d*)C'
            ]
            
            for pattern in temp_patterns:
                match = re.search(pattern, output)
                if match:
                    temp = float(match.group(1))
                    break
            
            # Parse power (look for VDD_IN pattern: VDD_IN 1183/1183)
            power_match = re.search(r'VDD_IN\s+(\d+)/\d+', output)
            if power_match:
                # Current in mA, voltage is typically 5V on Jetson
                current_ma = int(power_match.group(1))
                power = (current_ma / 1000.0) * 5.0  # Convert to watts
            
            if temp > 0 or power > 0:
                rospy.logdebug("Tegrastats: %.1fC, %.2fW", temp, power)
                
        except Exception as e:
            rospy.logwarn_throttle(60, "Tegrastats error: %s", str(e))
        
        # Method 2: Fallback to sysfs if tegrastats fails
        if temp == 0.0:
            try:
                # Read from thermal zone 0
                with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                    temp_raw = f.read().strip()
                    temp = float(temp_raw) / 1000.0
            except:
                pass
        
        if power == 0.0:
            try:
                # Try common power sensor paths
                power_paths = [
                    '/sys/bus/i2c/drivers/ina3221x/6-0040/iio:device0/in_power0_input',
                    '/sys/bus/i2c/drivers/ina3221/6-0040/iio:device0/in_power0_input',
                ]
                
                for path in power_paths:
                    if os.path.exists(path):
                        with open(path, 'r') as f:
                            power_raw = f.read().strip()
                            power = float(power_raw) / 1000000.0
                            break
            except:
                pass
        
        return (temp, power)
    
    def read_gas_sensor(self):
        """Read gas sensor state."""
        if not self.gpio_initialized or not self.enable_gas_sensor:
            return False
        
        try:
            state = GPIO.input(self.gas_sensor_pin)
            
            # Convert based on polarity
            if self.gas_polarity == 'active_low':
                # LOW = gas detected (LM393 open-collector pulls low)
                detected = (state == GPIO.LOW)
            else:  # active_high
                # HIGH = gas detected
                detected = (state == GPIO.HIGH)
            
            return detected
            
        except Exception as e:
            rospy.logwarn_throttle(10, "Gas sensor read error: %s", str(e))
            return False
    
    def buzzer_callback(self, msg):
        """Handle buzzer commands."""
        if not self.gpio_initialized or not self.enable_buzzer:
            rospy.logwarn("Buzzer not initialized")
            return
        
        with self.buzzer_lock:
            # If command is same as current state, do nothing
            if msg.data == self.buzzer_active:
                return
            
            self.buzzer_active = msg.data
            
            if msg.data:  # Turn ON
                self.start_buzzer()
                rospy.loginfo("Buzzer ON")
            else:  # Turn OFF
                self.stop_buzzer()
                rospy.loginfo("Buzzer OFF")
    
    def start_buzzer(self):
        """Start buzzer in a thread."""
        def buzzer_pattern():
            """Continuous beeping pattern."""
            try:
                while not rospy.is_shutdown():
                    with self.buzzer_lock:
                        if not self.buzzer_active:
                            break
                    
                    # Beep pattern: 0.1s ON, 0.1s OFF
                    GPIO.output(self.buzzer_pin, GPIO.HIGH)
                    time.sleep(0.1)
                    GPIO.output(self.buzzer_pin, GPIO.LOW)
                    time.sleep(0.1)
                    
            except Exception as e:
                rospy.logerr("Buzzer thread error: %s", str(e))
            finally:
                # Ensure buzzer is OFF
                try:
                    GPIO.output(self.buzzer_pin, GPIO.LOW)
                except:
                    pass
        
        # Stop existing thread if running
        if self.buzzer_thread and self.buzzer_thread.is_alive():
            self.buzzer_thread.join(timeout=0.5)
        
        # Start new thread
        self.buzzer_thread = threading.Thread(target=buzzer_pattern)
        self.buzzer_thread.daemon = True
        self.buzzer_thread.start()
    
    def stop_buzzer(self):
        """Stop buzzer."""
        with self.buzzer_lock:
            self.buzzer_active = False
        
        # Wait for thread to finish
        if self.buzzer_thread and self.buzzer_thread.is_alive():
            self.buzzer_thread.join(timeout=0.5)
        
        # Ensure buzzer is OFF
        try:
            if self.gpio_initialized:
                GPIO.output(self.buzzer_pin, GPIO.LOW)
        except:
            pass
    
    def publish_all_data(self):
        """Read and publish all sensor data."""
        # Read gas sensor
        if self.enable_gas_sensor:
            self.gas_detected = self.read_gas_sensor()
            self.gas_pub.publish(Bool(data=self.gas_detected))
        
        # Read and publish Jetson stats
        if self.enable_jetson_stats:
            temp, power = self.read_jetson_stats()
            self.current_temp = temp
            self.current_power = power
            
            # Publish temperature
            temp_msg = Temperature()
            temp_msg.temperature = temp
            temp_msg.header.stamp = rospy.Time.now()
            self.temp_pub.publish(temp_msg)
            
            # Publish power
            self.power_pub.publish(Float32(data=power))
            
            # Log only if we have valid readings
            if temp > 0:
                rospy.loginfo_throttle(30, "Jetson: %.1fC, %.1fW", temp, power)
    
    def run(self):
        """Main loop."""
        rate = rospy.Rate(2.0)  # 2 Hz = twice per second
        
        while not rospy.is_shutdown():
            self.publish_all_data()
            rate.sleep()
    
    def shutdown(self):
        """Clean shutdown."""
        rospy.loginfo("Shutting down sensors node...")
        
        # Stop buzzer
        self.stop_buzzer()
        
        # Cleanup GPIO
        if self.gpio_initialized:
            try:
                GPIO.cleanup()
            except:
                pass

if __name__ == '__main__':
    try:
        node = SensorsActuatorsNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Node error: %s", str(e))