#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Sensors and Actuators Node - SIMPLIFIED WORKING VERSION
ONLY publishes: gas_detected, jetson_temperature, jetson_power
ONLY subscribes: /buzzer_command
"""

import rospy
import time
import threading
import subprocess
import os
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
    rospy.logwarn("Jetson.GPIO not available. Install: sudo pip install Jetson.GPIO")

JTOP_AVAILABLE = False
try:
    from jtop import jtop
    JTOP_AVAILABLE = True
except ImportError:
    jtop = None
    rospy.logwarn("jtop not available. Install: sudo -H pip install jetson-stats")

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
        self.gas_sensor_pin = rospy.get_param('~gas_sensor_gpio_pin', 18)  # BOARD pin 18
        self.buzzer_pin = rospy.get_param('~buzzer_pin', 16)  # BOARD pin 16
        
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
        self.jtop_handle = None
        
        if self.enable_gas_sensor or self.enable_buzzer:
            self._init_gpio()
        
        if self.enable_jetson_stats:
            self._init_jetson_stats()
        
        # ROS Publishers
        self.gas_pub = rospy.Publisher('/gas_detected', Bool, queue_size=1)
        self.temp_pub = rospy.Publisher('/jetson_temperature', Temperature, queue_size=1)
        self.power_pub = rospy.Publisher('/jetson_power', Float32, queue_size=1)
        
        # ROS Subscriber
        rospy.Subscriber('/buzzer_command', Bool, self.buzzer_callback)
        
        rospy.loginfo("✅ Sensors Node Ready")
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
                rospy.loginfo("Gas sensor on pin %d (active_low)", self.gas_sensor_pin)
            
            # Buzzer pin (output)
            if self.enable_buzzer:
                GPIO.setup(self.buzzer_pin, GPIO.OUT)
                GPIO.output(self.buzzer_pin, GPIO.LOW)  # Start with buzzer OFF
                rospy.loginfo("Buzzer on pin %d", self.buzzer_pin)
            
            self.gpio_initialized = True
            
        except Exception as e:
            rospy.logerr("GPIO init failed: %s", str(e))
            self.gpio_initialized = False
    
    def _init_jetson_stats(self):
        """Initialize Jetson stats monitoring."""
        if not JTOP_AVAILABLE:
            rospy.logwarn("jtop not available - stats disabled")
            return
        
        try:
            self.jtop_handle = jtop()
            self.jtop_handle.start()
            time.sleep(0.5)  # Let jtop initialize
            
            if self.jtop_handle.ok():
                rospy.loginfo("✅ Jetson stats ready")
            else:
                rospy.logwarn("jtop initialized but not ready")
                self.jtop_handle = None
                
        except Exception as e:
            rospy.logerr("jtop init failed: %s", str(e))
            self.jtop_handle = None
    
    def read_jetson_stats(self):
        """Read temperature and power from jtop."""
        if not self.jtop_handle or not self.jtop_handle.ok():
            return (0.0, 0.0)
        
        try:
            # Get stats
            stats = self.jtop_handle.stats
            
            # Extract temperature (try multiple keys)
            temp = 0.0
            if 'temperature' in stats:
                temp_data = stats['temperature']
                if isinstance(temp_data, dict):
                    # Try to get CPU temperature
                    if 'CPU' in temp_data:
                        cpu_temp = temp_data['CPU']
                        if isinstance(cpu_temp, dict):
                            temp = cpu_temp.get('temp', 0.0)
                        else:
                            temp = float(cpu_temp)
                    # Fallback: get first numeric value
                    else:
                        for key, value in temp_data.items():
                            try:
                                temp = float(value)
                                break
                            except:
                                pass
            
            # Extract power (try multiple keys)
            power = 0.0
            if 'power' in stats:
                power_data = stats['power']
                if isinstance(power_data, dict):
                    # Try common power keys
                    for key in ['tot', 'total', 'cur', 'power']:
                        if key in power_data:
                            val = power_data[key]
                            if isinstance(val, dict):
                                power = val.get('power', val.get('val', 0.0))
                            else:
                                power = float(val)
                            break
            
            return (temp, power)
            
        except Exception as e:
            rospy.logwarn_throttle(30, "Stats read error: %s", str(e))
            return (0.0, 0.0)
    
    def read_gas_sensor(self):
        """Read gas sensor state."""
        if not self.gpio_initialized or not self.enable_gas_sensor:
            return False
        
        try:
            # Read GPIO pin
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
                rospy.loginfo("🔔 Buzzer ON")
            else:  # Turn OFF
                self.stop_buzzer()
                rospy.loginfo("🔔 Buzzer OFF")
    
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
        
        # Close jtop
        if self.jtop_handle:
            try:
                self.jtop_handle.close()
            except:
                pass
        
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