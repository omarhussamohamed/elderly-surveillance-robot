#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Sensors and Actuators Node - FIXED CONTINUOUS BUZZER
"""

import rospy
import time
import threading
import subprocess
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Temperature

class SensorsActuatorsNode:
    """ROS node for gas sensor, buzzer, and Jetson monitoring."""
    
    def __init__(self):
        rospy.init_node('sensors_actuators_node', anonymous=False)
        rospy.loginfo("Initializing Sensors & Actuators Node...")
        
        # Running flag for clean shutdown
        self.running = True
        
        # Load parameters
        self.enable_gas_sensor = rospy.get_param('~enable_gas_sensor', True)
        self.enable_buzzer = rospy.get_param('~enable_buzzer', True)
        self.enable_stats = rospy.get_param('~enable_stats', True)
        
        # GPIO pins
        self.gas_sensor_pin = rospy.get_param('~gas_sensor_gpio_pin', 18)
        self.buzzer_pin = rospy.get_param('~buzzer_pin', 16)
        self.gas_polarity = rospy.get_param('~gas_polarity', 'active_low')
        
        # State variables
        self.gas_detected = False
        self.buzzer_active = False
        self.buzzer_thread = None
        self.buzzer_lock = threading.Lock()
        
        # Initialize hardware
        self.init_hardware()
        
        # ROS Publishers
        self.gas_pub = rospy.Publisher('/gas_detected', Bool, queue_size=1, latch=True)
        self.temp_pub = rospy.Publisher('/jetson_temperature', Temperature, queue_size=1)
        self.power_pub = rospy.Publisher('/jetson_power', Float32, queue_size=1)
        
        # ROS Subscriber
        rospy.Subscriber('/buzzer_command', Bool, self.buzzer_callback, queue_size=1)
        
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Sensors Node Ready [Gas:%s, Buzzer:%s, Stats:%s]", 
                     "ON" if self.enable_gas_sensor else "OFF",
                     "ON" if self.enable_buzzer else "OFF",
                     "ON" if self.enable_stats else "OFF")
    
    def init_hardware(self):
        """Initialize hardware interfaces."""
        # GPIO
        if self.enable_gas_sensor or self.enable_buzzer:
            self.init_gpio()
    
    def init_gpio(self):
        """Initialize GPIO pins."""
        try:
            import Jetson.GPIO as GPIO
            GPIO.setmode(GPIO.BOARD)
            GPIO.setwarnings(False)
            
            if self.enable_gas_sensor:
                GPIO.setup(self.gas_sensor_pin, GPIO.IN)
                rospy.loginfo("Gas sensor on pin %d (polarity: %s)", 
                            self.gas_sensor_pin, self.gas_polarity)
            
            if self.enable_buzzer:
                GPIO.setup(self.buzzer_pin, GPIO.OUT, initial=GPIO.LOW)
                rospy.loginfo("Buzzer on pin %d", self.buzzer_pin)
            
            self.gpio = GPIO
            self.gpio_available = True
            
        except ImportError:
            rospy.logwarn("Jetson.GPIO not available - running in simulation mode")
            self.gpio_available = False
            self.gpio = None
    
    def read_gas_sensor(self):
        """Read gas sensor state."""
        if not self.gpio_available or not self.enable_gas_sensor:
            return False
        
        try:
            state = self.gpio.input(self.gas_sensor_pin)
            
            # Convert based on polarity
            if self.gas_polarity == 'active_low':
                detected = (state == self.gpio.LOW)  # LOW = gas detected
            else:
                detected = (state == self.gpio.HIGH)  # HIGH = gas detected
            
            return detected
            
        except Exception as e:
            rospy.logwarn("Gas sensor read error: %s", str(e))
            return False
    
    def buzzer_callback(self, msg):
        """Handle buzzer commands."""
        if not self.enable_buzzer:
            return
        
        with self.buzzer_lock:
            if msg.data == self.buzzer_active:
                return
            
            self.buzzer_active = msg.data
            
            if msg.data:  # Turn ON
                self.start_buzzer()
                rospy.loginfo("Buzzer ON (continuous)")
            else:  # Turn OFF
                self.stop_buzzer()
                rospy.loginfo("Buzzer OFF")
    
    def start_buzzer(self):
        """Start buzzer in a thread - CONTINUOUS TOGGLING."""
        def buzzer_pattern():
            """Continuous toggling pattern (on-off-on-off)."""
            try:
                while self.running and not rospy.is_shutdown():
                    with self.buzzer_lock:
                        if not self.buzzer_active:
                            break
                    
                    # Toggle every 0.5 seconds
                    if self.gpio_available:
                        self.gpio.output(self.buzzer_pin, self.gpio.HIGH)
                    time.sleep(0.5)
                    
                    if self.gpio_available:
                        self.gpio.output(self.buzzer_pin, self.gpio.LOW)
                    time.sleep(0.5)
                    
            except Exception as e:
                rospy.logwarn("Buzzer thread error: %s", str(e))
            finally:
                # Ensure buzzer is OFF
                if self.gpio_available:
                    try:
                        self.gpio.output(self.buzzer_pin, self.gpio.LOW)
                    except:
                        pass
        
        # Start new thread
        if self.buzzer_thread and self.buzzer_thread.is_alive():
            self.buzzer_thread.join(timeout=0.1)
        
        self.buzzer_thread = threading.Thread(target=buzzer_pattern)
        self.buzzer_thread.daemon = True
        self.buzzer_thread.start()
    
    def stop_buzzer(self):
        """Stop buzzer."""
        with self.buzzer_lock:
            self.buzzer_active = False
        
        # Ensure buzzer is OFF
        if self.gpio_available:
            try:
                self.gpio.output(self.buzzer_pin, self.gpio.LOW)
            except:
                pass
    
    def read_jetson_temperature(self):
        """Read Jetson temperature from thermal zones."""
        try:
            # Read from thermal zone 0
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                temp_millic = int(f.read().strip())
            return temp_millic / 1000.0  # Convert to Celsius
        except:
            return 0.0
    
    def read_jetson_power(self):
        """Read Jetson power consumption - FIXED."""
        try:
            # Use tegrastats for Jetson Nano
            import subprocess
            result = subprocess.run(['tegrastats', '--interval', '100', '--count', '1'], 
                                  capture_output=True, text=True, timeout=2)
            
            if result.returncode == 0:
                output = result.stdout
                # Parse power from tegrastats output
                # Example: "VDD_IN 1651/1651" means 1651 mW
                import re
                
                # Look for VDD_IN (total input power in mW)
                match = re.search(r'VDD_IN\s+(\d+)/(\d+)', output)
                if match:
                    current_power_mw = int(match.group(1))  # Current power in mW
                    return current_power_mw / 1000.0  # Convert to W
                
                # Alternative: sum all VDD_* powers
                total_power_mw = 0
                power_matches = re.findall(r'VDD_\w+\s+(\d+)/(\d+)', output)
                for match in power_matches:
                    total_power_mw += int(match[0])
                
                if total_power_mw > 0:
                    return total_power_mw / 1000.0  # Convert to W
                
        except Exception as e:
            rospy.logdebug("Power read error: %s", str(e))
        
        return 0.0
    
    def publish_data(self):
        """Read and publish all sensor data."""
        # Read gas sensor
        if self.enable_gas_sensor:
            new_gas_state = self.read_gas_sensor()
            if new_gas_state != self.gas_detected:
                self.gas_detected = new_gas_state
                self.gas_pub.publish(Bool(data=self.gas_detected))
                if self.gas_detected:
                    rospy.logwarn("GAS DETECTED")
        
        # Read and publish Jetson stats
        if self.enable_stats:
            # Temperature
            temp_msg = Temperature()
            temp_msg.temperature = self.read_jetson_temperature()
            temp_msg.header.stamp = rospy.Time.now()
            temp_msg.header.frame_id = 'jetson'
            self.temp_pub.publish(temp_msg)
            
            # Power
            power = self.read_jetson_power()
            self.power_pub.publish(Float32(data=power))
    
    def run(self):
        """Main loop."""
        rate = rospy.Rate(2.0)  # 2 Hz
        
        while self.running and not rospy.is_shutdown():
            try:
                self.publish_data()
            except Exception as e:
                rospy.logerr("Publish error: %s", str(e))
            rate.sleep()
    
    def shutdown(self):
        """Clean shutdown."""
        rospy.loginfo("Shutting down sensors node...")
        self.running = False
        
        # Stop buzzer
        self.stop_buzzer()
        
        # Cleanup GPIO
        if hasattr(self, 'gpio_available') and self.gpio_available:
            try:
                self.gpio.cleanup()
            except:
                pass

if __name__ == '__main__':
    try:
        node = SensorsActuatorsNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted")
    except Exception as e:
        rospy.logerr("Main error: %s", str(e))