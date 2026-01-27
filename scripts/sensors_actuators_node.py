#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Sensors and Actuators Node - FIXED (No Power Monitoring)
"""

import rospy
import time
import threading
import atexit
from std_msgs.msg import Bool
from sensor_msgs.msg import Temperature

class SensorsActuatorsNode:
    """ROS node for gas sensor, buzzer, and Jetson temperature only."""
    
    def __init__(self):
        rospy.init_node('sensors_actuators_node', anonymous=False)
        rospy.loginfo("Initializing Sensors & Actuators Node...")
        
        # Load parameters
        self.enable_gas_sensor = rospy.get_param('~enable_gas_sensor', True)
        self.enable_buzzer = rospy.get_param('~enable_buzzer', True)
        self.enable_stats = rospy.get_param('~enable_stats', True)
        
        # GPIO pins (BOARD numbering)
        self.gas_sensor_pin = rospy.get_param('~gas_pin', 18)  # Updated to match config
        self.buzzer_pin = rospy.get_param('~buzzer_pin', 16)
        self.gas_polarity = rospy.get_param('~gas_polarity', 'active_low')
        
        # State variables
        self.gas_detected = False
        self.buzzer_active = False
        self.buzzer_thread = None
        self.buzzer_stop_event = threading.Event()
        self.running = True
        self.last_log_time = 0
        
        # Initialize hardware
        self.init_hardware()
        
        # ROS Publishers
        self.gas_pub = rospy.Publisher('/gas_detected', Bool, queue_size=1, latch=True)
        self.temp_pub = rospy.Publisher('/jetson_temperature', Temperature, queue_size=1)
        # NO POWER PUBLISHER
        
        # ROS Subscriber (for commands FROM cloud via cloud_bridge_node)
        rospy.Subscriber('/buzzer_command', Bool, self.buzzer_callback, queue_size=1)
        
        rospy.on_shutdown(self.shutdown)
        atexit.register(self.cleanup_gpio)
        
        rospy.loginfo("Sensors Node Ready [Gas:%s, Buzzer:%s, Temp:%s]", 
                     "ON" if self.enable_gas_sensor else "OFF",
                     "ON" if self.enable_buzzer else "OFF",
                     "ON" if self.enable_stats else "OFF")
        
        # Delay startup logging
        rospy.sleep(5)
    
    def init_hardware(self):
        """Initialize hardware interfaces."""
        self.gpio_available = False
        
        if (self.enable_gas_sensor or self.enable_buzzer):
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
                
                self.GPIO = GPIO
                self.gpio_available = True
                
            except ImportError:
                rospy.logwarn("Jetson.GPIO not available - running in simulation mode")
                self.gpio_available = False
    
    def read_gas_sensor(self):
        """Read gas sensor state."""
        if not self.gpio_available or not self.enable_gas_sensor:
            return False
        
        try:
            state = self.GPIO.input(self.gas_sensor_pin)
            
            # Convert based on polarity
            if self.gas_polarity == 'active_low':
                detected = (state == self.GPIO.LOW)
            else:
                detected = (state == self.GPIO.HIGH)
            
            return detected
            
        except Exception as e:
            rospy.logwarn("Gas sensor read error: %s", str(e))
            return False
    
    def buzzer_callback(self, msg):
        """Handle buzzer commands FROM cloud_bridge_node."""
        if not self.enable_buzzer:
            return
        
        rospy.loginfo("Buzzer command received: %s", "ON" if msg.data else "OFF")
        
        if msg.data and not self.buzzer_active:
            # Turn buzzer ON
            self.buzzer_active = True
            self.buzzer_stop_event.clear()
            self.start_buzzer_thread()
            
        elif not msg.data and self.buzzer_active:
            # Turn buzzer OFF
            self.buzzer_active = False
            self.buzzer_stop_event.set()
            self.stop_buzzer()
    
    def start_buzzer_thread(self):
        """Start buzzer in a separate thread."""
        def buzzer_loop():
            rospy.logdebug("Buzzer thread started")
            try:
                while not self.buzzer_stop_event.is_set() and not rospy.is_shutdown():
                    if self.gpio_available:
                        self.GPIO.output(self.buzzer_pin, self.GPIO.HIGH)
                    time.sleep(0.5)  # ON for 0.5 seconds
                    
                    if self.gpio_available:
                        self.GPIO.output(self.buzzer_pin, self.GPIO.LOW)
                    time.sleep(0.5)  # OFF for 0.5 seconds
                    
            except Exception as e:
                rospy.logwarn("Buzzer thread error: %s", str(e))
            finally:
                rospy.logdebug("Buzzer thread stopped")
        
        # Stop any existing thread
        if self.buzzer_thread and self.buzzer_thread.is_alive():
            self.buzzer_stop_event.set()
            self.buzzer_thread.join(timeout=1.0)
        
        # Start new thread
        self.buzzer_stop_event.clear()
        self.buzzer_thread = threading.Thread(target=buzzer_loop)
        self.buzzer_thread.daemon = True
        self.buzzer_thread.start()
    
    def stop_buzzer(self):
        """Stop buzzer immediately."""
        if self.gpio_available and self.enable_buzzer:
            try:
                self.GPIO.output(self.buzzer_pin, self.GPIO.LOW)
            except Exception as e:
                rospy.logwarn("Error stopping buzzer: %s", str(e))
    
    def read_jetson_temperature(self):
        """Read Jetson temperature from thermal zones."""
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                temp_millic = int(f.read().strip())
            return temp_millic / 1000.0  # Convert to Celsius
        except Exception as e:
            rospy.logdebug("Temp read error: %s", str(e))
            return 35.0  # Default fallback
    
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
            else:
                # Republish gas even if unchanged for consistent rate
                self.gas_pub.publish(Bool(data=self.gas_detected))
        
        # Read and publish Jetson temperature only
        if self.enable_stats:
            # Temperature
            temp_msg = Temperature()
            temp_msg.temperature = self.read_jetson_temperature()
            temp_msg.header.stamp = rospy.Time.now()
            temp_msg.header.frame_id = 'jetson'
            self.temp_pub.publish(temp_msg)
            
            # Log stats every 1 second
            current_time = time.time()
            if current_time - self.last_log_time >= 1:
                rospy.loginfo("Stats: %.1f°C, Gas: %s", 
                              temp_msg.temperature, self.gas_detected)
                self.last_log_time = current_time
    
    def run(self):
        """Main loop."""
        rate = rospy.Rate(10.0)
        
        while self.running and not rospy.is_shutdown():
            try:
                self.publish_data()
            except Exception as e:
                rospy.logerr("Publish error: %s", str(e))
            rate.sleep()
    
    def cleanup_gpio(self):
        """Cleanup GPIO on exit."""
        if hasattr(self, 'gpio_available') and self.gpio_available:
            try:
                self.GPIO.cleanup()
                rospy.logdebug("GPIO cleanup completed")
            except:
                pass
    
    def shutdown(self):
        """Clean shutdown."""
        rospy.loginfo("Shutting down sensors node...")
        self.running = False
        
        # Stop buzzer
        self.buzzer_active = False
        self.buzzer_stop_event.set()
        self.stop_buzzer()
        
        # Wait for thread to finish
        if self.buzzer_thread and self.buzzer_thread.is_alive():
            self.buzzer_thread.join(timeout=2.0)
        
        # Cleanup GPIO
        self.cleanup_gpio()

if __name__ == '__main__':
    try:
        node = SensorsActuatorsNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted")
    except Exception as e:
        rospy.logerr("Main error: %s", str(e))