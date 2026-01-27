#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Sensors Node - CLEAN LOGS & ON-CHANGE PUBLISHING
- Logs "Stats: XX.X°C, Gas: True/False" exactly every 1s
- Publishes /gas_detected ONLY on change (False->True or True->False) to internal ROS
- Logs "GAS DETECTED" ONLY on rising edge
"""

import rospy
import time
import threading
import atexit
from std_msgs.msg import Bool
from sensor_msgs.msg import Temperature

class SensorsActuatorsNode:
    def __init__(self):
        rospy.init_node('sensors_actuators_node', anonymous=False)
        rospy.loginfo("Initializing Sensors & Actuators Node...")
        
        # Load parameters
        self.enable_gas_sensor = rospy.get_param('~enable_gas_sensor', True)
        self.enable_buzzer = rospy.get_param('~enable_buzzer', True)
        self.enable_stats = rospy.get_param('~enable_stats', True)
        
        # GPIO config
        self.gas_sensor_pin = rospy.get_param('~gas_pin', 18)
        self.buzzer_pin = rospy.get_param('~buzzer_pin', 16)
        self.gas_polarity = rospy.get_param('~gas_polarity', 'active_low')
        
        # State variables
        self.gas_detected = False
        self.cloud_buzzer = False 
        self.buzzer_active = False
        self.buzzer_thread = None
        self.buzzer_stop_event = threading.Event()
        self.running = True
        self.last_log_time = 0
        
        self.init_hardware()
        
        # Publishers
        self.gas_pub = rospy.Publisher('/gas_detected', Bool, queue_size=1, latch=True)
        self.temp_pub = rospy.Publisher('/jetson_temperature', Temperature, queue_size=1)
        
        # Subscriber
        rospy.Subscriber('/buzzer_command', Bool, self.buzzer_callback, queue_size=1)
        
        rospy.on_shutdown(self.shutdown)
        atexit.register(self.cleanup_gpio)
        
        rospy.sleep(5) # Sensor warm-up
    
    def init_hardware(self):
        self.gpio_available = False
        if (self.enable_gas_sensor or self.enable_buzzer):
            try:
                import Jetson.GPIO as GPIO
                GPIO.setmode(GPIO.BOARD)
                GPIO.setwarnings(False)
                if self.enable_gas_sensor:
                    GPIO.setup(self.gas_sensor_pin, GPIO.IN)
                if self.enable_buzzer:
                    GPIO.setup(self.buzzer_pin, GPIO.OUT, initial=GPIO.LOW)
                self.GPIO = GPIO
                self.gpio_available = True
            except ImportError:
                rospy.logwarn("Jetson.GPIO not available - simulation mode")

    def read_gas_sensor(self):
        if not self.gpio_available or not self.enable_gas_sensor:
            return False
        try:
            state = self.GPIO.input(self.gas_sensor_pin)
            return (state == self.GPIO.LOW) if self.gas_polarity == 'active_low' else (state == self.GPIO.HIGH)
        except:
            return False
    
    def buzzer_callback(self, msg):
        if not self.enable_buzzer: return
        self.cloud_buzzer = msg.data
        self.update_buzzer_state()
    
    def update_buzzer_state(self):
        desired = self.gas_detected or self.cloud_buzzer
        if desired and not self.buzzer_active:
            self.buzzer_active = True
            self.buzzer_stop_event.clear()
            self.start_buzzer_thread()
            # Only log status changes if needed, but keeping it quiet per request
        elif not desired and self.buzzer_active:
            self.buzzer_active = False
            self.buzzer_stop_event.set()
            self.stop_buzzer()
    
    def start_buzzer_thread(self):
        def buzzer_loop():
            try:
                while not self.buzzer_stop_event.is_set() and not rospy.is_shutdown():
                    if self.gpio_available:
                        self.GPIO.output(self.buzzer_pin, self.GPIO.HIGH)
                    time.sleep(0.5)
                    if self.gpio_available:
                        self.GPIO.output(self.buzzer_pin, self.GPIO.LOW)
                    time.sleep(0.5)
            except: pass
            finally:
                if self.gpio_available: self.GPIO.output(self.buzzer_pin, self.GPIO.LOW)
        
        if self.buzzer_thread and self.buzzer_thread.is_alive():
            self.buzzer_stop_event.set()
            self.buzzer_thread.join(timeout=1.0)
        
        self.buzzer_stop_event.clear()
        self.buzzer_thread = threading.Thread(target=buzzer_loop)
        self.buzzer_thread.daemon = True
        self.buzzer_thread.start()
    
    def stop_buzzer(self):
        if self.gpio_available and self.enable_buzzer:
            try: self.GPIO.output(self.buzzer_pin, self.GPIO.LOW)
            except: pass
    
    def read_jetson_temperature(self):
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                return int(f.read().strip()) / 1000.0
        except:
            return 35.0
    
    def publish_data(self):
        # 1. Handle Gas Logic
        if self.enable_gas_sensor:
            new_gas_state = self.read_gas_sensor()
            
            # Only publish internally to ROS on state CHANGE
            if new_gas_state != self.gas_detected:
                self.gas_detected = new_gas_state
                self.gas_pub.publish(Bool(data=self.gas_detected))
                
                # Log ONLY on Rising Edge
                if self.gas_detected:
                    rospy.logwarn("GAS DETECTED")
                
                self.update_buzzer_state()
        
        # 2. Handle Temp & Stats Logging
        if self.enable_stats:
            temp_val = self.read_jetson_temperature()
            
            # Publish Temp (High rate ok, bridge handles it)
            temp_msg = Temperature()
            temp_msg.temperature = temp_val
            temp_msg.header.stamp = rospy.Time.now()
            self.temp_pub.publish(temp_msg)
            
            # Stats Log - Exactly 1Hz
            current_time = time.time()
            if current_time - self.last_log_time >= 1:
                rospy.loginfo("Stats: %.1f°C, Gas: %s", 
                              temp_val, 
                              "True" if self.gas_detected else "False")
                self.last_log_time = current_time
    
    def run(self):
        rate = rospy.Rate(10.0)
        while self.running and not rospy.is_shutdown():
            try:
                self.publish_data()
            except Exception as e:
                rospy.logerr("Error: %s", str(e))
            rate.sleep()
    
    def cleanup_gpio(self):
        if hasattr(self, 'gpio_available') and self.gpio_available:
            try: self.GPIO.cleanup()
            except: pass
    
    def shutdown(self):
        self.running = False
        self.buzzer_stop_event.set()
        self.stop_buzzer()
        self.cleanup_gpio()

if __name__ == '__main__':
    try:
        node = SensorsActuatorsNode()
        node.run()
    except rospy.ROSInterruptException:
        pass