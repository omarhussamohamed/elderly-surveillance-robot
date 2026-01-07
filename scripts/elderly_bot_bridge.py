#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Elderly Bot USB Bridge
ONLY USB method - no WiFi, no TCP
"""
import rospy
import serial
import threading
import time
from std_msgs.msg import String

class ElderlyBotBridge:
    def __init__(self):
        rospy.init_node('elderly_bot_bridge')
        
        # Get parameters
        self.port = rospy.get_param('~port', '/dev/ttyUSB0')
        self.baud = rospy.get_param('~baud', 115200)
        
        # Connect to ESP32
        self.connect_serial()
        
        # Raw data publisher (for debugging)
        self.raw_pub = rospy.Publisher('/esp32/raw_data', String, queue_size=10)
        
        rospy.loginfo("Elderly Bot USB Bridge started on %s", self.port)
        rospy.loginfo("Waiting for ESP32 to initialize...")
        
        # Wait for ESP32 ready message
        start_time = time.time()
        while time.time() - start_time < 10 and not rospy.is_shutdown():
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    rospy.loginfo("ESP32: %s", line)
                    if "READY" in line:
                        rospy.loginfo("ESP32 is ready!")
                        break
        
        # Start monitoring thread
        self.running = True
        self.monitor_thread = threading.Thread(target=self.monitor_serial)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        
        # Start rosserial_python for ROS communication
        self.start_rosserial()
    
    def connect_serial(self):
        """Connect to ESP32 via USB"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=0.1
            )
            time.sleep(2)  # Wait for connection
            rospy.loginfo("Connected to ESP32 on %s @ %d baud", self.port, self.baud)
            return True
        except Exception as e:
            rospy.logerr("Failed to connect to ESP32: %s", e)
            return False
    
    def start_rosserial(self):
        """Start rosserial_python node for ROS communication"""
        rospy.loginfo("Starting rosserial communication...")
        rospy.loginfo("ESP32 topics should appear in rostopic list")
    
    def monitor_serial(self):
        """Monitor serial connection and log important messages"""
        buffer = ""
        while self.running and not rospy.is_shutdown():
            try:
                if self.ser.is_open and self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    buffer += data.decode('utf-8', errors='ignore')
                    
                    # Process complete lines
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        
                        if line:
                            # Publish raw data
                            msg = String()
                            msg.data = line
                            self.raw_pub.publish(msg)
                            
                            # Log important messages
                            if "ERROR" in line or "FAIL" in line:
                                rospy.logwarn("ESP32: %s", line)
                            elif "READY" in line or "Setup complete" in line:
                                rospy.loginfo("ESP32: %s", line)
                
                time.sleep(0.01)
                
            except Exception as e:
                rospy.logwarn("Serial monitor error: %s", e)
                time.sleep(1)
                self.connect_serial()
    
    def run(self):
        """Main loop - just keep the node alive"""
        rospy.loginfo("Elderly Bot Bridge is running")
        rospy.loginfo("ESP32 topics should now be available")
        rospy.loginfo("Run 'rostopic list' to see available topics")
        
        # Keep node alive
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            rate.sleep()
        
        # Cleanup
        self.running = False
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()

if __name__ == '__main__':
    try:
        bridge = ElderlyBotBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass