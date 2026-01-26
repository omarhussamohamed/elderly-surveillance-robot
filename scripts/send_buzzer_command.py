#!/usr/bin/env python2
# -*- coding: utf-8 -*-
# Simple script to send buzzer command via ROS

import rospy
from std_msgs.msg import Bool
import time

def test_buzzer():
    rospy.init_node('test_buzzer', anonymous=True)
    pub = rospy.Publisher('/buzzer_command', Bool, queue_size=1, latch=True)
    
    # Wait for publisher to connect
    time.sleep(1)
    
    # Turn buzzer ON
    print("Turning buzzer ON...")
    pub.publish(Bool(data=True))
    time.sleep(5)
    
    # Turn buzzer OFF
    print("Turning buzzer OFF...")
    pub.publish(Bool(data=False))
    time.sleep(2)
    
    # Test toggling
    print("Testing toggling (ON/OFF/ON)...")
    pub.publish(Bool(data=True))
    time.sleep(3)
    pub.publish(Bool(data=False))
    time.sleep(1)
    pub.publish(Bool(data=True))
    time.sleep(3)
    pub.publish(Bool(data=False))
    
    print("Test complete!")

if __name__ == '__main__':
    try:
        test_buzzer()
    except Exception as e:
        print("Error: %s" % e)