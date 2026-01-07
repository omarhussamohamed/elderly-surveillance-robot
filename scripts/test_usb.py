#!/usr/bin/env python
# Test USB communication
import rospy
import time
from geometry_msgs.msg import Twist

def test_usb():
    rospy.init_node('test_usb_connection')
    
    # Publisher for cmd_vel
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # Wait for connection
    time.sleep(2)
    
    print("\n" + "="*50)
    print("ELDERLY BOT USB COMMUNICATION TEST")
    print("="*50)
    
    print("\n1. Checking ROS topics...")
    try:
        import rostopic
        topics = rostopic.get_topic_list()
        esp32_topics = [t for t in topics[0] if 'wheel_odom' in t or 'imu' in t or 'cmd_vel' in t]
        
        if esp32_topics:
            print("✅ ESP32 topics found:")
            for topic in esp32_topics:
                print("   - " + topic)
        else:
            print("❌ No ESP32 topics found")
            return
    except:
        print("⚠️  Could not check topics")
    
    print("\n2. Testing motor control...")
    print("   WARNING: Ensure robot is on a safe surface!")
    
    test_commands = [
        (0.1, 0.0, "Forward"),
        (0.0, 0.3, "Rotate left"),
        (-0.1, 0.0, "Backward"),
        (0.0, -0.3, "Rotate right"),
        (0.0, 0.0, "Stop"),
    ]
    
    for linear, angular, desc in test_commands:
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        
        cmd_pub.publish(twist)
        print(f"   Sent: {desc} (lin={linear:.2f}, ang={angular:.2f})")
        time.sleep(1)
    
    print("\n3. Checking data flow...")
    print("   Run these commands to verify:")
    print("   rostopic hz /wheel_odom")
    print("   rostopic hz /imu/data")
    print("   rostopic echo /wheel_odom")
    
    print("\n" + "="*50)
    print("TEST COMPLETE")
    print("="*50)
    
    rospy.signal_shutdown("Test complete")

if __name__ == '__main__':
    try:
        test_usb()
    except rospy.ROSInterruptException:
        pass