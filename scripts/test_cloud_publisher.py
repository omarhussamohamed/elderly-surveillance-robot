#!/usr/bin/env python2
import rospy
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Temperature

def test_cloud_bridge():
    rospy.init_node('test_cloud_publisher')
    
    # Create publishers to simulate sensor data
    temp_pub = rospy.Publisher('/jetson_temperature', Temperature, queue_size=10)
    power_pub = rospy.Publisher('/jetson_power', Float32, queue_size=10)
    gas_pub = rospy.Publisher('/gas_detected', Bool, queue_size=10)
    
    rate = rospy.Rate(1)  # 1 Hz
    
    temp = 45.0
    power = 12.5
    
    rospy.loginfo("Publishing test sensor data for Cloud Bridge...")
    
    while not rospy.is_shutdown():
        # Create temperature message
        temp_msg = Temperature()
        temp_msg.temperature = temp
        temp_msg.header.stamp = rospy.Time.now()
        temp_pub.publish(temp_msg)
        
        # Create power message
        power_msg = Float32()
        power_msg.data = power
        power_pub.publish(power_msg)
        
        # Simulate gas detection every 10 seconds
        if rospy.get_time() % 10 < 1:
            gas_msg = Bool()
            gas_msg.data = True
            gas_pub.publish(gas_msg)
            rospy.loginfo("Gas detected simulation")
        else:
            gas_msg = Bool()
            gas_msg.data = False
            gas_pub.publish(gas_msg)
        
        # Increment values for visibility
        temp += 0.1
        power += 0.01
        
        rate.sleep()

if __name__ == '__main__':
    try:
        test_cloud_bridge()
    except rospy.ROSInterruptException:
        pass