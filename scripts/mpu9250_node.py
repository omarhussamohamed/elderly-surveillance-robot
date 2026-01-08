#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
MPU9250 ROS Node for Jetson Nano
Reads IMU data from MPU9250 connected directly to Jetson I2C bus
and publishes sensor_msgs/Imu messages.

Hardware Setup:
- MPU9250 SDA → Jetson I2C SDA (Pin 3 on J21 - I2C2_SDA)
- MPU9250 SCL → Jetson I2C SCL (Pin 5 on J21 - I2C2_SCL)
- MPU9250 VCC → Jetson 3.3V (Pin 1 on J21)
- MPU9250 GND → Jetson GND (Pin 6, 9, 14, or 20 on J21)

Requirements:
sudo apt-get install python3-smbus i2c-tools
pip3 install mpu9250-jetson  # Or use alternative library
"""

import rospy
import sensor_msgs.msg
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import math
import sys

try:
    # Option 1: Using smbus directly (manual I2C communication)
    import smbus
    
    # Option 2: Using MPU9250 library (if available)
    # from mpu9250 import MPU9250
    
except ImportError as e:
    rospy.logerr("Failed to import required libraries: %s", str(e))
    rospy.logerr("Install with: sudo apt-get install python3-smbus i2c-tools")
    sys.exit(1)


class MPU9250Node:
    """
    ROS node for MPU9250 IMU connected to Jetson I2C bus.
    """
    
    # MPU9250 I2C addresses
    MPU9250_ADDR = 0x68  # Default address (AD0 pin LOW)
    # MPU9250_ADDR = 0x69  # Alternative address (AD0 pin HIGH)
    
    # MPU9250 Registers
    PWR_MGMT_1 = 0x6B
    ACCEL_XOUT_H = 0x3B
    GYRO_XOUT_H = 0x43
    WHO_AM_I = 0x75
    
    # I2C bus (Jetson Nano has multiple I2C buses)
    # I2C1: /dev/i2c-1 (pins 27, 28)
    # I2C2: /dev/i2c-2 (pins 3, 5) - More commonly available
    I2C_BUS = 1  # Change to 2 if using I2C2
    
    def __init__(self):
        rospy.init_node('mpu9250_node', anonymous=False)
        
        # Parameters
        self.frame_id = rospy.get_param('~frame_id', 'imu_link')
        self.publish_rate = rospy.get_param('~publish_rate', 50.0)  # Hz
        self.i2c_bus_num = rospy.get_param('~i2c_bus', 1)
        self.i2c_address = rospy.get_param('~i2c_address', 0x68)
        
        # Initialize I2C
        try:
            self.bus = smbus.SMBus(self.i2c_bus_num)
            rospy.loginfo("Initialized I2C bus %d", self.i2c_bus_num)
        except Exception as e:
            rospy.logfatal("Failed to initialize I2C bus %d: %s", self.i2c_bus_num, str(e))
            sys.exit(1)
        
        # Initialize MPU9250
        if not self._init_mpu9250():
            rospy.logfatal("Failed to initialize MPU9250. Check wiring and I2C address.")
            sys.exit(1)
        
        # Publisher
        self.imu_pub = rospy.Publisher('imu/data', Imu, queue_size=10)
        
        # Calibration (optional - can be done at startup or dynamically)
        self.accel_offset = [0.0, 0.0, 0.0]
        self.gyro_offset = [0.0, 0.0, 0.0]
        
        rospy.loginfo("MPU9250 node started. Publishing to /imu/data at %d Hz", 
                     int(self.publish_rate))
        
        # Main loop
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            self._publish_imu()
            rate.sleep()
    
    def _init_mpu9250(self):
        """
        Initialize MPU9250 sensor.
        Returns True if successful, False otherwise.
        """
        try:
            # Wake up MPU9250 (clear sleep bit)
            self.bus.write_byte_data(self.i2c_address, self.PWR_MGMT_1, 0x00)
            rospy.sleep(0.1)  # Wait for sensor to wake up
            
            # Verify WHO_AM_I register
            who_am_i = self.bus.read_byte_data(self.i2c_address, self.WHO_AM_I)
            if who_am_i == 0x71:  # MPU9250 WHO_AM_I value
                rospy.loginfo("MPU9250 detected (WHO_AM_I = 0x%02X)", who_am_i)
                return True
            else:
                rospy.logwarn("Unexpected WHO_AM_I value: 0x%02X (expected 0x71)", who_am_i)
                return False
                
        except Exception as e:
            rospy.logerr("Error initializing MPU9250: %s", str(e))
            return False
    
    def _read_word_2c(self, addr):
        """
        Read 16-bit signed value from two consecutive registers.
        """
        high = self.bus.read_byte_data(self.i2c_address, addr)
        low = self.bus.read_byte_data(self.i2c_address, addr + 1)
        val = (high << 8) + low
        if val >= 0x8000:
            return -((65535 - val) + 1)
        else:
            return val
    
    def _read_accel_data(self):
        """
        Read accelerometer data.
        Returns (x, y, z) in m/s²
        """
        accel_x = self._read_word_2c(self.ACCEL_XOUT_H) / 16384.0  # ±2g range
        accel_y = self._read_word_2c(self.ACCEL_XOUT_H + 2) / 16384.0
        accel_z = self._read_word_2c(self.ACCEL_XOUT_H + 4) / 16384.0
        
        # Convert from g to m/s²
        accel_x *= 9.80665
        accel_y *= 9.80665
        accel_z *= 9.80665
        
        # Apply calibration offset
        accel_x -= self.accel_offset[0]
        accel_y -= self.accel_offset[1]
        accel_z -= self.accel_offset[2]
        
        return accel_x, accel_y, accel_z
    
    def _read_gyro_data(self):
        """
        Read gyroscope data.
        Returns (x, y, z) in rad/s
        """
        gyro_x = self._read_word_2c(self.GYRO_XOUT_H) / 131.0  # ±250°/s range
        gyro_y = self._read_word_2c(self.GYRO_XOUT_H + 2) / 131.0
        gyro_z = self._read_word_2c(self.GYRO_XOUT_H + 4) / 131.0
        
        # Convert from °/s to rad/s
        gyro_x = math.radians(gyro_x)
        gyro_y = math.radians(gyro_y)
        gyro_z = math.radians(gyro_z)
        
        # Apply calibration offset
        gyro_x -= self.gyro_offset[0]
        gyro_y -= self.gyro_offset[1]
        gyro_z -= self.gyro_offset[2]
        
        return gyro_x, gyro_y, gyro_z
    
    def _publish_imu(self):
        """
        Read IMU data and publish ROS message.
        """
        try:
            # Read sensor data
            accel_x, accel_y, accel_z = self._read_accel_data()
            gyro_x, gyro_y, gyro_z = self._read_gyro_data()
            
            # Create IMU message
            imu_msg = Imu()
            imu_msg.header = Header()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = self.frame_id
            
            # Linear acceleration (m/s²)
            imu_msg.linear_acceleration.x = accel_x
            imu_msg.linear_acceleration.y = accel_y
            imu_msg.linear_acceleration.z = accel_z
            
            # Angular velocity (rad/s)
            imu_msg.angular_velocity.x = gyro_x
            imu_msg.angular_velocity.y = gyro_y
            imu_msg.angular_velocity.z = gyro_z
            
            # Covariance matrices (diagonal)
            # These values should be calibrated for your specific sensor
            imu_msg.linear_acceleration_covariance[0] = 0.04  # x
            imu_msg.linear_acceleration_covariance[4] = 0.04  # y
            imu_msg.linear_acceleration_covariance[8] = 0.04  # z
            
            imu_msg.angular_velocity_covariance[0] = 0.02  # x
            imu_msg.angular_velocity_covariance[4] = 0.02  # y
            imu_msg.angular_velocity_covariance[8] = 0.02  # z
            
            # Orientation (not computed - let robot_localization handle it)
            imu_msg.orientation_covariance[0] = -1  # Unknown
            
            # Publish
            self.imu_pub.publish(imu_msg)
            
        except Exception as e:
            rospy.logerr("Error reading/publishing IMU data: %s", str(e))


if __name__ == '__main__':
    try:
        node = MPU9250Node()
    except rospy.ROSInterruptException:
        pass

