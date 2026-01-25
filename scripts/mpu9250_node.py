#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import math
import sys
from sensor_msgs.msg import Imu

# Import handling
try:
    import smbus2 as smbus
except ImportError:
    try:
        import smbus
    except ImportError:
        print("Dependency Error: smbus/smbus2 not found.")
        sys.exit(1)

class MPU9250Node:
    def __init__(self):
        rospy.init_node('mpu9250_node')
        
        # Params
        self.bus_id = rospy.get_param('~i2c_bus', 1)
        self.addr = 0x68  # Fixed per your confirmation
        self.frame_id = rospy.get_param('~frame_id', 'imu_link')
        
        # Hardware Setup
        try:
            self.bus = smbus.SMBus(self.bus_id)
            # Wake up sensor
            self.bus.write_byte_data(self.addr, 0x6B, 0x00)
            rospy.sleep(0.1)
        except Exception as e:
            rospy.logerr("Could not open I2C bus: %s", str(e))
            sys.exit(1)

        self.gyro_offset = [0.0, 0.0, 0.0]
        self.pub = rospy.Publisher('imu/data_raw', Imu, queue_size=10)
        
        # Initialization Sequence
        self._setup_sensor()
        self._calibrate_gyro()
        
        rate = rospy.Rate(rospy.get_param('~publish_rate', 50.0))
        while not rospy.is_shutdown():
            self._publish_imu()
            rate.sleep()

    def _setup_sensor(self):
        # Set Gyro range +-250deg/s and Accel +-2g
        self.bus.write_byte_data(self.addr, 0x1B, 0x00)
        self.bus.write_byte_data(self.addr, 0x1C, 0x00)
        # Set Low Pass Filter ~41Hz
        self.bus.write_byte_data(self.addr, 0x1A, 0x03)

    def _to_signed_16(self, high, low):
        val = (high << 8) | low
        if val > 32767:
            val -= 65536
        return val

    def _read_block(self, reg):
        # Reads 6 bytes starting from reg (X_H, X_L, Y_H, Y_L, Z_H, Z_L)
        return self.bus.read_i2c_block_data(self.addr, reg, 6)

    def _calibrate_gyro(self):
        rospy.loginfo("Calibrating... do not move IMU.")
        x, y, z = 0, 0, 0
        for _ in range(100):
            data = self._read_block(0x43)
            x += self._to_signed_16(data[0], data[1])
            y += self._to_signed_16(data[2], data[3])
            z += self._to_signed_16(data[4], data[5])
            rospy.sleep(0.01)
        self.gyro_offset = [x/100.0, y/100.0, z/100.0]

    def _publish_imu(self):
        try:
            # Read Accel (0x3B) and Gyro (0x43)
            a = self._read_block(0x3B)
            g = self._read_block(0x43)
            
            msg = Imu()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.frame_id
            
            # Scale Accel (16384 LSB/g) -> m/s^2
            msg.linear_acceleration.x = (self._to_signed_16(a[0], a[1]) / 16384.0) * 9.806
            msg.linear_acceleration.y = (self._to_signed_16(a[2], a[3]) / 16384.0) * 9.806
            msg.linear_acceleration.z = (self._to_signed_16(a[4], a[5]) / 16384.0) * 9.806
            
            # Scale Gyro (131 LSB/deg/s) -> rad/s (and apply calibration)
            msg.angular_velocity.x = math.radians((self._to_signed_16(g[0], g[1]) - self.gyro_offset[0]) / 131.0)
            msg.angular_velocity.y = math.radians((self._to_signed_16(g[2], g[3]) - self.gyro_offset[1]) / 131.0)
            msg.angular_velocity.z = math.radians((self._to_signed_16(g[4], g[5]) - self.gyro_offset[2]) / 131.0)
            
            # No orientation data provided by this raw node
            msg.orientation_covariance[0] = -1
            
            self.pub.publish(msg)
        except Exception as e:
            rospy.logwarn("Data read error: %s", str(e))

if __name__ == '__main__':
    try:
        MPU9250Node()
    except rospy.ROSInterruptException:
        pass