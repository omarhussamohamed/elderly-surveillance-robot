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

        self.gyro_offset = [0, 0, 0]  # Placeholder - calibrate in real use
        self.pub = rospy.Publisher('/imu/data_raw', Imu, queue_size=10)
        self.rate = rospy.Rate(100)  # 100 Hz

        rospy.loginfo("MPU9250 node started on bus %d, address 0x%02x", self.bus_id, self.addr)

    def _to_signed_16(self, msb, lsb):
        val = (msb << 8) | lsb
        return val if val < 32768 else val - 65536

    def run(self):
        while not rospy.is_shutdown():
            try:
                # Read accel (0x3B-0x40) and gyro (0x43-0x48)
                data = self.bus.read_i2c_block_data(self.addr, 0x3B, 14)
                a = data[0:6]
                g = data[8:14]

                msg = Imu()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = self.frame_id

                # Scale Accel (±2g = 16384 LSB/g)
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

            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = MPU9250Node()
        node.run()
    except rospy.ROSInterruptException:
        pass