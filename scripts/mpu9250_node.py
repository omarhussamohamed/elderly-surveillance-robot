#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
MPU9250 IMU Node (RAW DATA)

- Publishes raw IMU data to /imu/data_raw
- Acceleration + angular velocity only
- No orientation estimation (orientation_covariance = -1)
- Safe startup & clear error handling
"""

import rospy
import math
import sys
from sensor_msgs.msg import Imu

# ── I2C import handling ─────────────────────────────────────────
try:
    import smbus2 as smbus
except ImportError:
    try:
        import smbus
    except ImportError:
        smbus = None


class MPU9250Node(object):
    def __init__(self):
        rospy.init_node('mpu9250_node', anonymous=False)

        # ── Parameters ─────────────────────────────────────────────
        self.bus_id = rospy.get_param('~i2c_bus', 1)
        self.addr = rospy.get_param('~i2c_address', 0x68)
        self.frame_id = rospy.get_param('~frame_id', 'imu_link')
        self.rate_hz = rospy.get_param('~rate', 100)

        # Gyro calibration offsets (rad/s domain)
        self.gyro_offset = rospy.get_param(
            '~gyro_offset', [0.0, 0.0, 0.0]
        )

        # ── Hardware init ─────────────────────────────────────────
        if smbus is None:
            rospy.logerr("smbus not available — MPU9250 disabled")
            self.bus = None
        else:
            try:
                self.bus = smbus.SMBus(self.bus_id)
                # Wake up MPU9250
                self.bus.write_byte_data(self.addr, 0x6B, 0x00)
                rospy.sleep(0.1)
                rospy.loginfo(
                    "MPU9250 initialized on bus %d addr 0x%02X",
                    self.bus_id, self.addr
                )
            except Exception as e:
                rospy.logerr("I2C init failed: %s", str(e))
                self.bus = None

        # ── ROS I/O ───────────────────────────────────────────────
        self.pub = rospy.Publisher(
            '/imu/data_raw', Imu, queue_size=10
        )

        self.rate = rospy.Rate(self.rate_hz)

    # ─────────────────────────────────────────────────────────────
    @staticmethod
    def _to_int16(msb, lsb):
        val = (msb << 8) | lsb
        return val if val < 32768 else val - 65536

    # ─────────────────────────────────────────────────────────────
    def _read_sensor(self):
        # Accel: 0x3B–0x40, Gyro: 0x43–0x48
        data = self.bus.read_i2c_block_data(self.addr, 0x3B, 14)
        accel = data[0:6]
        gyro = data[8:14]
        return accel, gyro

    # ─────────────────────────────────────────────────────────────
    def run(self):
        rospy.loginfo("MPU9250 node running")

        while not rospy.is_shutdown():
            if self.bus is None:
                rospy.sleep(1.0)
                continue

            try:
                accel, gyro = self._read_sensor()

                msg = Imu()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = self.frame_id

                # ── Acceleration (±2g → 16384 LSB/g) ──
                msg.linear_acceleration.x = (
                    self._to_int16(accel[0], accel[1]) / 16384.0
                ) * 9.806
                msg.linear_acceleration.y = (
                    self._to_int16(accel[2], accel[3]) / 16384.0
                ) * 9.806
                msg.linear_acceleration.z = (
                    self._to_int16(accel[4], accel[5]) / 16384.0
                ) * 9.806

                # ── Angular velocity (131 LSB/deg/s → rad/s) ──
                msg.angular_velocity.x = math.radians(
                    (self._to_int16(gyro[0], gyro[1]) / 131.0)
                    - self.gyro_offset[0]
                )
                msg.angular_velocity.y = math.radians(
                    (self._to_int16(gyro[2], gyro[3]) / 131.0)
                    - self.gyro_offset[1]
                )
                msg.angular_velocity.z = math.radians(
                    (self._to_int16(gyro[4], gyro[5]) / 131.0)
                    - self.gyro_offset[2]
                )

                # Orientation not provided
                msg.orientation_covariance[0] = -1

                self.pub.publish(msg)

            except Exception as e:
                rospy.logwarn("MPU9250 read error: %s", str(e))

            self.rate.sleep()


if __name__ == '__main__':
    try:
        node = MPU9250Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
