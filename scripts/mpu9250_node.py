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
from sensor_msgs.msg import Imu, Temperature, MagneticField
from std_msgs.msg import Header
import math
import sys

try:
    # Try smbus2 first (Python 3, modern)
    try:
        import smbus2 as smbus
        rospy.loginfo("Using smbus2")
    except ImportError:
        # Fall back to smbus (Python 2, older)
        try:
            import smbus
            rospy.loginfo("Using smbus")
        except ImportError:
            rospy.logerr("Failed to import smbus. Install with:")
            rospy.logerr("  sudo apt-get install python3-smbus i2c-tools")
            rospy.logerr("  OR: pip3 install smbus2")
            sys.exit(1)
except Exception as e:
    rospy.logerr("Failed to import required libraries: %s", str(e))
    sys.exit(1)


class MPU9250Node:
    """
    ROS node for MPU9250 IMU connected to Jetson I2C bus.
    """
    
    # MPU9250 I2C addresses
    MPU9250_ADDR = 0x68  # Default address (AD0 pin LOW)
    # MPU9250_ADDR = 0x69  # Alternative address (AD0 pin HIGH)
    
    # AK8963 (Magnetometer) I2C address
    AK8963_ADDR = 0x0C
    
    # MPU9250 Registers
    PWR_MGMT_1 = 0x6B
    ACCEL_XOUT_H = 0x3B
    GYRO_XOUT_H = 0x43
    TEMP_OUT_H = 0x41
    WHO_AM_I = 0x75
    INT_PIN_CFG = 0x37
    USER_CTRL = 0x6A
    
    # AK8963 Registers
    AK8963_WHO_AM_I = 0x00
    AK8963_CNTL1 = 0x0A
    AK8963_CNTL2 = 0x0B
    AK8963_ST1 = 0x02
    AK8963_HXL = 0x03
    AK8963_ASAX = 0x10  # Magnetometer sensitivity adjustment values
    
    # I2C bus (Jetson Nano has multiple I2C buses)
    # I2C1: /dev/i2c-1 (pins 27, 28)
    # I2C2: /dev/i2c-2 (pins 3, 5) - More commonly available
    I2C_BUS = 1  # Change to 2 if using I2C2
    
    def __init__(self):
        rospy.init_node('mpu9250_node', anonymous=False)
        
        # Parameters
        self.frame_id = rospy.get_param('~frame_id', 'imu_link')
        self.publish_rate = rospy.get_param('~publish_rate', 50.0)  # Hz
        self.i2c_bus_num = int(rospy.get_param('~i2c_bus', 1))  # Ensure integer
        i2c_addr_str = rospy.get_param('~i2c_address', '0x68')
        
        # Convert I2C address to integer (handle both string and int)
        if isinstance(i2c_addr_str, str):
            if i2c_addr_str.startswith('0x') or i2c_addr_str.startswith('0X'):
                self.i2c_address = int(i2c_addr_str, 16)
            else:
                self.i2c_address = int(i2c_addr_str)
        else:
            self.i2c_address = int(i2c_addr_str)
        
        rospy.loginfo("I2C bus: %d, Address: 0x%02X", self.i2c_bus_num, self.i2c_address)
        
        # Initialize I2C
        try:
            self.bus = smbus.SMBus(self.i2c_bus_num)
            rospy.loginfo("Initialized I2C bus %d", self.i2c_bus_num)
        except Exception as e:
            rospy.logfatal("Failed to initialize I2C bus %d: %s", self.i2c_bus_num, str(e))
            rospy.logfatal("Make sure I2C is enabled: sudo raspi-config (Interface Options -> I2C)")
            sys.exit(1)
        
        # Initialize MPU9250
        if not self._init_mpu9250():
            rospy.logfatal("Failed to initialize MPU9250. Check wiring and I2C address.")
            sys.exit(1)
        
        # Initialize magnetometer (AK8963)
        self.mag_enabled = self._init_ak8963()
        if self.mag_enabled:
            rospy.loginfo("AK8963 magnetometer initialized successfully")
        else:
            rospy.logwarn("AK8963 magnetometer initialization failed - continuing without mag data")
        
        # Publishers
        # Publish raw IMU data for sensor fusion (e.g., imu_filter_madgwick)
        self.imu_pub = rospy.Publisher('imu/data_raw', Imu, queue_size=10)
        self.temp_pub = rospy.Publisher('imu/temperature', Temperature, queue_size=10)
        self.mag_pub = rospy.Publisher('imu/mag', MagneticField, queue_size=10)
        
        # Calibration (optional - can be done at startup or dynamically)
        self.accel_offset = [0.0, 0.0, 0.0]
        self.gyro_offset = [0.0, 0.0, 0.0]
        
        rospy.loginfo("MPU9250 node started. Publishing to /imu/data_raw at %d Hz", 
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
        try:
            # Read 2 bytes starting from addr (more efficient than two separate reads)
            data = self.bus.read_i2c_block_data(self.i2c_address, addr, 2)
            high = data[0]
            low = data[1]
            val = (high << 8) | low
            # Convert to signed 16-bit
            if val >= 0x8000:
                return -((65535 - val) + 1)
            else:
                return val
        except Exception as e:
            rospy.logwarn("Error reading word from 0x%02X: %s", addr, str(e))
            return 0
    
    def _read_accel_data(self):
        """
        Read accelerometer data.
        Returns (x, y, z) in m/s²
        ACCEL_XOUT_H is at register 0x3B
        """
        try:
            # Read all 6 accelerometer registers at once (0x3B-0x40)
            data = self.bus.read_i2c_block_data(int(self.i2c_address), int(self.ACCEL_XOUT_H), 6)
            accel_x = (data[0] << 8) | data[1]
            accel_y = (data[2] << 8) | data[3]
            accel_z = (data[4] << 8) | data[5]
            
            # Convert to signed 16-bit
            if accel_x >= 0x8000:
                accel_x = -((65535 - accel_x) + 1)
            if accel_y >= 0x8000:
                accel_y = -((65535 - accel_y) + 1)
            if accel_z >= 0x8000:
                accel_z = -((65535 - accel_z) + 1)
            
            # Scale for ±2g range (16384 LSB/g)
            accel_x = accel_x / 16384.0
            accel_y = accel_y / 16384.0
            accel_z = accel_z / 16384.0
            
            # Convert from g to m/s²
            accel_x *= 9.80665
            accel_y *= 9.80665
            accel_z *= 9.80665
            
            # Apply calibration offset
            accel_x -= self.accel_offset[0]
            accel_y -= self.accel_offset[1]
            accel_z -= self.accel_offset[2]
            
            return accel_x, accel_y, accel_z
        except Exception as e:
            rospy.logwarn("Error reading accelerometer: %s", str(e))
            return 0.0, 0.0, 0.0
    
    def _read_gyro_data(self):
        """
        Read gyroscope data.
        Returns (x, y, z) in rad/s
        GYRO_XOUT_H is at register 0x43
        """
        try:
            # Read all 6 gyroscope registers at once (0x43-0x48)
            data = self.bus.read_i2c_block_data(int(self.i2c_address), int(self.GYRO_XOUT_H), 6)
            gyro_x = (data[0] << 8) | data[1]
            gyro_y = (data[2] << 8) | data[3]
            gyro_z = (data[4] << 8) | data[5]
            
            # Convert to signed 16-bit
            if gyro_x >= 0x8000:
                gyro_x = -((65535 - gyro_x) + 1)
            if gyro_y >= 0x8000:
                gyro_y = -((65535 - gyro_y) + 1)
            if gyro_z >= 0x8000:
                gyro_z = -((65535 - gyro_z) + 1)
            
            # Scale for ±250°/s range (131 LSB/°/s)
            gyro_x = gyro_x / 131.0
            gyro_y = gyro_y / 131.0
            gyro_z = gyro_z / 131.0
            
            # Convert from °/s to rad/s
            gyro_x = math.radians(gyro_x)
            gyro_y = math.radians(gyro_y)
            gyro_z = math.radians(gyro_z)
            
            # Apply calibration offset
            gyro_x -= self.gyro_offset[0]
            gyro_y -= self.gyro_offset[1]
            gyro_z -= self.gyro_offset[2]
            
            return gyro_x, gyro_y, gyro_z
        except Exception as e:
            rospy.logwarn("Error reading gyroscope: %s", str(e))
            return 0.0, 0.0, 0.0
    
    def _read_temperature(self):
        """
        Read temperature data from MPU9250.
        Returns temperature in degrees Celsius.
        TEMP_OUT_H is at register 0x41
        """
        try:
            # Read 2 temperature registers (0x41-0x42)
            data = self.bus.read_i2c_block_data(int(self.i2c_address), int(self.TEMP_OUT_H), 2)
            temp_raw = (data[0] << 8) | data[1]
            
            # Convert to signed 16-bit
            if temp_raw >= 0x8000:
                temp_raw = -((65535 - temp_raw) + 1)
            
            # Convert to Celsius using MPU9250 formula:
            # Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/333.87 + 21.0
            temperature = (temp_raw / 333.87) + 21.0
            
            return temperature
        except Exception as e:
            rospy.logwarn("Error reading temperature: %s", str(e))
            return 0.0
    
    def _init_ak8963(self):
        """
        Initialize AK8963 magnetometer (inside MPU9250).
        Returns True if successful, False otherwise.
        """
        try:
            # Enable I2C master mode and connect to AK8963
            # Disable I2C master mode first
            self.bus.write_byte_data(self.i2c_address, self.USER_CTRL, 0x00)
            rospy.sleep(0.01)
            
            # Enable bypass mode to access AK8963 directly
            self.bus.write_byte_data(self.i2c_address, self.INT_PIN_CFG, 0x02)
            rospy.sleep(0.01)
            
            # Check AK8963 WHO_AM_I
            who_am_i = self.bus.read_byte_data(self.AK8963_ADDR, self.AK8963_WHO_AM_I)
            if who_am_i != 0x48:
                rospy.logwarn("AK8963 WHO_AM_I check failed: 0x%02X (expected 0x48)", who_am_i)
                return False
            
            # Power down magnetometer
            self.bus.write_byte_data(self.AK8963_ADDR, self.AK8963_CNTL1, 0x00)
            rospy.sleep(0.01)
            
            # Enter fuse ROM access mode to read sensitivity adjustment values
            self.bus.write_byte_data(self.AK8963_ADDR, self.AK8963_CNTL1, 0x0F)
            rospy.sleep(0.01)
            
            # Read sensitivity adjustment values
            asa_data = self.bus.read_i2c_block_data(self.AK8963_ADDR, self.AK8963_ASAX, 3)
            self.mag_sensitivity_x = (asa_data[0] - 128) / 256.0 + 1.0
            self.mag_sensitivity_y = (asa_data[1] - 128) / 256.0 + 1.0
            self.mag_sensitivity_z = (asa_data[2] - 128) / 256.0 + 1.0
            
            # Power down magnetometer
            self.bus.write_byte_data(self.AK8963_ADDR, self.AK8963_CNTL1, 0x00)
            rospy.sleep(0.01)
            
            # Set continuous measurement mode 2 (100Hz) with 16-bit output
            # Mode 0x16: Continuous measurement mode 2 (100 Hz) + 16-bit output
            self.bus.write_byte_data(self.AK8963_ADDR, self.AK8963_CNTL1, 0x16)
            rospy.sleep(0.01)
            
            rospy.loginfo("AK8963 sensitivity adjustments: X=%.3f, Y=%.3f, Z=%.3f",
                         self.mag_sensitivity_x, self.mag_sensitivity_y, self.mag_sensitivity_z)
            return True
            
        except Exception as e:
            rospy.logerr("Error initializing AK8963: %s", str(e))
            return False
    
    def _read_magnetometer(self):
        """
        Read magnetometer data from AK8963.
        Returns (x, y, z) in Tesla (T).
        """
        if not self.mag_enabled:
            return 0.0, 0.0, 0.0
        
        try:
            # Check if data is ready (ST1 register, bit 0)
            st1 = self.bus.read_byte_data(self.AK8963_ADDR, self.AK8963_ST1)
            if not (st1 & 0x01):
                # Data not ready
                return 0.0, 0.0, 0.0
            
            # Read all 7 bytes (6 data + 1 status)
            data = self.bus.read_i2c_block_data(self.AK8963_ADDR, self.AK8963_HXL, 7)
            
            # Check overflow flag in ST2 (bit 3)
            st2 = data[6]
            if st2 & 0x08:
                rospy.logwarn_throttle(1.0, "Magnetometer overflow detected")
                return 0.0, 0.0, 0.0
            
            # Combine high and low bytes (little-endian for AK8963)
            mag_x = (data[1] << 8) | data[0]
            mag_y = (data[3] << 8) | data[2]
            mag_z = (data[5] << 8) | data[4]
            
            # Convert to signed 16-bit
            if mag_x >= 0x8000:
                mag_x = -((65535 - mag_x) + 1)
            if mag_y >= 0x8000:
                mag_y = -((65535 - mag_y) + 1)
            if mag_z >= 0x8000:
                mag_z = -((65535 - mag_z) + 1)
            
            # Apply sensitivity adjustment
            mag_x = mag_x * self.mag_sensitivity_x
            mag_y = mag_y * self.mag_sensitivity_y
            mag_z = mag_z * self.mag_sensitivity_z
            
            # Convert from raw values to Tesla
            # 16-bit mode: 0.15 µT/LSB = 0.15e-6 T/LSB
            mag_x = mag_x * 0.15e-6
            mag_y = mag_y * 0.15e-6
            mag_z = mag_z * 0.15e-6
            
            return mag_x, mag_y, mag_z
            
        except Exception as e:
            rospy.logwarn("Error reading magnetometer: %s", str(e))
            return 0.0, 0.0, 0.0
    
    def _publish_imu(self):
        """
        Read IMU data and publish ROS messages.
        """
        try:
            # Get current timestamp for synchronization
            current_time = rospy.Time.now()
            
            # Read sensor data
            accel_x, accel_y, accel_z = self._read_accel_data()
            gyro_x, gyro_y, gyro_z = self._read_gyro_data()
            temperature = self._read_temperature()
            mag_x, mag_y, mag_z = self._read_magnetometer()
            
            # Create IMU message
            imu_msg = Imu()
            imu_msg.header = Header()
            imu_msg.header.stamp = current_time
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
            
            # Publish IMU message
            self.imu_pub.publish(imu_msg)
            
            # Create and publish Temperature message
            temp_msg = Temperature()
            temp_msg.header = Header()
            temp_msg.header.stamp = current_time
            temp_msg.header.frame_id = self.frame_id
            temp_msg.temperature = temperature
            temp_msg.variance = 0.5  # Temperature sensor variance (adjust as needed)
            self.temp_pub.publish(temp_msg)
            
            # Create and publish MagneticField message (if magnetometer is enabled)
            if self.mag_enabled:
                mag_msg = MagneticField()
                mag_msg.header = Header()
                mag_msg.header.stamp = current_time
                mag_msg.header.frame_id = self.frame_id
                mag_msg.magnetic_field.x = mag_x
                mag_msg.magnetic_field.y = mag_y
                mag_msg.magnetic_field.z = mag_z
                
                # Covariance (diagonal) - these should be calibrated
                mag_msg.magnetic_field_covariance[0] = 0.0001  # x variance in T²
                mag_msg.magnetic_field_covariance[4] = 0.0001  # y variance in T²
                mag_msg.magnetic_field_covariance[8] = 0.0001  # z variance in T²
                
                self.mag_pub.publish(mag_msg)
            
        except Exception as e:
            rospy.logerr("Error reading/publishing IMU data: %s", str(e))


if __name__ == '__main__':
    try:
        node = MPU9250Node()
    except rospy.ROSInterruptException:
        pass

