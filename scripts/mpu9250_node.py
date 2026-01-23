#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
MPU9250 IMU Driver Node

Publishes raw gyroscope and accelerometer data from MPU-9250 IMU sensor
connected to Jetson Nano via I2C. Magnetometer is disabled due to indoor
electromagnetic interference.

Published Topics:
    /imu/data_raw (sensor_msgs/Imu): Raw IMU data (no orientation)

Parameters:
    ~frame_id (str): TF frame for IMU (default: "imu_link")
    ~publish_rate (float): Publishing frequency in Hz (default: 50.0)
    ~i2c_bus (int): I2C bus number (default: 1)
    ~i2c_address (str/int): MPU9250 I2C address (default: "0x68")

Hardware:
    - MPU9250 SDA → Jetson Pin 3 (I2C1 SDA)
    - MPU9250 SCL → Jetson Pin 5 (I2C1 SCL)
    - MPU9250 VCC → Jetson 3.3V (Pin 1)
    - MPU9250 GND → Jetson GND (Pin 6, 9, 14, or 20)

Dependencies:
    - smbus or smbus2 (sudo apt-get install python3-smbus)
    - i2c-tools (sudo apt-get install i2c-tools)

Note:
    Robot must be stationary during startup for gyroscope calibration.
    Magnetometer is disabled - see TF_OWNERSHIP_AND_GUARDRAILS.md for details.
"""

import rospy
import sensor_msgs.msg
from sensor_msgs.msg import Imu
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
    Only uses gyroscope + accelerometer (magnetometer disabled - see header).
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
        
        rospy.loginfo("MPU9250 initialized (Gyro + Accel only, Magnetometer DISABLED)")
        
        # Publishers
        # Publish raw IMU data for sensor fusion (e.g., imu_filter_madgwick)
        self.imu_pub = rospy.Publisher('imu/data_raw', Imu, queue_size=10)
        
        # Calibration offsets - will be set dynamically on startup
        # Gyroscope static drift (rad/s) - subtracted from raw readings
        self.gyro_offset = [0.0, 0.0, 0.0]
        
        # Accelerometer offset (m/s²) - can be adjusted if needed
        self.accel_offset = [0.0, 0.0, 0.0]
        
        # Perform dynamic gyroscope calibration
        rospy.loginfo("Starting dynamic gyroscope calibration (100 samples)...")
        if self._calibrate_gyro():
            rospy.loginfo("Gyro calibration complete: X=%.4f, Y=%.4f, Z=%.4f rad/s", 
                         self.gyro_offset[0], self.gyro_offset[1], self.gyro_offset[2])
        else:
            rospy.logwarn("Gyro calibration failed - using zero offsets")
        
        # Wait for ROS time to be initialized (prevents TF extrapolation errors)
        rospy.loginfo("Waiting for ROS time to be initialized...")
        while rospy.Time.now().to_sec() == 0.0 and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo("ROS time initialized")
        
        rospy.loginfo("MPU9250 node started. Publishing to /imu/data_raw at %d Hz", 
                     int(self.publish_rate))
        rospy.loginfo("Magnetometer DISABLED (indoor EMI protection)")
        
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
    
    def _calibrate_gyro(self):
        """
        Perform dynamic gyroscope calibration on startup.
        Reads 100 samples and averages them to determine static drift offset.
        Robot MUST be stationary during this process.
        Returns True if successful, False otherwise.
        """
        try:
            rospy.loginfo("Keep robot STATIONARY for gyroscope calibration...")
            rospy.sleep(0.5)  # Give user time to see message
            
            num_samples = 100
            gyro_sum = [0.0, 0.0, 0.0]
            valid_samples = 0
            
            for i in range(num_samples):
                # Read raw gyro data without offset
                gyro_x, gyro_y, gyro_z = self._read_gyro_raw()
                
                # Check if reading is valid (not all zeros)
                if abs(gyro_x) > 0.0001 or abs(gyro_y) > 0.0001 or abs(gyro_z) > 0.0001 or valid_samples > 0:
                    gyro_sum[0] += gyro_x
                    gyro_sum[1] += gyro_y
                    gyro_sum[2] += gyro_z
                    valid_samples += 1
                
                rospy.sleep(0.01)  # 10ms between samples (~100Hz)
            
            if valid_samples < 50:
                rospy.logerr("Gyro calibration failed: insufficient valid samples (%d)", valid_samples)
                return False
            
            # Calculate average (this is the static drift)
            self.gyro_offset[0] = gyro_sum[0] / valid_samples
            self.gyro_offset[1] = gyro_sum[1] / valid_samples
            self.gyro_offset[2] = gyro_sum[2] / valid_samples
            
            rospy.loginfo("Calibration successful with %d samples", valid_samples)
            return True
            
        except Exception as e:
            rospy.logerr("Error during gyro calibration: %s", str(e))
            return False
    
    def _read_gyro_raw(self):
        """
        Read raw gyroscope data WITHOUT applying calibration offset.
        Used during calibration process.
        Returns (x, y, z) in rad/s
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
            
            return gyro_x, gyro_y, gyro_z
        except Exception as e:
            return 0.0, 0.0, 0.0
    
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
            
            # CRITICAL: Remap from MPU9250 NED to ROS ENU convention
            # NED: X=forward, Y=right, Z=down
            # ENU: X=forward, Y=left, Z=up
            # Transform: X_enu=X_ned, Y_enu=-Y_ned, Z_enu=-Z_ned
            accel_y = -accel_y
            accel_z = -accel_z
            
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
            
            # COORDINATE FRAME FIX: Keep gyro_z positive for CCW rotation
            # Robot's IMU is mounted aligned with base_link (Z-up, right-hand rule)
            # Positive gyro_z = counter-clockwise rotation (correct for ROS REP-103)
            gyro_y = -gyro_y  # Y-axis may need inversion depending on mount
            # gyro_z inversion REMOVED - was causing wrong rotation direction
            
            return gyro_x, gyro_y, gyro_z
        except Exception as e:
            rospy.logwarn("Error reading gyroscope: %s", str(e))
            return 0.0, 0.0, 0.0
    
    def _read_magnetometer(self):
        """
        Magnetometer DISABLED (returns zeros).
        Reason: Indoor EMI from motors/PSU/battery corrupts readings.
        See TF_OWNERSHIP_AND_GUARDRAILS.md for full explanation.
        """
        return 0.0, 0.0, 0.0
    
    def _publish_imu(self):
        """
        Read IMU data and publish ROS messages.
        """
        try:
            # Get current timestamp for synchronization
            current_time = rospy.Time.now()
            
            # Read sensor data (gyro + accel only)
            accel_x, accel_y, accel_z = self._read_accel_data()
            gyro_x, gyro_y, gyro_z = self._read_gyro_data()
            
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
            
            # Orientation covariance - small positive diagonal keeps filter stable
            # Diagonal values signal low uncertainty for a well-calibrated sensor
            imu_msg.orientation_covariance[0] = 0.01  # x
            imu_msg.orientation_covariance[4] = 0.01  # y
            imu_msg.orientation_covariance[8] = 0.01  # z
            
            # Publish IMU message
            self.imu_pub.publish(imu_msg)
            
        except Exception as e:
            rospy.logerr("Error reading/publishing IMU data: %s", str(e))


if __name__ == '__main__':
    try:
        node = MPU9250Node()
    except rospy.ROSInterruptException:
        pass

