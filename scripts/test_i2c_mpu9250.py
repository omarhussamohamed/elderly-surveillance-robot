#!/usr/bin/env python3
"""
Quick I2C test script for MPU9250
Run this to verify MPU9250 is detected on I2C bus before using ROS node
"""

import sys

try:
    try:
        import smbus2 as smbus
        print("Using smbus2")
    except ImportError:
        import smbus
        print("Using smbus")
except ImportError:
    print("ERROR: smbus not installed")
    print("Install with: sudo apt-get install python3-smbus i2c-tools")
    sys.exit(1)

# MPU9250 I2C address
MPU9250_ADDR = 0x68  # Default (0x69 if AD0 pin is HIGH)

# MPU9250 Registers
PWR_MGMT_1 = 0x6B
WHO_AM_I = 0x75

print("Testing MPU9250 I2C connection...")
print("")

# Test both I2C buses
for bus_num in [1, 2]:
    try:
        print(f"Testing I2C bus {bus_num}...")
        bus = smbus.SMBus(bus_num)
        
        # Wake up MPU9250
        bus.write_byte_data(MPU9250_ADDR, PWR_MGMT_1, 0x00)
        
        # Read WHO_AM_I register
        who_am_i = bus.read_byte_data(MPU9250_ADDR, WHO_AM_I)
        
        print(f"  I2C bus {bus_num}: Device found at 0x{MPU9250_ADDR:02X}")
        print(f"  WHO_AM_I register: 0x{who_am_i:02X}")
        
        if who_am_i in [0x71, 0x73, 0x70]:
            print(f"  ✓ MPU9250 detected on bus {bus_num}!")
            print(f"  Use i2c_bus={bus_num} in launch file")
            bus.close()
            sys.exit(0)
        else:
            print(f"  ✗ Unexpected WHO_AM_I value (expected 0x71/0x73/0x70)")
        
        bus.close()
        
    except IOError as e:
        print(f"  I2C bus {bus_num}: No device at 0x{MPU9250_ADDR:02X}")
    except Exception as e:
        print(f"  I2C bus {bus_num}: Error - {e}")

print("")
print("MPU9250 not found on any I2C bus!")
print("Check wiring:")
print("  - SDA connected?")
print("  - SCL connected?")
print("  - VCC = 3.3V?")
print("  - GND connected?")
print("  - I2C enabled? (sudo raspi-config)")
print("")
print("Scan I2C buses:")
print("  sudo i2cdetect -y 1")
print("  sudo i2cdetect -y 2")
sys.exit(1)

