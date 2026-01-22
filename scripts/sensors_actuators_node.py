#!/usr/bin/env python3
"""
Sensors and Actuators Node for Elderly Bot
===========================================
Handles MQ-6 gas sensor (via I2C ADC), active buzzer (GPIO), and Jetson monitoring.

SAFETY FEATURES:
- Runs without crashing even if hardware is missing or libraries fail to import
- All features individually enable/disable via parameters (default: False until hardware ready)
- Publishes safe default values when sensors unavailable
- Buzzer defaults OFF on startup and on any error
- Auto-shutoff buzzer if no command received for >5 seconds
- Extensive error handling with clear log messages

HARDWARE:
- MQ-6 Gas Sensor: Analog output → ADS1115 I2C ADC (16-bit, 0x48) → Channel A0
- Active Buzzer: Digital output → Jetson GPIO pin (BOARD numbering)
- Jetson Stats: jtop library for temperature/power monitoring

DEPENDENCIES (install when hardware ready):
  sudo apt install python3-pip
  pip3 install jetson-stats Jetson.GPIO smbus2
  
TO ENABLE: Set parameters in config/sensors_actuators.yaml and launch with sensors_actuators:=true
"""

import rospy
import time
import threading
from std_msgs.msg import Float32, Bool, Float32MultiArray
from sensor_msgs.msg import Temperature

# === HARDWARE IMPORTS (with graceful failure) ===

# GPIO for buzzer
GPIO_AVAILABLE = False
try:
    import Jetson.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError as e:
    GPIO = None
    # Will log warning in __init__ when enable_buzzer=True

# I2C for gas sensor (ADS1115)
ADS_AVAILABLE = False
try:
    import smbus2
    ADS_AVAILABLE = True
except ImportError as e:
    smbus2 = None
    # Will log warning in __init__ when enable_gas_sensor=True

# Jetson stats
JTOP_AVAILABLE = False
try:
    from jtop import jtop
    JTOP_AVAILABLE = True
except ImportError as e:
    jtop = None
    # Will log warning in __init__ when enable_jetson_stats=True


class SensorsActuatorsNode:
    """
    ROS node that manages:
    1. Gas sensor reading (MQ-6 via ADS1115 I2C ADC)
    2. Buzzer control (GPIO digital output with safety timeout)
    3. Jetson monitoring (temperature, power via jtop)
    
    Designed for graceful degradation - each feature can fail independently.
    """
    
    def __init__(self):
        rospy.init_node('sensors_actuators_node', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        
        # === PARAMETERS ===
        self.enable_gas_sensor = rospy.get_param('~enable_gas_sensor', False)
        self.enable_buzzer = rospy.get_param('~enable_buzzer', False)
        self.enable_jetson_stats = rospy.get_param('~enable_jetson_stats', True)
        
        self.gas_threshold_voltage = rospy.get_param('~gas_threshold_voltage', 1.0)
        self.buzzer_pin = rospy.get_param('~buzzer_pin', 0)  # BOARD numbering
        self.adc_i2c_address = rospy.get_param('~adc_i2c_address', 0x48)
        self.adc_i2c_bus = rospy.get_param('~adc_i2c_bus', 1)
        self.publish_rate = rospy.get_param('~publish_rate', 1.0)
        
        # === HARDWARE STATE ===
        self.i2c_bus = None  # smbus2.SMBus instance
        self.buzzer_initialized = False
        self.jtop_handle = None
        
        # Buzzer safety
        self.buzzer_lock = threading.Lock()
        self.last_buzzer_command_time = 0.0
        self.buzzer_timeout_seconds = 5.0
        
        # ADS1115 registers
        self.ADS1115_REG_CONVERSION = 0x00
        self.ADS1115_REG_CONFIG = 0x01
        
        # === INITIALIZE HARDWARE ===
        if self.enable_gas_sensor:
            self._init_gas_sensor()
        
        if self.enable_buzzer:
            self._init_buzzer()
        
        if self.enable_jetson_stats:
            self._init_jetson_stats()
        
        # === ROS INTERFACE ===
        # Publishers (always create - publish safe defaults if hardware unavailable)
        self.gas_voltage_pub = rospy.Publisher('/gas_level', Float32, queue_size=1)
        self.gas_detected_pub = rospy.Publisher('/gas_detected', Bool, queue_size=1)
        self.jetson_temp_pub = rospy.Publisher('/jetson_temperature', Temperature, queue_size=1)
        self.jetson_power_pub = rospy.Publisher('/jetson_power', Float32, queue_size=1)
        
        # Subscribers
        rospy.Subscriber('/buzzer_command', Bool, self.buzzer_command_callback)
        
        rospy.loginfo("=== Sensors & Actuators Node Initialized ===")
        rospy.loginfo(f"  Gas sensor (MQ-6): {'ENABLED' if self.i2c_bus is not None else 'DISABLED'}")
        rospy.loginfo(f"  Buzzer: {'ENABLED' if self.buzzer_initialized else 'DISABLED'}")
        rospy.loginfo(f"  Jetson stats: {'ENABLED' if self.jtop_handle is not None else 'DISABLED'}")
        rospy.loginfo(f"  Publish rate: {self.publish_rate} Hz")
    
    def _init_gas_sensor(self):
        """Initialize ADS1115 ADC via I2C for gas sensor."""
        if not ADS_AVAILABLE:
            rospy.logwarn("Gas sensor: smbus2 library not available")
            rospy.logwarn("  Install: sudo pip3 install smbus2")
            return
        
        try:
            # Open I2C bus
            self.i2c_bus = smbus2.SMBus(self.adc_i2c_bus)
            
            # Configure ADS1115 for single-shot mode
            # Config register: OS=1 (start), MUX=100 (AIN0), PGA=001 (±4.096V), MODE=1 (single-shot)
            # DR=100 (128 SPS), rest=0
            # Binary: 1100 0001 1000 0011 = 0xC183
            config = 0xC183
            
            # Write config (swap bytes for I2C word format)
            config_bytes = [(config >> 8) & 0xFF, config & 0xFF]
            self.i2c_bus.write_i2c_block_data(self.adc_i2c_address, self.ADS1115_REG_CONFIG, config_bytes)
            
            rospy.loginfo(f"Gas sensor initialized: I2C bus {self.adc_i2c_bus}, addr 0x{self.adc_i2c_address:02X}")
            
        except Exception as e:
            rospy.logerr(f"Failed to initialize gas sensor: {e}")
            rospy.logerr(f"  Check I2C: sudo i2cdetect -y -r {self.adc_i2c_bus}")
            if self.i2c_bus is not None:
                try:
                    self.i2c_bus.close()
                except:
                    pass
            self.i2c_bus = None
    
    def _init_buzzer(self):
        """Initialize GPIO for buzzer control."""
        if not GPIO_AVAILABLE:
            rospy.logwarn("Buzzer: Jetson.GPIO library not available")
            rospy.logwarn("  Install: sudo pip3 install Jetson.GPIO")
            return
        
        if self.buzzer_pin == 0:
            rospy.logwarn("Buzzer: pin not configured (buzzer_pin=0)")
            return
        
        try:
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(self.buzzer_pin, GPIO.OUT, initial=GPIO.LOW)
            self.buzzer_initialized = True
            rospy.loginfo(f"Buzzer initialized: GPIO pin {self.buzzer_pin} (BOARD)")
        except Exception as e:
            rospy.logerr(f"Failed to initialize buzzer: {e}")
            self.buzzer_initialized = False
    
    def _init_jetson_stats(self):
        """Initialize jtop for Jetson monitoring."""
        if not JTOP_AVAILABLE:
            rospy.logwarn("Jetson stats: jtop library not available")
            rospy.logwarn("  Install: sudo pip3 install jetson-stats && sudo reboot")
            return
        
        try:
            self.jtop_handle = jtop()
            self.jtop_handle.start()
            rospy.loginfo("Jetson stats initialized")
        except Exception as e:
            rospy.logerr(f"Failed to initialize Jetson stats: {e}")
            rospy.logerr("  Did you reboot after installing jetson-stats?")
            self.jtop_handle = None
    
    def read_gas_sensor(self):
        """
        Read gas sensor voltage from ADS1115.
        Returns: (voltage, detected) tuple. Safe defaults (0.0, False) if hardware unavailable.
        """
        if self.i2c_bus is None:
            return (0.0, False)
        
        try:
            # Start conversion: write config with OS bit set
            config = 0xC183
            config_bytes = [(config >> 8) & 0xFF, config & 0xFF]
            self.i2c_bus.write_i2c_block_data(self.adc_i2c_address, self.ADS1115_REG_CONFIG, config_bytes)
            
            # Wait for conversion (8ms for 128 SPS)
            time.sleep(0.01)
            
            # Read conversion result (16-bit signed)
            data = self.i2c_bus.read_i2c_block_data(self.adc_i2c_address, self.ADS1115_REG_CONVERSION, 2)
            raw_adc = (data[0] << 8) | data[1]
            
            # Convert to signed int16
            if raw_adc > 32767:
                raw_adc -= 65536
            
            # Convert to voltage: FSR = ±4.096V, 15-bit resolution (sign bit)
            voltage = raw_adc * 4.096 / 32768.0
            
            # Gas sensor outputs positive voltage
            voltage = abs(voltage)
            
            detected = voltage > self.gas_threshold_voltage
            return (voltage, detected)
            
        except Exception as e:
            rospy.logwarn_throttle(10.0, f"Gas sensor read error: {e}")
            return (0.0, False)
    
    def set_buzzer(self, state):
        """
        Set buzzer state (True=ON, False=OFF).
        Thread-safe. No-op if buzzer not initialized.
        """
        if not self.buzzer_initialized:
            return
        
        with self.buzzer_lock:
            try:
                GPIO.output(self.buzzer_pin, GPIO.HIGH if state else GPIO.LOW)
            except Exception as e:
                rospy.logerr_throttle(5.0, f"Buzzer control error: {e}")
    
    def buzzer_command_callback(self, msg):
        """
        Handle buzzer commands from /buzzer_command topic.
        Safety: auto-shutoff if no command received for >5 seconds.
        """
        if not self.buzzer_initialized:
            return
        
        self.last_buzzer_command_time = rospy.get_time()
        self.set_buzzer(msg.data)
        
        if msg.data:
            rospy.loginfo("Buzzer ON")
        else:
            rospy.loginfo("Buzzer OFF")
    
    def check_buzzer_timeout(self):
        """Auto-shutoff buzzer if no command received for >5 seconds."""
        if not self.buzzer_initialized:
            return
        
        if self.last_buzzer_command_time == 0.0:
            return  # No commands yet
        
        elapsed = rospy.get_time() - self.last_buzzer_command_time
        if elapsed > self.buzzer_timeout_seconds:
            self.set_buzzer(False)
            rospy.logwarn_throttle(10.0, f"Buzzer auto-shutoff (no command for {elapsed:.1f}s)")
            self.last_buzzer_command_time = 0.0  # Reset
    
    def read_jetson_stats(self):
        """
        Read Jetson temperature and power.
        Returns: (temperature_C, power_W) tuple. Safe defaults (0.0, 0.0) if unavailable.
        """
        if self.jtop_handle is None:
            return (0.0, 0.0)
        
        try:
            # Get temperature (average of all thermal zones)
            temps = self.jtop_handle.temperature
            if temps:
                avg_temp = sum(temps.values()) / len(temps)
            else:
                avg_temp = 0.0
            
            # Get power (total)
            power = self.jtop_handle.power.get('total', 0.0)
            
            return (avg_temp, power)
            
        except Exception as e:
            rospy.logwarn_throttle(10.0, f"Jetson stats read error: {e}")
            return (0.0, 0.0)
    
    def publish_sensor_data(self):
        """Publish all sensor data to ROS topics."""
        # Gas sensor
        voltage, detected = self.read_gas_sensor()
        self.gas_voltage_pub.publish(Float32(data=voltage))
        self.gas_detected_pub.publish(Bool(data=detected))
        
        # Jetson stats
        temp, power = self.read_jetson_stats()
        temp_msg = Temperature()
        temp_msg.header.stamp = rospy.Time.now()
        temp_msg.temperature = temp
        self.jetson_temp_pub.publish(temp_msg)
        self.jetson_power_pub.publish(Float32(data=power))
    
    def run(self):
        """Main loop."""
        rate = rospy.Rate(self.publish_rate)
        
        while not rospy.is_shutdown():
            self.publish_sensor_data()
            self.check_buzzer_timeout()
            rate.sleep()
    
    def shutdown(self):
        """Clean shutdown: turn off buzzer, close I2C, close jtop."""
        rospy.loginfo("Shutting down sensors & actuators node...")
        
        # Turn off buzzer
        if self.buzzer_initialized:
            try:
                self.set_buzzer(False)
                GPIO.cleanup()
            except:
                pass
        
        # Close I2C
        if self.i2c_bus is not None:
            try:
                self.i2c_bus.close()
            except:
                pass
        
        # Close jtop
        if self.jtop_handle is not None:
            try:
                self.jtop_handle.close()
            except:
                pass


if __name__ == '__main__':
    try:
        node = SensorsActuatorsNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
