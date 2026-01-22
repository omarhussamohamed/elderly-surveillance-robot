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
  pip3 install jetson-stats Jetson.GPIO adafruit-blinka adafruit-circuitpython-ads1x15
  
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

# ADC for gas sensor
ADS_AVAILABLE = False
try:
    import board
    import busio
    import adafruit_ads1x15.ads1115 as ADS
    from adafruit_ads1x15.analog_in import AnalogIn
    ADS_AVAILABLE = True
except ImportError as e:
    ADS = None
    # Will log warning in __init__ when enable_gas_sensor=True

# Jetson stats
JTOP_AVAILABLE = False
try:
    from jtop import jtop
    JTOP_AVAILABLE = True
except ImportError as e:
    jtop = None
    # Will log warning in __init__ when enable_stats=True


class SensorsActuatorsNode:
    """
    Main node class handling all sensors and actuators with individual enable flags.
    Each feature can fail independently without crashing the node.
    """
    
    def __init__(self):
        rospy.init_node('sensors_actuators_node', anonymous=False)
        rospy.loginfo("=== Sensors & Actuators Node Starting ===")
        
        # === PARAMETERS (all with safe defaults) ===
        self.enable_gas_sensor = rospy.get_param('~enable_gas_sensor', False)
        self.enable_buzzer = rospy.get_param('~enable_buzzer', False)
        self.enable_stats = rospy.get_param('~enable_stats', True)
        
        self.gas_threshold_voltage = rospy.get_param('~gas_threshold_voltage', 1.0)
        self.buzzer_pin = rospy.get_param('~buzzer_pin', 0)  # 0 means disabled
        self.adc_i2c_address = rospy.get_param('~adc_i2c_address', 0x48)
        self.adc_channel = rospy.get_param('~adc_channel', 0)  # A0=0, A1=1, etc.
        self.adc_gain = rospy.get_param('~adc_gain', 1)
        self.publish_rate = rospy.get_param('~publish_rate', 1.0)  # Hz
        
        # === HARDWARE STATE ===
        self.gas_sensor_active = False
        self.buzzer_active = False
        self.stats_active = False
        
        self.ads = None
        self.adc_channel_obj = None
        self.jetson = None
        
        # Buzzer state and timeout
        self.buzzer_state = False
        self.last_buzzer_cmd_time = None
        self.buzzer_timeout = 5.0  # seconds
        self.buzzer_lock = threading.Lock()
        
        # === PUBLISHERS (always create, but only publish if enabled) ===
        self.pub_gas_level = rospy.Publisher('/gas_level', Float32, queue_size=10)
        self.pub_gas_detected = rospy.Publisher('/gas_detected', Bool, queue_size=10)
        self.pub_temperature = rospy.Publisher('/jetson_temperature', Temperature, queue_size=10)
        self.pub_power = rospy.Publisher('/jetson_power', Float32, queue_size=10)
        
        # === SUBSCRIBERS ===
        self.sub_buzzer = rospy.Subscriber('/buzzer_command', Bool, self.buzzer_callback, queue_size=1)
        
        # === INITIALIZE HARDWARE ===
        self._init_gas_sensor()
        self._init_buzzer()
        self._init_jetson_stats()
        
        rospy.loginfo("=== Sensors & Actuators Node Initialized ===")
        rospy.loginfo(f"Gas Sensor: {'ACTIVE' if self.gas_sensor_active else 'DISABLED'}")
        rospy.loginfo(f"Buzzer: {'ACTIVE' if self.buzzer_active else 'DISABLED'}")
        rospy.loginfo(f"Jetson Stats: {'ACTIVE' if self.stats_active else 'DISABLED'}")
        
        # === SHUTDOWN HOOK ===
        rospy.on_shutdown(self.shutdown_hook)
    
    def _init_gas_sensor(self):
        """Initialize MQ-6 gas sensor via ADS1115 I2C ADC."""
        if not self.enable_gas_sensor:
            rospy.loginfo("Gas sensor disabled by parameter")
            return
        
        if not ADS_AVAILABLE:
            rospy.logwarn("Gas sensor disabled: Required libraries not available (adafruit-ads1x15)")
            rospy.logwarn("Install: pip3 install adafruit-blinka adafruit-circuitpython-ads1x15")
            return
        
        try:
            # Initialize I2C bus
            i2c = busio.I2C(board.SCL, board.SDA)
            
            # Create ADS1115 object with specified address
            self.ads = ADS.ADS1115(i2c, address=self.adc_i2c_address)
            
            # Set gain (1 = +/-4.096V range, suitable for 3.3V sensors)
            self.ads.gain = self.adc_gain
            
            # Create analog input channel
            if self.adc_channel == 0:
                self.adc_channel_obj = AnalogIn(self.ads, ADS.P0)
            elif self.adc_channel == 1:
                self.adc_channel_obj = AnalogIn(self.ads, ADS.P1)
            elif self.adc_channel == 2:
                self.adc_channel_obj = AnalogIn(self.ads, ADS.P2)
            elif self.adc_channel == 3:
                self.adc_channel_obj = AnalogIn(self.ads, ADS.P3)
            else:
                raise ValueError(f"Invalid ADC channel: {self.adc_channel}")
            
            # Test read
            test_voltage = self.adc_channel_obj.voltage
            rospy.loginfo(f"Gas sensor initialized: ADS1115 at 0x{self.adc_i2c_address:02X}, Channel A{self.adc_channel}")
            rospy.loginfo(f"Initial gas sensor reading: {test_voltage:.3f}V")
            
            self.gas_sensor_active = True
            
        except Exception as e:
            rospy.logwarn(f"Gas sensor disabled: Failed to initialize ADS1115: {e}")
            rospy.logwarn("Check I2C connections and address (default 0x48)")
            self.ads = None
            self.adc_channel_obj = None
    
    def _init_buzzer(self):
        """Initialize active buzzer on Jetson GPIO pin."""
        if not self.enable_buzzer:
            rospy.loginfo("Buzzer disabled by parameter")
            return
        
        if self.buzzer_pin <= 0:
            rospy.logwarn("Buzzer disabled: Invalid pin number (must be > 0)")
            return
        
        if not GPIO_AVAILABLE:
            rospy.logwarn("Buzzer disabled: Jetson.GPIO library not available")
            rospy.logwarn("Install: pip3 install Jetson.GPIO")
            return
        
        try:
            # Set GPIO mode to BOARD numbering
            GPIO.setmode(GPIO.BOARD)
            
            # Configure buzzer pin as output, default LOW (buzzer OFF)
            GPIO.setup(self.buzzer_pin, GPIO.OUT, initial=GPIO.LOW)
            
            rospy.loginfo(f"Buzzer initialized on GPIO pin {self.buzzer_pin} (BOARD numbering)")
            rospy.loginfo("Buzzer state: OFF (safe default)")
            
            self.buzzer_active = True
            self.buzzer_state = False
            
        except Exception as e:
            rospy.logwarn(f"Buzzer disabled: Failed to initialize GPIO: {e}")
            rospy.logwarn(f"Check pin {self.buzzer_pin} is valid and not in use")
    
    def _init_jetson_stats(self):
        """Initialize Jetson stats monitoring via jtop."""
        if not self.enable_stats:
            rospy.loginfo("Jetson stats disabled by parameter")
            return
        
        if not JTOP_AVAILABLE:
            rospy.logwarn("Jetson stats disabled: jetson-stats library not available")
            rospy.logwarn("Install: pip3 install jetson-stats (then reboot)")
            return
        
        try:
            # jtop will be opened in the read function (context manager)
            # Just verify it's importable here
            rospy.loginfo("Jetson stats initialized (jtop available)")
            self.stats_active = True
            
        except Exception as e:
            rospy.logwarn(f"Jetson stats disabled: {e}")
    
    def read_gas_sensor(self):
        """
        Read gas sensor voltage from ADC.
        Returns (voltage, detected) tuple or (0.0, False) on failure.
        
        NOTE: Raw voltage is published. To convert to PPM (parts per million),
        you need to calibrate with known gas concentrations. Typical MQ-6:
        - Clean air: ~0.4-0.6V
        - LPG/Propane present: voltage increases
        - Threshold example: >1.0V might indicate gas presence
        """
        if not self.gas_sensor_active or self.adc_channel_obj is None:
            return 0.0, False
        
        try:
            voltage = self.adc_channel_obj.voltage
            detected = voltage > self.gas_threshold_voltage
            return voltage, detected
            
        except Exception as e:
            rospy.logwarn_throttle(10, f"Failed to read gas sensor: {e}")
            return 0.0, False
    
    def read_jetson_stats(self):
        """
        Read Jetson temperature and power consumption.
        Returns (temperature_C, power_W) tuple or (0.0, 0.0) on failure.
        """
        if not self.stats_active:
            return 0.0, 0.0
        
        try:
            with jtop() as jetson:
                # Wait for jtop to initialize
                jetson.ok()
                
                # Read temperature (various sensors available)
                # Try GPU temp first (most representative), fallback to others
                temp_C = 0.0
                if hasattr(jetson, 'temperature') and jetson.temperature:
                    temp_dict = jetson.temperature
                    if 'GPU' in temp_dict:
                        temp_C = temp_dict['GPU']
                    elif 'CPU' in temp_dict:
                        temp_C = temp_dict['CPU']
                    elif 'thermal' in temp_dict:
                        temp_C = temp_dict['thermal']
                    else:
                        # Average all available temps
                        temps = [v for v in temp_dict.values() if isinstance(v, (int, float))]
                        temp_C = sum(temps) / len(temps) if temps else 0.0
                
                # Read total power consumption
                power_W = 0.0
                if hasattr(jetson, 'power') and jetson.power:
                    power_dict = jetson.power
                    # Try 'tot' or 'total' keys
                    if 'tot' in power_dict:
                        power_W = power_dict['tot'] / 1000.0  # Convert mW to W
                    elif 'total' in power_dict:
                        power_W = power_dict['total'] / 1000.0
                    else:
                        # Sum all rail powers
                        power_W = sum(v for v in power_dict.values() if isinstance(v, (int, float))) / 1000.0
                
                return temp_C, power_W
                
        except Exception as e:
            rospy.logwarn_throttle(10, f"Failed to read Jetson stats: {e}")
            return 0.0, 0.0
    
    def set_buzzer(self, state):
        """
        Set buzzer state (True=ON, False=OFF).
        Thread-safe with automatic safety timeout.
        """
        if not self.buzzer_active:
            return
        
        with self.buzzer_lock:
            try:
                if state:
                    GPIO.output(self.buzzer_pin, GPIO.HIGH)
                    rospy.loginfo("Buzzer: ON")
                else:
                    GPIO.output(self.buzzer_pin, GPIO.LOW)
                    rospy.loginfo("Buzzer: OFF")
                
                self.buzzer_state = state
                
            except Exception as e:
                rospy.logwarn(f"Failed to set buzzer: {e}")
                # On error, try to turn off
                try:
                    GPIO.output(self.buzzer_pin, GPIO.LOW)
                except:
                    pass
                self.buzzer_state = False
    
    def buzzer_callback(self, msg):
        """Handle buzzer command messages with timeout tracking."""
        self.last_buzzer_cmd_time = rospy.Time.now()
        self.set_buzzer(msg.data)
    
    def check_buzzer_timeout(self):
        """
        Safety feature: Auto-shutoff buzzer if no command received for >5 seconds.
        Prevents stuck-on buzzer in case of communication failure.
        """
        if not self.buzzer_active or not self.buzzer_state:
            return
        
        if self.last_buzzer_cmd_time is None:
            return
        
        time_since_cmd = (rospy.Time.now() - self.last_buzzer_cmd_time).to_sec()
        if time_since_cmd > self.buzzer_timeout:
            rospy.logwarn(f"Buzzer timeout: No command for {time_since_cmd:.1f}s, turning OFF")
            self.set_buzzer(False)
            self.last_buzzer_cmd_time = None  # Reset to avoid repeated warnings
    
    def publish_data(self):
        """Publish all enabled sensor data at configured rate."""
        
        # === GAS SENSOR ===
        if self.gas_sensor_active:
            voltage, detected = self.read_gas_sensor()
            
            self.pub_gas_level.publish(Float32(voltage))
            self.pub_gas_detected.publish(Bool(detected))
            
            if detected:
                rospy.loginfo_throttle(5, f"GAS DETECTED! Level: {voltage:.3f}V (threshold: {self.gas_threshold_voltage}V)")
        
        # === JETSON STATS ===
        if self.stats_active:
            temp_C, power_W = self.read_jetson_stats()
            
            # Publish temperature as sensor_msgs/Temperature
            temp_msg = Temperature()
            temp_msg.header.stamp = rospy.Time.now()
            temp_msg.header.frame_id = "jetson_board"
            temp_msg.temperature = temp_C
            temp_msg.variance = 0.0  # Unknown variance
            self.pub_temperature.publish(temp_msg)
            
            # Publish power as simple Float32
            self.pub_power.publish(Float32(power_W))
        
        # === BUZZER TIMEOUT CHECK ===
        if self.buzzer_active:
            self.check_buzzer_timeout()
    
    def run(self):
        """Main loop: publish sensor data at configured rate."""
        rate = rospy.Rate(self.publish_rate)
        
        rospy.loginfo(f"Main loop starting at {self.publish_rate} Hz")
        
        while not rospy.is_shutdown():
            try:
                self.publish_data()
                rate.sleep()
                
            except rospy.ROSInterruptException:
                break
            except Exception as e:
                rospy.logerr(f"Error in main loop: {e}")
                rospy.sleep(1.0)  # Prevent rapid error spam
    
    def shutdown_hook(self):
        """Clean shutdown: turn off buzzer and cleanup GPIO."""
        rospy.loginfo("=== Shutting down Sensors & Actuators Node ===")
        
        # Turn off buzzer
        if self.buzzer_active:
            try:
                self.set_buzzer(False)
                rospy.loginfo("Buzzer turned OFF")
            except Exception as e:
                rospy.logwarn(f"Failed to turn off buzzer on shutdown: {e}")
        
        # Cleanup GPIO
        if GPIO_AVAILABLE and self.buzzer_active:
            try:
                GPIO.cleanup()
                rospy.loginfo("GPIO cleaned up")
            except Exception as e:
                rospy.logwarn(f"Failed to cleanup GPIO: {e}")
        
        rospy.loginfo("Shutdown complete")


if __name__ == '__main__':
    try:
        node = SensorsActuatorsNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Fatal error: {e}")
