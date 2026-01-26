#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Sensors and Actuators Node - USING JETSON-STATS
"""

import rospy
import time
import threading
import traceback
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Temperature

# === HARDWARE IMPORTS ===
GPIO_AVAILABLE = False
try:
    import Jetson.GPIO as GPIO
    GPIO.setwarnings(False)
    GPIO_AVAILABLE = True
except ImportError:
    GPIO = None

JTOP_AVAILABLE = False
try:
    from jtop import jtop
    JTOP_AVAILABLE = True
except ImportError:
    jtop = None

class SensorsActuatorsNode:
    """ROS node for gas sensor, buzzer, and Jetson monitoring."""
    
    def __init__(self):
        try:
            rospy.init_node('sensors_actuators_node', anonymous=False)
            rospy.loginfo("Initializing Sensors Node...")
            
            # Running flag for clean shutdown
            self.running = True
            
            # Parameters - with defaults
            self.enable_gas_sensor = rospy.get_param('~enable_gas_sensor', False)
            self.enable_buzzer = rospy.get_param('~enable_buzzer', False)
            self.enable_jetson_stats = rospy.get_param('~enable_stats', True)  # Note: changed from enable_jetson_stats to enable_stats
            
            # GPIO pins with defaults
            self.gas_sensor_pin = rospy.get_param('~gas_sensor_gpio_pin', 18)
            self.buzzer_pin = rospy.get_param('~buzzer_pin', 16)
            
            # Polarity configuration
            self.gas_polarity = rospy.get_param('~gas_polarity', 'active_low')
            
            # State variables
            self.current_temp = 0.0
            self.current_power = 0.0
            self.gas_detected = False
            
            # Buzzer control
            self.buzzer_active = False
            self.buzzer_thread = None
            self.buzzer_lock = threading.Lock()
            
            # Initialize hardware
            self.gpio_initialized = False
            self.jtop_handle = None
            self.jtop_error_reported = False
            
            if self.enable_gas_sensor or self.enable_buzzer:
                self._init_gpio()
            
            # Initialize Jetson stats
            if self.enable_jetson_stats:
                self._init_jetson_stats()
            
            # ROS Publishers
            self.gas_pub = rospy.Publisher('/gas_detected', Bool, queue_size=1)
            self.temp_pub = rospy.Publisher('/jetson_temperature', Temperature, queue_size=1)
            self.power_pub = rospy.Publisher('/jetson_power', Float32, queue_size=1)
            
            # ROS Subscriber
            rospy.Subscriber('/buzzer_command', Bool, self.buzzer_callback)
            
            rospy.loginfo("Sensors Node Ready")
            rospy.loginfo("  Gas: %s | Buzzer: %s | Stats: %s",
                         "ON" if self.enable_gas_sensor else "OFF",
                         "ON" if self.enable_buzzer else "OFF",
                         "ON" if self.enable_jetson_stats else "OFF")
            
            # Set shutdown hook
            rospy.on_shutdown(self.shutdown)
            
        except Exception as e:
            rospy.logerr("FATAL: Failed to initialize node: %s", str(e))
            rospy.logerr(traceback.format_exc())
            self.running = False
            raise
    
    def _init_gpio(self):
        """Initialize GPIO pins."""
        if not GPIO_AVAILABLE:
            rospy.logwarn("Jetson.GPIO not available - hardware disabled")
            return
        
        try:
            GPIO.setmode(GPIO.BOARD)
            
            # Gas sensor pin (input)
            if self.enable_gas_sensor:
                GPIO.setup(self.gas_sensor_pin, GPIO.IN)
                rospy.loginfo("Gas sensor on pin %d", self.gas_sensor_pin)
            
            # Buzzer pin (output)
            if self.enable_buzzer:
                GPIO.setup(self.buzzer_pin, GPIO.OUT)
                GPIO.output(self.buzzer_pin, GPIO.LOW)
                rospy.loginfo("Buzzer on pin %d", self.buzzer_pin)
            
            self.gpio_initialized = True
            
        except Exception as e:
            rospy.logerr("GPIO init failed: %s", str(e))
            self.gpio_initialized = False
    
    def _init_jetson_stats(self):
        """Initialize Jetson stats using jtop."""
        if not JTOP_AVAILABLE:
            rospy.logwarn("jtop not available. Install with: sudo -H pip install jetson-stats")
            return
        
        try:
            # Try to open jtop connection
            self.jtop_handle = jtop()
            self.jtop_handle.start()
            
            # Give it a moment to initialize
            time.sleep(0.5)
            
            if self.jtop_handle.ok():
                rospy.loginfo("Jetson stats (jtop) initialized successfully")
            else:
                rospy.logwarn("jtop initialized but not ready")
                self.jtop_handle.close()
                self.jtop_handle = None
                
        except Exception as e:
            if not self.jtop_error_reported:
                rospy.logwarn("Failed to initialize jtop: %s", str(e))
                rospy.loginfo("You may need to add user to jtop group: sudo usermod -aG jtop $USER")
                self.jtop_error_reported = True
            self.jtop_handle = None
    
    def read_jetson_stats(self):
        """Read Jetson temperature and power using jtop."""
        if not self.running or not self.jtop_handle:
            return (0.0, 0.0)
        
        temp = 0.0
        power = 0.0
        
        try:
            if not self.jtop_handle.ok():
                # Try to restart jtop if it's not ok
                self.jtop_handle.close()
                time.sleep(0.1)
                self.jtop_handle = jtop()
                self.jtop_handle.start()
                time.sleep(0.5)
                
                if not self.jtop_handle.ok():
                    return (0.0, 0.0)
            
            # Get stats from jtop
            stats = self.jtop_handle.stats
            
            # Extract temperature (try multiple keys)
            if 'temperature' in stats:
                temp_data = stats['temperature']
                if isinstance(temp_data, dict):
                    # Try CPU temperature first
                    if 'CPU' in temp_data:
                        cpu_temp = temp_data['CPU']
                        if isinstance(cpu_temp, dict):
                            temp = cpu_temp.get('temp', 0.0)
                        else:
                            temp = float(cpu_temp)
                    # Try thermal as fallback
                    elif 'thermal' in temp_data:
                        thermal_temp = temp_data['thermal']
                        if isinstance(thermal_temp, dict):
                            temp = thermal_temp.get('temp', 0.0)
                        else:
                            temp = float(thermal_temp)
                    # Try any temperature value
                    else:
                        for key, value in temp_data.items():
                            try:
                                if isinstance(value, dict):
                                    temp = value.get('temp', 0.0)
                                else:
                                    temp = float(value)
                                if temp > 0:
                                    break
                            except:
                                pass
            
            # Extract power (try multiple keys)
            if 'power' in stats:
                power_data = stats['power']
                if isinstance(power_data, dict):
                    # Try different power keys
                    power_keys = ['tot', 'total', 'cur', 'power', 'val']
                    for key in power_keys:
                        if key in power_data:
                            try:
                                pwr_val = power_data[key]
                                if isinstance(pwr_val, dict):
                                    power = pwr_val.get('power', pwr_val.get('val', 0.0))
                                else:
                                    power = float(pwr_val)
                                break
                            except:
                                pass
            
            return (temp, power)
            
        except Exception as e:
            if not self.jtop_error_reported:
                rospy.logwarn("Jetson stats read error: %s", str(e))
                self.jtop_error_reported = True
            return (0.0, 0.0)
    
    def read_gas_sensor(self):
        """Read gas sensor state."""
        if not self.running or not self.gpio_initialized or not self.enable_gas_sensor:
            return False
        
        try:
            state = GPIO.input(self.gas_sensor_pin)
            
            # Convert based on polarity
            if self.gas_polarity == 'active_low':
                # LOW = gas detected (LM393 open-collector pulls low)
                detected = (state == GPIO.LOW)
            else:  # active_high
                # HIGH = gas detected
                detected = (state == GPIO.HIGH)
            
            return detected
            
        except Exception as e:
            rospy.logwarn("Gas sensor read error: %s", str(e))
            return False
    
    def buzzer_callback(self, msg):
        """Handle buzzer commands."""
        if not self.running or not self.gpio_initialized or not self.enable_buzzer:
            rospy.logwarn("Buzzer not initialized")
            return
        
        try:
            with self.buzzer_lock:
                # If command is same as current state, do nothing
                if msg.data == self.buzzer_active:
                    return
                
                self.buzzer_active = msg.data
                
                if msg.data:  # Turn ON
                    self.start_buzzer()
                    rospy.loginfo("Buzzer ON")
                else:  # Turn OFF
                    self.stop_buzzer()
                    rospy.loginfo("Buzzer OFF")
        except Exception as e:
            rospy.logerr("Buzzer callback error: %s", str(e))
    
    def start_buzzer(self):
        """Start buzzer in a thread."""
        def buzzer_pattern():
            """Continuous beeping pattern."""
            try:
                while self.running and not rospy.is_shutdown():
                    with self.buzzer_lock:
                        if not self.buzzer_active:
                            break
                    
                    # Beep pattern: 0.1s ON, 0.1s OFF
                    GPIO.output(self.buzzer_pin, GPIO.HIGH)
                    time.sleep(0.1)
                    GPIO.output(self.buzzer_pin, GPIO.LOW)
                    time.sleep(0.1)
                    
            except Exception as e:
                if self.running:
                    rospy.logerr("Buzzer thread error: %s", str(e))
            finally:
                # Ensure buzzer is OFF
                try:
                    GPIO.output(self.buzzer_pin, GPIO.LOW)
                except:
                    pass
        
        # Stop existing thread if running
        if self.buzzer_thread and self.buzzer_thread.is_alive():
            with self.buzzer_lock:
                self.buzzer_active = False
            self.buzzer_thread.join(timeout=0.5)
        
        # Start new thread
        self.buzzer_thread = threading.Thread(target=buzzer_pattern)
        self.buzzer_thread.daemon = True
        self.buzzer_thread.start()
    
    def stop_buzzer(self):
        """Stop buzzer."""
        with self.buzzer_lock:
            self.buzzer_active = False
        
        # Wait for thread to finish
        if self.buzzer_thread and self.buzzer_thread.is_alive():
            self.buzzer_thread.join(timeout=0.5)
        
        # Ensure buzzer is OFF
        try:
            if self.gpio_initialized:
                GPIO.output(self.buzzer_pin, GPIO.LOW)
        except:
            pass
    
    def publish_all_data(self):
        """Read and publish all sensor data."""
        if not self.running:
            return
        
        try:
            # Read gas sensor
            if self.enable_gas_sensor:
                self.gas_detected = self.read_gas_sensor()
                self.gas_pub.publish(Bool(data=self.gas_detected))
            
            # Read and publish Jetson stats
            if self.enable_jetson_stats:
                temp, power = self.read_jetson_stats()
                self.current_temp = temp
                self.current_power = power
                
                # Publish temperature
                temp_msg = Temperature()
                temp_msg.temperature = temp
                temp_msg.header.stamp = rospy.Time.now()
                self.temp_pub.publish(temp_msg)
                
                # Publish power
                self.power_pub.publish(Float32(data=power))
                
                # Log only if we have valid readings
                if temp > 0:
                    rospy.loginfo_throttle(30, "Jetson: %.1fC, %.1fW", temp, power)
        except Exception as e:
            rospy.logerr("Publish error: %s", str(e))
    
    def run(self):
        """Main loop."""
        try:
            rate = rospy.Rate(2.0)  # 2 Hz = twice per second
            
            while self.running and not rospy.is_shutdown():
                try:
                    self.publish_all_data()
                except Exception as e:
                    rospy.logerr("Loop error: %s", str(e))
                rate.sleep()
        except Exception as e:
            rospy.logerr("Run method error: %s", str(e))
    
    def shutdown(self):
        """Clean shutdown."""
        rospy.loginfo("Shutting down sensors node...")
        self.running = False
        
        # Stop buzzer
        self.stop_buzzer()
        
        # Give time for any in-progress operations to complete
        time.sleep(0.1)
        
        # Cleanup GPIO
        if self.gpio_initialized and GPIO_AVAILABLE:
            try:
                GPIO.cleanup()
            except:
                pass
        
        # Close jtop connection
        if self.jtop_handle:
            try:
                self.jtop_handle.close()
            except:
                pass

if __name__ == '__main__':
    try:
        node = SensorsActuatorsNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted")
    except Exception as e:
        rospy.logerr("Main error: %s", str(e))
        rospy.logerr(traceback.format_exc())