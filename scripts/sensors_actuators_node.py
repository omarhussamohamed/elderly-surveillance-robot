#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Sensors & Actuators Node
- Gas detection with edge-based publishing
- Buzzer control (local + cloud)
- Jetson temperature publishing
- Clean logs and safe shutdown
"""

import rospy
import time
import threading
import atexit
from std_msgs.msg import Bool
from sensor_msgs.msg import Temperature


class SensorsActuatorsNode(object):
    def __init__(self):
        rospy.init_node('sensors_actuators_node', anonymous=False)
        rospy.loginfo("Starting Sensors & Actuators Node")

        # ── Parameters ─────────────────────────────────────────────
        self.enable_gas_sensor = rospy.get_param('~enable_gas_sensor', True)
        self.enable_buzzer = rospy.get_param('~enable_buzzer', True)
        self.enable_stats = rospy.get_param('~enable_stats', True)

        self.gas_sensor_pin = rospy.get_param('~gas_pin', 18)
        self.buzzer_pin = rospy.get_param('~buzzer_pin', 16)
        self.gas_polarity = rospy.get_param('~gas_polarity', 'active_low')

        self.loop_rate = rospy.get_param('~loop_rate', 10.0)
        self.stats_rate = rospy.get_param('~stats_rate', 1.0)

        # ── State ─────────────────────────────────────────────────
        self.gas_detected = False
        self.cloud_buzzer = False
        self.buzzer_active = False

        self.last_stats_time = 0.0
        self.running = True

        self.buzzer_stop_event = threading.Event()
        self.buzzer_thread = None

        # ── Hardware Init ─────────────────────────────────────────
        self.gpio_available = False
        self.GPIO = None
        self._init_gpio()

        # ── ROS I/O ───────────────────────────────────────────────
        self.gas_pub = rospy.Publisher(
            '/gas_detected', Bool, queue_size=1, latch=True
        )
        self.temp_pub = rospy.Publisher(
            '/jetson_temperature', Temperature, queue_size=1
        )

        rospy.Subscriber(
            '/buzzer_command', Bool, self._buzzer_command_cb, queue_size=1
        )

        rospy.on_shutdown(self.shutdown)
        atexit.register(self._cleanup_gpio)

        rospy.sleep(5.0)  # Sensor warm-up
        rospy.loginfo("Sensors & Actuators Node ready")

    # ─────────────────────────────────────────────────────────────
    def _init_gpio(self):
        if not (self.enable_gas_sensor or self.enable_buzzer):
            return

        try:
            import Jetson.GPIO as GPIO
            GPIO.setmode(GPIO.BOARD)
            GPIO.setwarnings(False)

            if self.enable_gas_sensor:
                GPIO.setup(self.gas_sensor_pin, GPIO.IN)

            if self.enable_buzzer:
                GPIO.setup(self.buzzer_pin, GPIO.OUT, initial=GPIO.LOW)

            self.GPIO = GPIO
            self.gpio_available = True
            rospy.loginfo("GPIO initialized successfully")

        except ImportError:
            rospy.logwarn("Jetson.GPIO not available (simulation mode)")
        except Exception as e:
            rospy.logerr("GPIO init failed: %s", str(e))

    # ─────────────────────────────────────────────────────────────
    def _read_gas_sensor(self):
        if not (self.gpio_available and self.enable_gas_sensor):
            return False

        try:
            state = self.GPIO.input(self.gas_sensor_pin)
            if self.gas_polarity == 'active_low':
                return state == self.GPIO.LOW
            else:
                return state == self.GPIO.HIGH
        except Exception:
            return False

    # ─────────────────────────────────────────────────────────────
    def _buzzer_command_cb(self, msg):
        if not self.enable_buzzer:
            return
        self.cloud_buzzer = msg.data
        self._update_buzzer_state()

    # ─────────────────────────────────────────────────────────────
    def _update_buzzer_state(self):
        desired = self.gas_detected or self.cloud_buzzer

        if desired and not self.buzzer_active:
            self.buzzer_active = True
            self._start_buzzer()
        elif not desired and self.buzzer_active:
            self.buzzer_active = False
            self._stop_buzzer()

    # ─────────────────────────────────────────────────────────────
    def _start_buzzer(self):
        self.buzzer_stop_event.clear()

        def buzzer_loop():
            try:
                while not self.buzzer_stop_event.is_set() and not rospy.is_shutdown():
                    if self.gpio_available:
                        self.GPIO.output(self.buzzer_pin, self.GPIO.HIGH)
                    time.sleep(0.5)
                    if self.gpio_available:
                        self.GPIO.output(self.buzzer_pin, self.GPIO.LOW)
                    time.sleep(0.5)
            finally:
                if self.gpio_available:
                    self.GPIO.output(self.buzzer_pin, self.GPIO.LOW)

        if self.buzzer_thread and self.buzzer_thread.is_alive():
            self.buzzer_stop_event.set()
            self.buzzer_thread.join(timeout=1.0)

        self.buzzer_thread = threading.Thread(target=buzzer_loop)
        self.buzzer_thread.daemon = True
        self.buzzer_thread.start()

    # ─────────────────────────────────────────────────────────────
    def _stop_buzzer(self):
        self.buzzer_stop_event.set()
        if self.gpio_available:
            try:
                self.GPIO.output(self.buzzer_pin, self.GPIO.LOW)
            except Exception:
                pass

    # ─────────────────────────────────────────────────────────────
    def _read_temperature(self):
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                return int(f.read().strip()) / 1000.0
        except Exception:
            return 35.0  # Safe fallback

    # ─────────────────────────────────────────────────────────────
    def _publish_cycle(self):
        # Gas logic (edge-based)
        if self.enable_gas_sensor:
            new_state = self._read_gas_sensor()
            if new_state != self.gas_detected:
                self.gas_detected = new_state
                self.gas_pub.publish(Bool(self.gas_detected))
                self._update_buzzer_state()

        # Temperature + stats
        if self.enable_stats:
            temp = self._read_temperature()

            msg = Temperature()
            msg.header.stamp = rospy.Time.now()
            msg.temperature = temp
            self.temp_pub.publish(msg)

            now = time.time()
            if now - self.last_stats_time >= (1.0 / self.stats_rate):
                rospy.loginfo(
                    "Stats: %.1f°C, Gas: %s",
                    temp, "True" if self.gas_detected else "False"
                )
                self.last_stats_time = now

    # ─────────────────────────────────────────────────────────────
    def run(self):
        rate = rospy.Rate(self.loop_rate)
        while self.running and not rospy.is_shutdown():
            try:
                self._publish_cycle()
            except Exception as e:
                rospy.logerr("Runtime error: %s", str(e))
            rate.sleep()

    # ─────────────────────────────────────────────────────────────
    def _cleanup_gpio(self):
        if self.gpio_available:
            try:
                self.GPIO.cleanup()
            except Exception:
                pass

    def shutdown(self):
        rospy.loginfo("Shutting down Sensors & Actuators Node")
        self.running = False
        self.buzzer_stop_event.set()
        self._stop_buzzer()
        self._cleanup_gpio()


if __name__ == '__main__':
    try:
        node = SensorsActuatorsNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
