#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Direct buzzer test without ROS dependency - PYTHON 2 VERSION
"""

import time

def test_buzzer_direct():
    print("Testing buzzer directly via GPIO (Python 2)...")
    
    try:
        import Jetson.GPIO as GPIO
        
        # Setup
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        buzzer_pin = 16
        
        GPIO.setup(buzzer_pin, GPIO.OUT, initial=GPIO.LOW)
        print("Buzzer on pin %d" % buzzer_pin)
        
        # Test sequence
        print("1. Buzzer ON (steady)...")
        GPIO.output(buzzer_pin, GPIO.HIGH)
        time.sleep(2)
        
        print("2. Buzzer OFF...")
        GPIO.output(buzzer_pin, GPIO.LOW)
        time.sleep(1)
        
        print("3. Buzzer blinking (5 times)...")
        for i in range(5):
            GPIO.output(buzzer_pin, GPIO.HIGH)
            time.sleep(0.5)
            GPIO.output(buzzer_pin, GPIO.LOW)
            time.sleep(0.5)
            print("  Blink %d/5" % (i+1))
        
        print("4. Cleanup...")
        GPIO.output(buzzer_pin, GPIO.LOW)
        GPIO.cleanup()
        
        print("Buzzer test complete!")
        
    except ImportError:
        print("Jetson.GPIO not installed")
    except Exception as e:
        print("Error: %s" % str(e))
        try:
            GPIO.cleanup()
        except:
            pass

if __name__ == '__main__':
    test_buzzer_direct()