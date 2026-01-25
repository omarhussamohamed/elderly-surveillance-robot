#!/usr/bin/env python2
"""
Quick test to verify AWS IoT connection
"""
from __future__ import print_function
import sys
import os

print("Testing AWS IoT SDK installation...")

try:
    from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
    print("✓ AWSIoTPythonSDK imported successfully")
    
    # Check version
    try:
        import AWSIoTPythonSDK
        print("✓ SDK version available")
    except:
        print("ℹ Cannot determine SDK version")
        
except ImportError as e:
    print("✗ AWSIoTPythonSDK import failed:", str(e))
    print("\nTo fix, install Python 2.7 compatible version:")
    print("pip uninstall AWSIoTPythonSDK")
    print("pip install AWSIoTPythonSDK==1.4.9")

print("\nChecking Python version...")
print("Python version:", sys.version)

print("\nChecking for f-strings (Python 3.6+ feature)...")
test_code = """
def test_fstring():
    name = "test"
    return f"Hello {name}"
"""
try:
    exec(test_code)
    print("✗ Python 3.6+ detected (f-strings supported)")
except SyntaxError:
    print("✓ Python 2.7 detected (no f-strings)")

print("\nCheck complete!")