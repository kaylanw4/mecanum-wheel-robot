#!/usr/bin/env python3
"""
Test script for USB port detection
Helps diagnose USB port issues for Yahboom robot
"""

import sys
sys.path.append('/home/kaylanw4/ros2_ws/src/robot_hardware/robot_hardware')

import glob
import serial
from yahboom_driver import find_yahboom_port

def test_usb_ports():
    """Test all available USB ports"""
    print("🔍 USB Port Detection Test")
    print("=" * 40)
    
    # Check all possible ports
    all_ports = []
    all_ports.extend(glob.glob('/dev/ttyUSB*'))
    all_ports.extend(glob.glob('/dev/ttyACM*'))
    all_ports.sort()
    
    print(f"📋 All USB serial ports found: {all_ports}")
    
    if not all_ports:
        print("❌ No USB serial ports found!")
        print("   Make sure:")
        print("   1. Robot is connected via USB")
        print("   2. User is in dialout group: sudo usermod -a -G dialout $USER")
        print("   3. USB cable is working")
        return
    
    print("\n🧪 Testing each port:")
    accessible_ports = []
    
    for port in all_ports:
        try:
            with serial.Serial(port, 115200, timeout=1):
                print(f"   ✅ {port} - ACCESSIBLE")
                accessible_ports.append(port)
        except Exception as e:
            print(f"   ❌ {port} - NOT ACCESSIBLE ({e})")
    
    print(f"\n📊 Summary:")
    print(f"   Total ports found: {len(all_ports)}")
    print(f"   Accessible ports: {len(accessible_ports)}")
    
    if accessible_ports:
        print(f"   Recommended port: {accessible_ports[0]}")
    
    print(f"\n🎯 Auto-detection result:")
    selected_port = find_yahboom_port()
    print(f"   Selected port: {selected_port}")
    
    return selected_port

if __name__ == "__main__":
    test_usb_ports()