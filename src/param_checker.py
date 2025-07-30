#!/usr/bin/env python3
"""
Parameter Checker Tool
Verifies that calibration parameters are actually loaded by the hardware driver
"""

import subprocess
import sys

def check_calibration_parameters():
    """Check if calibration parameters are loaded"""
    print("🔍 Checking if calibration parameters are loaded...")
    print("=" * 60)
    
    try:
        # Get list of nodes
        result = subprocess.run(['ros2', 'node', 'list'], 
                              capture_output=True, text=True, timeout=10)
        
        if result.returncode != 0:
            print("❌ Failed to get node list")
            return False
            
        nodes = result.stdout.strip().split('\n')
        hardware_node = None
        
        # Find hardware driver node
        for node in nodes:
            if 'yahboom_hardware_driver' in node:
                hardware_node = node
                break
                
        if not hardware_node:
            print("❌ Hardware driver node not found!")
            print("   Available nodes:")
            for node in nodes:
                print(f"   - {node}")
            print("\n   Make sure to run:")
            print("   ros2 launch robot_bringup robot_bringup.launch.py")
            return False
            
        print(f"✅ Found hardware driver: {hardware_node}")
        
        # Get parameters from the node
        print("\n🔍 Checking calibration parameters...")
        
        param_result = subprocess.run(['ros2', 'param', 'list', hardware_node], 
                                    capture_output=True, text=True, timeout=10)
        
        if param_result.returncode != 0:
            print("❌ Failed to get parameters")
            return False
            
        params = param_result.stdout.strip().split('\n')
        
        # Check for calibration parameters
        calibration_params = [
            'motor_calibration_fl',
            'motor_calibration_fr', 
            'motor_calibration_rl',
            'motor_calibration_rr',
            'invert_forward_direction'
        ]
        
        found_params = {}
        missing_params = []
        
        for cal_param in calibration_params:
            if any(cal_param in param for param in params):
                # Get the parameter value
                value_result = subprocess.run(['ros2', 'param', 'get', hardware_node, cal_param], 
                                            capture_output=True, text=True, timeout=5)
                if value_result.returncode == 0:
                    value = value_result.stdout.strip()
                    found_params[cal_param] = value
                else:
                    missing_params.append(cal_param)
            else:
                missing_params.append(cal_param)
        
        # Report results
        if found_params:
            print("✅ Found calibration parameters:")
            for param, value in found_params.items():
                status = "✅" if param.startswith('motor_calibration') and '1.0' not in value else "⚠️"
                print(f"   {param}: {value} {status}")
                
            print("\n📊 Parameter Analysis:")
            
            # Check if parameters are at default values
            all_default = True
            for param, value in found_params.items():
                if param.startswith('motor_calibration') and '1.0' not in value:
                    all_default = False
                    break
                    
            if all_default:
                print("⚠️  All motor calibration values appear to be at default (1.0)")
                print("   This suggests calibration may not be applied")
                print("   Check your hardware.yaml file")
            else:
                print("✅ Custom calibration values detected")
                print("   Calibration should be active")
                
        else:
            print("❌ No calibration parameters found!")
            
        if missing_params:
            print("\n⚠️  Missing parameters:")
            for param in missing_params:
                print(f"   - {param}")
                
        return len(found_params) > 0
        
    except subprocess.TimeoutExpired:
        print("❌ Timeout while checking parameters")
        return False
    except Exception as e:
        print(f"❌ Error checking parameters: {e}")
        return False

def check_config_file():
    """Check if hardware config file has calibration values"""
    print("\n🔍 Checking hardware configuration file...")
    print("=" * 60)
    
    config_paths = [
        "src/robot_bringup/config/hardware.yaml",
        "./src/robot_bringup/config/hardware.yaml",
        "../src/robot_bringup/config/hardware.yaml"
    ]
    
    config_found = False
    
    for config_path in config_paths:
        try:
            with open(config_path, 'r') as f:
                content = f.read()
                config_found = True
                
                print(f"✅ Found config file: {config_path}")
                
                # Check for calibration parameters
                cal_params = ['motor_calibration_fl', 'motor_calibration_fr', 
                             'motor_calibration_rl', 'motor_calibration_rr']
                
                found_in_config = []
                for param in cal_params:
                    if param in content:
                        # Extract the line with this parameter
                        lines = content.split('\n')
                        for line in lines:
                            if param in line and ':' in line:
                                found_in_config.append(line.strip())
                                break
                
                if found_in_config:
                    print("✅ Found calibration in config file:")
                    for line in found_in_config:
                        # Check if it's not default
                        if '1.0' not in line:
                            print(f"   {line} ✅")
                        else:
                            print(f"   {line} ⚠️ (default)")
                else:
                    print("⚠️  No calibration parameters found in config file")
                    
                break
                
        except FileNotFoundError:
            continue
        except Exception as e:
            print(f"❌ Error reading {config_path}: {e}")
            
    if not config_found:
        print("❌ Could not find hardware.yaml config file")
        print("   Expected locations:")
        for path in config_paths:
            print(f"   - {path}")
            
    return config_found

def main():
    print("🔧 ROS2 Parameter Verification Tool")
    print("This tool checks if your calibration is actually loaded")
    print("")
    
    # Check running parameters
    params_ok = check_calibration_parameters()
    
    # Check config file
    config_ok = check_config_file()
    
    print("\n📋 SUMMARY:")
    print("=" * 60)
    
    if params_ok and config_ok:
        print("✅ Calibration appears to be properly loaded")
        print("   If you're still seeing issues, the calibration values")
        print("   might need to be larger, or there may be mechanical problems")
    elif params_ok:
        print("✅ Parameters loaded, but config file check failed")
        print("   Calibration should still be working")
    elif config_ok:
        print("⚠️  Config file looks good, but parameters not loaded")
        print("   Try restarting the robot:")
        print("   ros2 launch robot_bringup robot_bringup.launch.py")
    else:
        print("❌ Calibration not found in running system or config")
        print("   Steps to fix:")
        print("   1. Make sure hardware.yaml has calibration values")
        print("   2. Rebuild: colcon build && source install/setup.bash")
        print("   3. Restart robot: ros2 launch robot_bringup robot_bringup.launch.py")
        
    print("\n🔧 Next: Run the calibration verification tool to test movement")

if __name__ == '__main__':
    main()