#!/usr/bin/env python3
"""
Simple Velocity Calibration Script for Mecanum Robot

Tests forward movement at 0.5 m/s for 5 seconds (expecting 2.5m travel)
Then tests strafe and rotation with similar pattern.
Ensures proper stop commands are sent.

Usage:
    python3 velocity_calibration.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import time
import math
import sys
from datetime import datetime

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class VelocityCalibration(Node):
    """Simple velocity calibration for mecanum robot"""
    
    def __init__(self):
        super().__init__('velocity_calibration')
        
        # Setup QoS profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos_profile)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, qos_profile)
        
        # State variables
        self.current_odom = None
        
        self.get_logger().info("Velocity Calibration Tool Initialized")
        self.get_logger().info(f"Publishing cmd_vel to: {self.cmd_vel_pub.topic_name}")
        self.get_logger().info(f"Subscribing to odom from: {self.odom_sub.topic_name}")
    
    def odom_callback(self, msg):
        """Store latest odometry data"""
        self.current_odom = msg
    
    def get_current_position(self):
        """Get current robot position (x, y, theta) from odometry"""
        if self.current_odom is None:
            return None
            
        pos = self.current_odom.pose.pose.position
        orientation = self.current_odom.pose.pose.orientation
        
        # Convert quaternion to euler angle (yaw)
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        theta = math.atan2(siny_cosp, cosy_cosp)
        
        return (pos.x, pos.y, theta)
    
    def wait_for_odometry(self, timeout=10.0):
        """Wait for odometry data"""
        self.get_logger().info("Waiting for odometry data...")
        
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_odom is not None:
                self.get_logger().info("Odometry data received")
                return True
        
        self.get_logger().error("Timeout waiting for odometry")
        return False
    
    def stop_robot(self):
        """Send zero velocity command to stop robot"""
        self.get_logger().info("Creating STOP command (all zeros)...")
        stop_msg = Twist()
        # All fields default to 0.0: linear.x, linear.y, linear.z, angular.x, angular.y, angular.z
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.linear.z = 0.0
        stop_msg.angular.x = 0.0
        stop_msg.angular.y = 0.0
        stop_msg.angular.z = 0.0
        
        self.get_logger().info(f"Publishing STOP: linear.x={stop_msg.linear.x}, linear.y={stop_msg.linear.y}, angular.z={stop_msg.angular.z}")
        self.cmd_vel_pub.publish(stop_msg)
        
        # Add a small delay and send again to be sure
        time.sleep(0.1)
        self.cmd_vel_pub.publish(stop_msg)
        self.get_logger().info("STOP command published (sent twice for reliability)")
    
    def execute_motion_test(self, velocity_cmd, test_name, expected_distance):
        """Execute a single motion test"""
        
        self.get_logger().info(f"\n=== {test_name} ===")
        self.get_logger().info(f"Expected distance: {expected_distance}m")
        
        # Stabilization delay
        self.get_logger().info("Stabilizing for 2 seconds...")
        time.sleep(2.0)
        
        # Get starting position
        start_pos = self.get_current_position()
        if start_pos is None:
            self.get_logger().error("Could not get starting position")
            return False
        
        self.get_logger().info(f"Start position: X={start_pos[0]:.3f}m, Y={start_pos[1]:.3f}m, Θ={math.degrees(start_pos[2]):.1f}°")
        
        # Execute motion for 5 seconds
        self.get_logger().info("Executing motion for 5 seconds...")
        start_time = time.time()
        end_time = start_time + 5.0
        
        # Debug: Show what command we're sending
        self.get_logger().info(f"Sending cmd_vel: linear.x={velocity_cmd.linear.x}, linear.y={velocity_cmd.linear.y}, angular.z={velocity_cmd.angular.z}")
        
        # Send velocity commands at 10 Hz (every 0.1 seconds)
        command_count = 0
        last_log_time = start_time
        loop_interval = 0.1  # 10 Hz
        
        while time.time() < end_time and rclpy.ok():
            current_time = time.time()
            
            # Log progress every second
            if current_time - last_log_time >= 1.0:
                elapsed = current_time - start_time
                remaining = end_time - current_time
                self.get_logger().info(f"Motion progress: {elapsed:.1f}s elapsed, {remaining:.1f}s remaining, {command_count} commands sent")
                last_log_time = current_time
            
            # Publish command
            self.cmd_vel_pub.publish(velocity_cmd)
            command_count += 1
            
            # Simple sleep instead of rate.sleep()
            time.sleep(loop_interval)
        
        actual_duration = time.time() - start_time
        self.get_logger().info(f"EXIT MOTION LOOP: {actual_duration:.2f}s, sent {command_count} commands")
        
        # Stop robot immediately
        self.get_logger().info("TIME TO STOP - calling stop_robot()...")
        self.stop_robot()
        self.get_logger().info("stop_robot() completed, robot should be stopped")
        
        # Wait for robot to stop
        time.sleep(1.0)
        
        # Get ending position
        end_pos = self.get_current_position()
        if end_pos is None:
            self.get_logger().error("Could not get ending position")
            return False
        
        self.get_logger().info(f"End position: X={end_pos[0]:.3f}m, Y={end_pos[1]:.3f}m, Θ={math.degrees(end_pos[2]):.1f}°")
        
        # Calculate actual distance traveled
        if "Forward" in test_name:
            # For forward motion, calculate linear distance
            actual_distance = math.sqrt((end_pos[0] - start_pos[0])**2 + (end_pos[1] - start_pos[1])**2)
        elif "Strafe" in test_name:
            # For strafe motion, calculate linear distance
            actual_distance = math.sqrt((end_pos[0] - start_pos[0])**2 + (end_pos[1] - start_pos[1])**2)
        elif "Rotation" in test_name:
            # For rotation, calculate angular distance
            angle_diff = end_pos[2] - start_pos[2]
            # Normalize angle
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            actual_distance = abs(angle_diff)
        else:
            actual_distance = 0.0
        
        # Calculate error
        error = actual_distance - expected_distance
        error_percent = (error / expected_distance) * 100 if expected_distance != 0 else 0
        
        self.get_logger().info(f"Actual distance: {actual_distance:.3f}")
        self.get_logger().info(f"Expected distance: {expected_distance:.3f}")
        self.get_logger().info(f"Error: {error:.3f} ({error_percent:+.1f}%)")
        
        return True
    
    def run_rotation_tests(self):
        """Interactive rotation testing with multiple angles and repeat option"""
        
        # Rotation test options
        rotation_options = {
            '1': {'name': '45° rotation', 'angle_deg': 45, 'angle_rad': math.pi/4, 'duration': (math.pi/4) / 1.0},
            '2': {'name': '90° rotation', 'angle_deg': 90, 'angle_rad': math.pi/2, 'duration': (math.pi/2) / 1.0}, 
            '3': {'name': '180° rotation', 'angle_deg': 180, 'angle_rad': math.pi, 'duration': math.pi / 1.0},
            '4': {'name': '360° rotation', 'angle_deg': 360, 'angle_rad': 2*math.pi, 'duration': (2*math.pi) / 1.0},
            '5': {'name': '5-second test (286.5°)', 'angle_deg': 286.5, 'angle_rad': 5.0, 'duration': 5.0}
        }
        
        while True:
            self.get_logger().info("\n=== ROTATION TEST OPTIONS ===")
            print("Choose a rotation test:")
            print("  1. 45°  rotation  (0.785 rad, 0.79s)")
            print("  2. 90°  rotation  (1.571 rad, 1.57s)")
            print("  3. 180° rotation  (3.142 rad, 3.14s)")
            print("  4. 360° rotation  (6.283 rad, 6.28s)")
            print("  5. 5-second test  (5.000 rad, 5.00s)")
            print("  s. Skip rotation tests")
            print("  q. Quit rotation tests")
            
            choice = input("Enter your choice: ").strip().lower()
            
            if choice == 's':
                self.get_logger().info("Skipping all rotation tests")
                break
            elif choice == 'q':
                self.get_logger().info("Exiting rotation tests")
                break
            elif choice in rotation_options:
                option = rotation_options[choice]
                
                self.get_logger().info(f"\n=== {option['name'].upper()} ===")
                self.get_logger().info(f"Expected: {option['angle_rad']:.3f} radians = {option['angle_deg']:.1f}°")
                self.get_logger().info(f"Duration: {option['duration']:.2f} seconds at 1.0 rad/s")
                
                confirm = input(f"Press Enter to start {option['name']}, or 'c' to cancel: ").strip().lower()
                if confirm != 'c':
                    # Create rotation command
                    cmd_rotate = Twist()
                    cmd_rotate.angular.z = 1.0
                    
                    # Execute test with custom duration
                    if not self.execute_rotation_test(cmd_rotate, option['name'], option['angle_rad'], option['duration']):
                        self.get_logger().error("Rotation test failed")
                        continue
                else:
                    self.get_logger().info(f"Cancelled {option['name']}")
                
                # Ask if user wants to test another angle
                repeat = input("\nTest another rotation angle? (y/N): ").strip().lower()
                if repeat not in ['y', 'yes']:
                    break
            else:
                print("Invalid choice. Please try again.")

    def execute_rotation_test(self, velocity_cmd, test_name, expected_radians, duration):
        """Execute a rotation test with custom duration"""
        
        self.get_logger().info(f"\n=== {test_name} ===")
        self.get_logger().info(f"Expected rotation: {expected_radians:.3f} radians ({math.degrees(expected_radians):.1f}°)")
        
        # Stabilization delay
        self.get_logger().info("Stabilizing for 2 seconds...")
        time.sleep(2.0)
        
        # Get starting position
        start_pos = self.get_current_position()
        if start_pos is None:
            self.get_logger().error("Could not get starting position")
            return False
        
        self.get_logger().info(f"Start position: X={start_pos[0]:.3f}m, Y={start_pos[1]:.3f}m, Θ={math.degrees(start_pos[2]):.1f}°")
        
        # Debug: Show what command we're sending
        self.get_logger().info(f"Sending cmd_vel: angular.z={velocity_cmd.angular.z}, duration={duration:.2f}s")
        
        # Execute motion for specified duration
        self.get_logger().info(f"Executing rotation for {duration:.2f} seconds...")
        start_time = time.time()
        end_time = start_time + duration
        
        command_count = 0
        last_log_time = start_time
        loop_interval = 0.1  # 10 Hz
        
        while time.time() < end_time and rclpy.ok():
            current_time = time.time()
            
            # Log progress every second
            if current_time - last_log_time >= 1.0:
                elapsed = current_time - start_time
                remaining = end_time - current_time
                self.get_logger().info(f"Rotation progress: {elapsed:.1f}s elapsed, {remaining:.1f}s remaining")
                last_log_time = current_time
            
            # Publish command
            self.cmd_vel_pub.publish(velocity_cmd)
            command_count += 1
            
            # Simple sleep instead of rate.sleep()
            time.sleep(loop_interval)
        
        actual_duration = time.time() - start_time
        self.get_logger().info(f"EXIT ROTATION LOOP: {actual_duration:.2f}s, sent {command_count} commands")
        
        # Stop robot immediately
        self.get_logger().info("TIME TO STOP - calling stop_robot()...")
        self.stop_robot()
        self.get_logger().info("stop_robot() completed, robot should be stopped")
        
        # Wait for robot to stop
        time.sleep(1.0)
        
        # Get ending position
        end_pos = self.get_current_position()
        if end_pos is None:
            self.get_logger().error("Could not get ending position")
            return False
        
        self.get_logger().info(f"End position: X={end_pos[0]:.3f}m, Y={end_pos[1]:.3f}m, Θ={math.degrees(end_pos[2]):.1f}°")
        
        # Calculate actual rotation
        angle_diff = end_pos[2] - start_pos[2]
        # Normalize angle to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        actual_rotation = abs(angle_diff)
        
        # Calculate error
        error = actual_rotation - expected_radians
        error_percent = (error / expected_radians) * 100 if expected_radians != 0 else 0
        
        self.get_logger().info(f"Actual rotation: {actual_rotation:.3f} rad ({math.degrees(actual_rotation):.1f}°)")
        self.get_logger().info(f"Expected rotation: {expected_radians:.3f} rad ({math.degrees(expected_radians):.1f}°)")
        self.get_logger().info(f"Error: {error:.3f} rad ({error_percent:+.1f}%)")
        
        return True
    
    def check_joystick_interference(self):
        """Check if joystick is running and warn user"""
        try:
            # Check if joystick_controller node exists
            result = self.get_node_names()
            if '/joystick_controller' in result:
                self.get_logger().warn("WARNING: joystick_controller is running!")
                self.get_logger().warn("This may interfere with calibration commands.")
                self.get_logger().warn("Launch robot with use_joystick:=false for calibration")
                return True
        except:
            pass
        return False

    def run_calibration(self):
        """Run the complete calibration sequence"""
        
        try:
            # Wait for odometry
            if not self.wait_for_odometry():
                return False
                
            # Check for joystick interference
            if self.check_joystick_interference():
                self.get_logger().warn("Continuing anyway, but results may be unreliable...")
            
            self.get_logger().info("Starting velocity calibration...")
            
            # Test 1: Forward motion at 0.5 m/s for 5 seconds (expect 2.5m)
            self.get_logger().info("\n=== READY FOR TEST 1: FORWARD MOTION ===")
            response = input("Press Enter to start, or 's' to skip: ").strip().lower()
            if response != 's':
                cmd_forward = Twist()
                cmd_forward.linear.x = 0.5
                if not self.execute_motion_test(cmd_forward, "Forward Motion Test", 2.5):
                    return False
            else:
                self.get_logger().info("Skipping forward motion test")
            
            # Test 2: Strafe right at 0.5 m/s for 5 seconds (expect 2.5m)
            self.get_logger().info("\n=== READY FOR TEST 2: STRAFE RIGHT ===")
            response = input("Press Enter to start, or 's' to skip: ").strip().lower()
            if response != 's':
                cmd_strafe = Twist()
                cmd_strafe.linear.y = 0.5
                if not self.execute_motion_test(cmd_strafe, "Strafe Right Test", 2.5):
                    return False
            else:
                self.get_logger().info("Skipping strafe test")
            
            # Test 3: Rotation tests with multiple angle options
            self.run_rotation_tests()
            
            self.get_logger().info("\n=== CALIBRATION COMPLETE ===")
            self.get_logger().info("Check the logged results above for velocity accuracy")
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Calibration failed: {str(e)}")
            self.stop_robot()
            return False
    
    def destroy_node(self):
        """Ensure robot stops when node is destroyed"""
        self.get_logger().info("Shutting down - stopping robot")
        self.stop_robot()
        super().destroy_node()


def main():
    """Main function"""
    
    print("="*60)
    print("    SIMPLE VELOCITY CALIBRATION")
    print("="*60)
    print("This will test:")
    print("  • Forward at 0.5 m/s for 5s (expecting 2.5m travel)")
    print("  • Strafe at 0.5 m/s for 5s (expecting 2.5m travel)")
    print("  • Rotate at 1.0 rad/s for 5s (expecting 5.0 rad)")
    print()
    print("SETUP REQUIRED:")
    print("  • Launch robot with: use_joystick:=false")
    print("  • Example: ros2 launch robot_bringup enhanced_robot_bringup.launch.py use_joystick:=false")
    print("  • This prevents joystick from interfering with calibration commands")
    print()
    print("SAFETY: Ensure 5m x 5m clear space around robot")
    print()
    
    # Get user confirmation (skip if --auto flag is provided)
    if "--auto" not in sys.argv:
        try:
            response = input("Continue with calibration? (y/N): ")
            if response.lower() not in ['y', 'yes']:
                print("Calibration cancelled")
                return
        except KeyboardInterrupt:
            print("\nCalibration cancelled")
            return
    
    # Initialize ROS2
    rclpy.init()
    
    calibration_tool = None
    try:
        calibration_tool = VelocityCalibration()
        success = calibration_tool.run_calibration()
        
        if success:
            print("\nCalibration completed successfully!")
        else:
            print("\nCalibration failed")
            
    except KeyboardInterrupt:
        print("\nCalibration interrupted")
        if calibration_tool:
            calibration_tool.stop_robot()
    except Exception as e:
        print(f"\nCalibration error: {str(e)}")
        if calibration_tool:
            calibration_tool.stop_robot()
    finally:
        if calibration_tool:
            try:
                calibration_tool.destroy_node()
            except:
                pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()