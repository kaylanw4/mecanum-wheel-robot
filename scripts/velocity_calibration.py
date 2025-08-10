#!/usr/bin/env python3
"""
Professional Mecanum Robot Velocity Calibration Tool

This industry-standard calibration script tests and calibrates velocity control
for a mecanum wheel robot across all three degrees of freedom:
1. Linear forward/backward motion (X-axis)
2. Lateral strafe motion (Y-axis) 
3. Rotational motion (Z-axis)

The script provides:
- Automated velocity commands with precise timing
- Real-time position/orientation tracking
- Statistical analysis of velocity accuracy
- Calibration factor recommendations
- Professional data logging and reporting

Author: Claude Code Assistant
Date: 2025-08-10
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

import sys
import time
import math
import csv
import os
from datetime import datetime
from dataclasses import dataclass
from typing import List, Optional, Tuple
import numpy as np
from scipy import stats

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import Joy

# ANSI color codes for terminal output
class Colors:
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    WHITE = '\033[97m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    END = '\033[0m'


@dataclass
class VelocityTestResult:
    """Data structure for storing individual test results"""
    test_name: str
    commanded_velocity: float
    measured_velocity: float
    duration: float
    start_position: Tuple[float, float, float]  # x, y, theta
    end_position: Tuple[float, float, float]
    displacement: float
    velocity_error: float
    velocity_error_percent: float
    timestamp: str


@dataclass
class CalibrationResults:
    """Data structure for storing calibration analysis results"""
    test_type: str  # 'linear_x', 'linear_y', 'angular_z'
    mean_error_percent: float
    std_error_percent: float
    recommended_scale_factor: float
    r_squared: float
    test_results: List[VelocityTestResult]


class VelocityCalibrationTool(Node):
    """
    Professional velocity calibration tool for mecanum wheel robots
    
    Features:
    - Comprehensive testing of all 3 DOF (linear X/Y, angular Z)
    - Statistical analysis with R² correlation
    - Automated calibration factor calculation
    - Professional data logging and CSV export
    - Real-time progress monitoring
    - Safety features with emergency stop
    """
    
    def __init__(self):
        super().__init__('velocity_calibration_tool')
        
        # Setup QoS profiles for reliable communication
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Publishers and subscribers
        # Note: Publishing to /cmd_vel bypasses the velocity smoother for direct hardware control
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos_profile)
        self.status_pub = self.create_publisher(String, 'calibration_status', qos_profile)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, qos_profile)
        self.joy_sub = self.create_subscription(
            Joy, 'joy', self.joy_callback, qos_profile)
            
        # State variables
        self.current_odom: Optional[Odometry] = None
        self.initial_position: Optional[Tuple[float, float, float]] = None
        self.test_start_position: Optional[Tuple[float, float, float]] = None
        self.test_start_time: Optional[float] = None
        self.emergency_stop = False
        self.joystick_connected = False
        
        # Test configuration
        self.test_velocities = {
            'linear_x': [0.1, 0.2, 0.3, 0.5, 0.8],  # m/s forward/backward
            'linear_y': [0.1, 0.2, 0.3, 0.5, 0.8],  # m/s left/right strafe
            'angular_z': [0.2, 0.5, 0.8, 1.0, 1.5]  # rad/s rotation
        }
        self.test_duration = 5.0  # seconds per test
        self.stabilization_time = 2.0  # seconds to wait between tests
        
        # Results storage
        self.all_results: List[VelocityTestResult] = []
        self.calibration_results: List[CalibrationResults] = []
        
        # Create output directory
        self.output_dir = '/home/kaylanw4/ros2_ws/calibration_results'
        os.makedirs(self.output_dir, exist_ok=True)
        
        self.get_logger().info(f"{Colors.GREEN}🚀 Professional Velocity Calibration Tool Initialized{Colors.END}")
        self.get_logger().info(f"{Colors.CYAN}📁 Results will be saved to: {self.output_dir}{Colors.END}")
        
    def odom_callback(self, msg: Odometry):
        """Store latest odometry data"""
        self.current_odom = msg
        
    def joy_callback(self, msg: Joy):
        """Monitor joystick for emergency stop (PS4 controller)"""
        self.joystick_connected = True
        # PS4 controller: button 8 is Share, button 9 is Options
        # Emergency stop if both pressed simultaneously
        if len(msg.buttons) > 9 and msg.buttons[8] and msg.buttons[9]:
            self.emergency_stop = True
            self.get_logger().error(f"{Colors.RED}🚨 EMERGENCY STOP ACTIVATED via joystick{Colors.END}")
            self.stop_robot()
    
    def stop_robot(self):
        """Immediately stop the robot"""
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)
        
    def get_current_position(self) -> Optional[Tuple[float, float, float]]:
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
    
    def wait_for_odometry(self, timeout: float = 10.0) -> bool:
        """Wait for odometry data to be available"""
        self.get_logger().info(f"{Colors.YELLOW}⏳ Waiting for odometry data...{Colors.END}")
        
        start_time = time.time()
        rate = self.create_rate(10)  # 10 Hz
        
        while rclpy.ok() and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_odom is not None:
                self.get_logger().info(f"{Colors.GREEN}✅ Odometry data received{Colors.END}")
                return True
            rate.sleep()
        
        self.get_logger().error(f"{Colors.RED}❌ Timeout waiting for odometry data{Colors.END}")
        return False
    
    def calculate_displacement(self, start_pos: Tuple[float, float, float], 
                              end_pos: Tuple[float, float, float], 
                              motion_type: str) -> float:
        """Calculate displacement based on motion type"""
        x1, y1, theta1 = start_pos
        x2, y2, theta2 = end_pos
        
        if motion_type == 'linear_x':
            # Forward/backward displacement
            return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        elif motion_type == 'linear_y':
            # Strafe displacement (should be perpendicular to forward direction)
            return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        elif motion_type == 'angular_z':
            # Angular displacement
            delta_theta = theta2 - theta1
            # Normalize angle to [-pi, pi]
            while delta_theta > math.pi:
                delta_theta -= 2 * math.pi
            while delta_theta < -math.pi:
                delta_theta += 2 * math.pi
            return abs(delta_theta)
        else:
            return 0.0
    
    def execute_velocity_test(self, velocity: float, motion_type: str, 
                             test_number: int, total_tests: int) -> VelocityTestResult:
        """Execute a single velocity test"""
        
        self.get_logger().info(f"{Colors.BOLD}{Colors.BLUE}🧪 Test {test_number}/{total_tests}: "
                              f"{motion_type.replace('_', ' ').title()} at {velocity} "
                              f"{'m/s' if 'linear' in motion_type else 'rad/s'}{Colors.END}")
        
        # Wait for stabilization
        self.get_logger().info(f"{Colors.YELLOW}⏳ Stabilizing for {self.stabilization_time}s...{Colors.END}")
        time.sleep(self.stabilization_time)
        
        # Check for emergency stop
        if self.emergency_stop:
            raise RuntimeError("Emergency stop activated")
        
        # Get starting position
        start_pos = self.get_current_position()
        if start_pos is None:
            raise RuntimeError("Could not get starting position")
        
        self.get_logger().info(f"{Colors.CYAN}📍 Start position: X={start_pos[0]:.3f}m, "
                              f"Y={start_pos[1]:.3f}m, Θ={math.degrees(start_pos[2]):.1f}°{Colors.END}")
        
        # Create and publish velocity command
        cmd = Twist()
        if motion_type == 'linear_x':
            cmd.linear.x = velocity
        elif motion_type == 'linear_y':
            cmd.linear.y = velocity
        elif motion_type == 'angular_z':
            cmd.angular.z = velocity
        
        # Record test start time
        test_start_time = time.time()
        
        self.get_logger().info(f"{Colors.GREEN}🚀 Executing motion for {self.test_duration}s...{Colors.END}")
        
        # Execute motion for specified duration
        end_time = test_start_time + self.test_duration
        rate = self.create_rate(20)  # 20 Hz command rate
        
        while time.time() < end_time and not self.emergency_stop:
            self.cmd_vel_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.01)
            rate.sleep()
        
        # Stop the robot
        self.stop_robot()
        
        # Small delay to ensure robot stops
        time.sleep(0.5)
        
        # Get ending position
        end_pos = self.get_current_position()
        if end_pos is None:
            raise RuntimeError("Could not get ending position")
        
        actual_duration = time.time() - test_start_time
        
        self.get_logger().info(f"{Colors.CYAN}📍 End position: X={end_pos[0]:.3f}m, "
                              f"Y={end_pos[1]:.3f}m, Θ={math.degrees(end_pos[2]):.1f}°{Colors.END}")
        
        # Calculate results
        displacement = self.calculate_displacement(start_pos, end_pos, motion_type)
        measured_velocity = displacement / actual_duration
        velocity_error = measured_velocity - abs(velocity)  # Use abs for bidirectional motion
        velocity_error_percent = (velocity_error / abs(velocity)) * 100 if velocity != 0 else 0
        
        self.get_logger().info(f"{Colors.MAGENTA}📊 Results: Displacement={displacement:.3f}, "
                              f"Measured Velocity={measured_velocity:.3f}, "
                              f"Error={velocity_error_percent:.1f}%{Colors.END}")
        
        # Create test result
        result = VelocityTestResult(
            test_name=f"{motion_type}_{velocity}",
            commanded_velocity=velocity,
            measured_velocity=measured_velocity,
            duration=actual_duration,
            start_position=start_pos,
            end_position=end_pos,
            displacement=displacement,
            velocity_error=velocity_error,
            velocity_error_percent=velocity_error_percent,
            timestamp=datetime.now().isoformat()
        )
        
        return result
    
    def analyze_calibration_results(self, motion_type: str, 
                                   results: List[VelocityTestResult]) -> CalibrationResults:
        """Perform statistical analysis and calculate calibration factors"""
        
        self.get_logger().info(f"{Colors.BOLD}{Colors.BLUE}📈 Analyzing {motion_type.replace('_', ' ').title()} "
                              f"calibration results...{Colors.END}")
        
        if not results:
            raise ValueError("No results to analyze")
        
        # Extract data for analysis
        commanded_vels = [abs(r.commanded_velocity) for r in results]
        measured_vels = [r.measured_velocity for r in results]
        errors_percent = [r.velocity_error_percent for r in results]
        
        # Calculate statistics
        mean_error_percent = np.mean(errors_percent)
        std_error_percent = np.std(errors_percent)
        
        # Linear regression to find scale factor
        slope, intercept, r_value, p_value, std_err = stats.linregress(commanded_vels, measured_vels)
        r_squared = r_value ** 2
        
        # Recommended scale factor is 1/slope (to correct for systematic error)
        recommended_scale_factor = 1.0 / slope if slope != 0 else 1.0
        
        self.get_logger().info(f"{Colors.GREEN}📊 Analysis Results:{Colors.END}")
        self.get_logger().info(f"   Mean Error: {mean_error_percent:.2f}% ± {std_error_percent:.2f}%")
        self.get_logger().info(f"   R² Correlation: {r_squared:.4f}")
        self.get_logger().info(f"   Recommended Scale Factor: {recommended_scale_factor:.4f}")
        
        return CalibrationResults(
            test_type=motion_type,
            mean_error_percent=mean_error_percent,
            std_error_percent=std_error_percent,
            recommended_scale_factor=recommended_scale_factor,
            r_squared=r_squared,
            test_results=results
        )
    
    def save_results_to_csv(self, filename: str):
        """Save all test results to CSV file"""
        
        filepath = os.path.join(self.output_dir, filename)
        
        with open(filepath, 'w', newline='') as csvfile:
            fieldnames = [
                'test_name', 'commanded_velocity', 'measured_velocity', 'duration',
                'start_x', 'start_y', 'start_theta', 'end_x', 'end_y', 'end_theta',
                'displacement', 'velocity_error', 'velocity_error_percent', 'timestamp'
            ]
            
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            
            for result in self.all_results:
                writer.writerow({
                    'test_name': result.test_name,
                    'commanded_velocity': result.commanded_velocity,
                    'measured_velocity': result.measured_velocity,
                    'duration': result.duration,
                    'start_x': result.start_position[0],
                    'start_y': result.start_position[1],
                    'start_theta': result.start_position[2],
                    'end_x': result.end_position[0],
                    'end_y': result.end_position[1],
                    'end_theta': result.end_position[2],
                    'displacement': result.displacement,
                    'velocity_error': result.velocity_error,
                    'velocity_error_percent': result.velocity_error_percent,
                    'timestamp': result.timestamp
                })
        
        self.get_logger().info(f"{Colors.GREEN}💾 Detailed results saved to: {filepath}{Colors.END}")
    
    def generate_calibration_report(self):
        """Generate a comprehensive calibration report"""
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_filename = f"calibration_report_{timestamp}.txt"
        report_path = os.path.join(self.output_dir, report_filename)
        
        with open(report_path, 'w') as f:
            f.write("="*80 + "\n")
            f.write("MECANUM ROBOT VELOCITY CALIBRATION REPORT\n")
            f.write("="*80 + "\n")
            f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Total Tests Performed: {len(self.all_results)}\n\n")
            
            # Summary for each motion type
            for cal_result in self.calibration_results:
                f.write(f"\n{cal_result.test_type.replace('_', ' ').upper()} CALIBRATION\n")
                f.write("-" * 40 + "\n")
                f.write(f"Mean Velocity Error: {cal_result.mean_error_percent:.2f}% "
                       f"± {cal_result.std_error_percent:.2f}%\n")
                f.write(f"R² Correlation: {cal_result.r_squared:.4f}\n")
                f.write(f"Recommended Scale Factor: {cal_result.recommended_scale_factor:.4f}\n")
                
                f.write("\nIndividual Test Results:\n")
                for result in cal_result.test_results:
                    f.write(f"  {result.commanded_velocity:>6.2f} -> {result.measured_velocity:>6.3f} "
                           f"({result.velocity_error_percent:>+6.1f}%)\n")
            
            # Configuration recommendations
            f.write("\n" + "="*80 + "\n")
            f.write("HARDWARE CONFIGURATION RECOMMENDATIONS\n")
            f.write("="*80 + "\n")
            f.write("\nAdd the following parameters to your hardware.yaml:\n\n")
            
            for cal_result in self.calibration_results:
                param_name = f"velocity_scale_{cal_result.test_type.split('_')[1]}"
                f.write(f"{param_name}: {cal_result.recommended_scale_factor:.4f}\n")
            
            f.write("\nExample configuration:\n")
            f.write("enhanced_yahboom_driver:\n")
            f.write("  ros__parameters:\n")
            for cal_result in self.calibration_results:
                param_name = f"velocity_scale_{cal_result.test_type.split('_')[1]}"
                f.write(f"    {param_name}: {cal_result.recommended_scale_factor:.4f}\n")
        
        self.get_logger().info(f"{Colors.GREEN}📋 Calibration report saved to: {report_path}{Colors.END}")
        
        # Also print key results to console
        self.print_summary_results()
    
    def print_summary_results(self):
        """Print summary results to console"""
        
        print(f"\n{Colors.BOLD}{Colors.GREEN}🎯 CALIBRATION SUMMARY{Colors.END}")
        print("="*60)
        
        for cal_result in self.calibration_results:
            motion_name = cal_result.test_type.replace('_', ' ').title()
            print(f"\n{Colors.BOLD}{motion_name}:{Colors.END}")
            print(f"  Mean Error: {Colors.YELLOW}{cal_result.mean_error_percent:+6.2f}%{Colors.END} "
                  f"(±{cal_result.std_error_percent:.2f}%)")
            print(f"  Correlation: {Colors.CYAN}R² = {cal_result.r_squared:.4f}{Colors.END}")
            print(f"  Scale Factor: {Colors.MAGENTA}{cal_result.recommended_scale_factor:.4f}{Colors.END}")
        
        print(f"\n{Colors.BOLD}{Colors.BLUE}📁 Files Generated:{Colors.END}")
        print(f"  • Detailed CSV data")
        print(f"  • Calibration report with config recommendations")
        print(f"  • All files saved to: {self.output_dir}")
        
        print(f"\n{Colors.BOLD}{Colors.GREEN}✅ Calibration Complete!{Colors.END}")
    
    def run_full_calibration(self):
        """Execute the complete calibration sequence"""
        
        try:
            # Wait for odometry
            if not self.wait_for_odometry():
                return False
            
            self.get_logger().info(f"{Colors.BOLD}{Colors.GREEN}🚀 Starting Full Velocity Calibration{Colors.END}")
            
            if self.joystick_connected:
                self.get_logger().info(f"{Colors.YELLOW}🎮 Joystick connected - Press Share+Options for emergency stop{Colors.END}")
            
            # Calculate total number of tests
            total_tests = sum(len(velocities) for velocities in self.test_velocities.values())
            current_test = 0
            
            # Run tests for each motion type
            for motion_type, velocities in self.test_velocities.items():
                
                self.get_logger().info(f"\n{Colors.BOLD}{Colors.BLUE}🧪 Starting {motion_type.replace('_', ' ').title()} "
                                      f"Calibration Tests{Colors.END}")
                
                motion_results = []
                
                for velocity in velocities:
                    current_test += 1
                    
                    # Test positive velocity
                    result_pos = self.execute_velocity_test(velocity, motion_type, current_test, total_tests)
                    motion_results.append(result_pos)
                    self.all_results.append(result_pos)
                    
                    # For linear motions, also test negative velocity
                    if motion_type.startswith('linear'):
                        current_test += 1
                        result_neg = self.execute_velocity_test(-velocity, motion_type, current_test, total_tests)
                        motion_results.append(result_neg)
                        self.all_results.append(result_neg)
                
                # Analyze results for this motion type
                calibration_result = self.analyze_calibration_results(motion_type, motion_results)
                self.calibration_results.append(calibration_result)
            
            # Save results
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            csv_filename = f"velocity_calibration_data_{timestamp}.csv"
            self.save_results_to_csv(csv_filename)
            
            # Generate report
            self.generate_calibration_report()
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"{Colors.RED}❌ Calibration failed: {str(e)}{Colors.END}")
            self.stop_robot()
            return False


def main():
    """Main function to run the calibration tool"""
    
    print(f"{Colors.BOLD}{Colors.BLUE}")
    print("="*80)
    print("          PROFESSIONAL MECANUM ROBOT VELOCITY CALIBRATION")
    print("="*80)
    print(f"{Colors.END}")
    print(f"{Colors.GREEN}🤖 This tool will calibrate velocity control for all 3 degrees of freedom:{Colors.END}")
    print(f"   • {Colors.CYAN}Linear X:{Colors.END} Forward/backward motion")
    print(f"   • {Colors.CYAN}Linear Y:{Colors.END} Left/right strafe motion") 
    print(f"   • {Colors.CYAN}Angular Z:{Colors.END} Rotation around vertical axis")
    print()
    print(f"{Colors.BLUE}🔧 Enhanced System Compatibility:{Colors.END}")
    print("   • Works with enhanced_robot_bringup.launch.py")
    print("   • Bypasses velocity smoother for direct hardware calibration")
    print("   • Requires joystick disabled during calibration")
    print()
    print(f"{Colors.YELLOW}⚠️  SAFETY REQUIREMENTS:{Colors.END}")
    print("   • Ensure robot has at least 5x5 meter clear space")
    print("   • Keep emergency stop (joystick Share+Options) ready")
    print("   • Monitor robot during all tests")
    print("   • Ensure robot is on level ground")
    print()
    
    # Get user confirmation
    try:
        response = input(f"{Colors.BOLD}Continue with calibration? (y/N): {Colors.END}")
        if response.lower() not in ['y', 'yes']:
            print(f"{Colors.YELLOW}Calibration cancelled by user{Colors.END}")
            return
    except KeyboardInterrupt:
        print(f"\n{Colors.YELLOW}Calibration cancelled by user{Colors.END}")
        return
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create calibration node
        calibration_tool = VelocityCalibrationTool()
        
        # Run calibration
        success = calibration_tool.run_full_calibration()
        
        if success:
            print(f"\n{Colors.BOLD}{Colors.GREEN}🎉 Calibration completed successfully!{Colors.END}")
        else:
            print(f"\n{Colors.BOLD}{Colors.RED}❌ Calibration failed{Colors.END}")
            
    except KeyboardInterrupt:
        print(f"\n{Colors.YELLOW}🛑 Calibration interrupted by user{Colors.END}")
    except Exception as e:
        print(f"\n{Colors.RED}❌ Calibration error: {str(e)}{Colors.END}")
    finally:
        # Cleanup
        try:
            calibration_tool.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()