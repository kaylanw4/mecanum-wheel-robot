#!/usr/bin/env python3
"""
Motor Calibration Tool for Yahboom Robot
Helps identify and correct motor speed differences for precise control
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import time
import math
import threading
from statistics import mean


class MotorCalibrationTool(Node):
    """
    Tool to calibrate motor speeds and identify correction factors
    """
    
    def __init__(self):
        super().__init__('motor_calibration_tool')
        
        self.get_logger().info("🔧 Motor Calibration Tool Started")
        self.get_logger().info("=" * 60)
        self.get_logger().info("This tool will help you calibrate your robot's motors")
        self.get_logger().info("for equal speeds and straight-line movement.")
        self.get_logger().info("=" * 60)
        
        # Data storage
        self.joint_data = []
        self.odom_data = []
        self.is_collecting = False
        self.test_duration = 5.0  # seconds
        
        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
            
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Wait for connections
        self.get_logger().info("📡 Waiting for robot connections...")
        time.sleep(2.0)
        
        # Start calibration sequence
        self.run_calibration_sequence()
        
    def joint_callback(self, msg):
        """Store joint state data during tests"""
        if self.is_collecting:
            # Calculate wheel speeds from velocities
            wheel_speeds = {
                'front_left': abs(msg.velocity[0]) if len(msg.velocity) > 0 else 0.0,
                'front_right': abs(msg.velocity[1]) if len(msg.velocity) > 1 else 0.0,
                'rear_left': abs(msg.velocity[2]) if len(msg.velocity) > 2 else 0.0,
                'rear_right': abs(msg.velocity[3]) if len(msg.velocity) > 3 else 0.0,
                'timestamp': time.time()
            }
            self.joint_data.append(wheel_speeds)
            
    def odom_callback(self, msg):
        """Store odometry data during tests"""
        if self.is_collecting:
            odom_info = {
                'linear_x': msg.twist.twist.linear.x,
                'linear_y': msg.twist.twist.linear.y,
                'angular_z': msg.twist.twist.angular.z,
                'position_x': msg.pose.pose.position.x,
                'position_y': msg.pose.pose.position.y,
                'timestamp': time.time()
            }
            self.odom_data.append(odom_info)
            
    def send_velocity_command(self, vx=0.0, vy=0.0, vz=0.0):
        """Send velocity command to robot"""
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = vz
        self.cmd_vel_pub.publish(cmd)
        
    def stop_robot(self):
        """Stop the robot"""
        self.send_velocity_command(0.0, 0.0, 0.0)
        time.sleep(0.5)
        
    def collect_data_for_duration(self, duration):
        """Collect data for specified duration"""
        self.joint_data.clear()
        self.odom_data.clear()
        self.is_collecting = True
        
        time.sleep(duration)
        
        self.is_collecting = False
        return self.joint_data.copy(), self.odom_data.copy()
        
    def analyze_wheel_speeds(self, joint_data):
        """Analyze wheel speed data and calculate averages"""
        if not joint_data:
            return None
            
        # Calculate average speeds for each wheel
        fl_speeds = [d['front_left'] for d in joint_data if d['front_left'] > 0]
        fr_speeds = [d['front_right'] for d in joint_data if d['front_right'] > 0]
        rl_speeds = [d['rear_left'] for d in joint_data if d['rear_left'] > 0]
        rr_speeds = [d['rear_right'] for d in joint_data if d['rear_right'] > 0]
        
        if not all([fl_speeds, fr_speeds, rl_speeds, rr_speeds]):
            return None
            
        avg_speeds = {
            'front_left': mean(fl_speeds),
            'front_right': mean(fr_speeds),
            'rear_left': mean(rl_speeds),
            'rear_right': mean(rr_speeds)
        }
        
        return avg_speeds
        
    def calculate_calibration_factors(self, avg_speeds):
        """Calculate calibration factors to equalize wheel speeds"""
        if not avg_speeds:
            return None
            
        # Find the minimum speed as reference
        min_speed = min(avg_speeds.values())
        
        # Calculate factors to bring all wheels to the same speed
        factors = {
            wheel: min_speed / speed if speed > 0 else 1.0
            for wheel, speed in avg_speeds.items()
        }
        
        return factors
        
    def analyze_movement_drift(self, odom_data):
        """Analyze how much the robot drifts during forward movement"""
        if len(odom_data) < 2:
            return None
            
        start_pos = odom_data[0]
        end_pos = odom_data[-1]
        
        # Calculate drift
        dx = end_pos['position_x'] - start_pos['position_x']
        dy = end_pos['position_y'] - start_pos['position_y']
        
        drift_distance = math.sqrt(dx**2 + dy**2)
        drift_angle = math.atan2(dy, dx) * 180 / math.pi
        
        return {
            'distance': drift_distance,
            'angle': drift_angle,
            'lateral_drift': abs(dy),
            'forward_distance': dx
        }
        
    def run_calibration_sequence(self):
        """Run the complete calibration sequence"""
        
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("🚀 STARTING MOTOR CALIBRATION SEQUENCE")
        self.get_logger().info("="*60)
        
        input("\n⚠️  SAFETY: Make sure robot is on the ground with clear space around it.")
        input("Press ENTER when ready to start...")
        
        # Test 1: Forward movement
        self.get_logger().info("\n📊 TEST 1: Forward Movement Analysis")
        self.get_logger().info("-" * 40)
        self.get_logger().info(f"🔄 Commanding robot to move forward at 0.3 m/s for {self.test_duration} seconds...")
        
        self.send_velocity_command(0.3, 0.0, 0.0)
        joint_data, odom_data = self.collect_data_for_duration(self.test_duration)
        self.stop_robot()
        
        # Analyze results
        avg_speeds = self.analyze_wheel_speeds(joint_data)
        drift_analysis = self.analyze_movement_drift(odom_data)
        
        if avg_speeds:
            self.get_logger().info("\n📈 WHEEL SPEED ANALYSIS:")
            self.get_logger().info(f"  Front Left:  {avg_speeds['front_left']:.3f} rad/s")
            self.get_logger().info(f"  Front Right: {avg_speeds['front_right']:.3f} rad/s") 
            self.get_logger().info(f"  Rear Left:   {avg_speeds['rear_left']:.3f} rad/s")
            self.get_logger().info(f"  Rear Right:  {avg_speeds['rear_right']:.3f} rad/s")
            
            # Calculate speed differences
            speeds = list(avg_speeds.values())
            speed_range = max(speeds) - min(speeds)
            speed_variation = (speed_range / mean(speeds)) * 100
            
            self.get_logger().info(f"\n📊 SPEED VARIATION: {speed_variation:.1f}%")
            if speed_variation > 10:
                self.get_logger().warn("⚠️  HIGH speed variation detected! Calibration needed.")
            else:
                self.get_logger().info("✅ Speed variation within acceptable range.")
        
        if drift_analysis:
            self.get_logger().info(f"\n🎯 MOVEMENT DRIFT ANALYSIS:")
            self.get_logger().info(f"  Forward distance: {drift_analysis['forward_distance']:.3f} m")
            self.get_logger().info(f"  Lateral drift:    {drift_analysis['lateral_drift']:.3f} m")
            self.get_logger().info(f"  Drift angle:      {drift_analysis['angle']:.1f}°")
            
            if drift_analysis['lateral_drift'] > 0.1:
                self.get_logger().warn("⚠️  SIGNIFICANT lateral drift detected!")
                if drift_analysis['angle'] > 0:
                    self.get_logger().info("   → Robot drifts LEFT (right motors faster)")
                else:
                    self.get_logger().info("   → Robot drifts RIGHT (left motors faster)")
            else:
                self.get_logger().info("✅ Lateral drift within acceptable range.")
        
        # Calculate calibration factors
        if avg_speeds:
            factors = self.calculate_calibration_factors(avg_speeds)
            
            self.get_logger().info("\n🔧 RECOMMENDED CALIBRATION FACTORS:")
            self.get_logger().info("Add these to your hardware.yaml file:")
            self.get_logger().info("")
            self.get_logger().info("enhanced_yahboom_driver:")
            self.get_logger().info("  ros__parameters:")
            self.get_logger().info(f"    motor_cal_fl: {factors['front_left']:.3f}  # Front Left")
            self.get_logger().info(f"    motor_cal_fr: {factors['front_right']:.3f}  # Front Right") 
            self.get_logger().info(f"    motor_cal_rl: {factors['rear_left']:.3f}  # Rear Left")
            self.get_logger().info(f"    motor_cal_rr: {factors['rear_right']:.3f}  # Rear Right")
        
        # Test 2: Rotational movement
        self.get_logger().info("\n" + "-"*60)
        input("Press ENTER to test rotational movement...")
        
        self.get_logger().info("\n📊 TEST 2: Rotational Movement Analysis")
        self.get_logger().info("-" * 40)
        self.get_logger().info(f"🔄 Commanding robot to rotate at 0.5 rad/s for {self.test_duration} seconds...")
        
        self.send_velocity_command(0.0, 0.0, 0.5)
        joint_data_rot, odom_data_rot = self.collect_data_for_duration(self.test_duration)
        self.stop_robot()
        
        # Analyze rotational results
        avg_speeds_rot = self.analyze_wheel_speeds(joint_data_rot)
        
        if avg_speeds_rot:
            self.get_logger().info("\n📈 ROTATIONAL WHEEL SPEEDS:")
            self.get_logger().info(f"  Front Left:  {avg_speeds_rot['front_left']:.3f} rad/s")
            self.get_logger().info(f"  Front Right: {avg_speeds_rot['front_right']:.3f} rad/s")
            self.get_logger().info(f"  Rear Left:   {avg_speeds_rot['rear_left']:.3f} rad/s") 
            self.get_logger().info(f"  Rear Right:  {avg_speeds_rot['rear_right']:.3f} rad/s")
            
            # For rotation, opposite wheels should have similar speeds
            left_avg = (avg_speeds_rot['front_left'] + avg_speeds_rot['rear_left']) / 2
            right_avg = (avg_speeds_rot['front_right'] + avg_speeds_rot['rear_right']) / 2
            
            rotation_balance = abs(left_avg - right_avg) / max(left_avg, right_avg) * 100
            self.get_logger().info(f"\n🔄 ROTATION BALANCE: {rotation_balance:.1f}% difference")
            
            if rotation_balance > 15:
                self.get_logger().warn("⚠️  UNBALANCED rotation detected!")
            else:
                self.get_logger().info("✅ Rotation balance within acceptable range.")
        
        # Final recommendations
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("📋 CALIBRATION COMPLETE - RECOMMENDATIONS")
        self.get_logger().info("="*60)
        
        if speed_variation > 10 or (drift_analysis and drift_analysis['lateral_drift'] > 0.1):
            self.get_logger().info("🔧 CALIBRATION NEEDED:")
            self.get_logger().info("1. Update your hardware.yaml with the recommended calibration factors")
            self.get_logger().info("2. Restart your robot driver")
            self.get_logger().info("3. Re-run this calibration tool to verify improvements")
            self.get_logger().info("4. Fine-tune factors if needed")
        else:
            self.get_logger().info("✅ ROBOT IS WELL CALIBRATED:")
            self.get_logger().info("Your robot shows good speed matching and minimal drift.")
            self.get_logger().info("No calibration adjustments are needed at this time.")
        
        self.get_logger().info("\n🎯 NEXT STEPS FOR NAV2 COMPATIBILITY:")
        self.get_logger().info("1. Ensure calibration factors are applied")
        self.get_logger().info("2. Test with teleop for smooth control")
        self.get_logger().info("3. Verify odometry accuracy over longer distances")
        self.get_logger().info("4. Ready for ZED 2i camera integration and SLAM!")
        
        self.get_logger().info("\n" + "="*60)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        calibration_tool = MotorCalibrationTool()
        # Don't spin - the tool runs its sequence and exits
        
    except KeyboardInterrupt:
        print("\n🛑 Calibration interrupted by user")
    except Exception as e:
        print(f"❌ Calibration failed: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()