#!/usr/bin/env python3
"""
Motor Calibration Diagnostic Tool for Yahboom Mecanum Robot
Focuses on detecting small calibration differences, not mapping issues
Uses higher velocities to overcome motor deadband
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import time
import math

class CalibrationDiagnostic(Node):
    def __init__(self):
        super().__init__('calibration_diagnostic')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)
        
        # Data storage
        self.joint_positions = {}
        self.initial_positions = {}
        self.position_history = []
        
        # Test configuration - HIGHER VALUES to overcome deadband
        self.test_configs = [
            {"name": "Forward", "vx": 0.5, "vy": 0.0, "vz": 0.0, 
             "expected": "All wheels forward, check for drift"},
            {"name": "Backward", "vx": -0.5, "vy": 0.0, "vz": 0.0,
             "expected": "All wheels backward, check for drift"},
            {"name": "Strafe Right", "vx": 0.0, "vy": -0.5, "vz": 0.0,
             "expected": "FL+RR backward, FR+RL forward"},
            {"name": "Strafe Left", "vx": 0.0, "vy": 0.5, "vz": 0.0,
             "expected": "FL+RR forward, FR+RL backward"},
            {"name": "Rotate Right", "vx": 0.0, "vy": 0.0, "vz": -0.8,
             "expected": "FL+RL forward, FR+RR backward"},
            {"name": "Rotate Left", "vx": 0.0, "vy": 0.0, "vz": 0.8,
             "expected": "FL+RL backward, FR+RR forward"}
        ]
        
        # Diagnostic state
        self.current_test = 0
        self.test_phase = "waiting"  # waiting, moving, analyzing
        self.phase_start_time = time.time()
        self.test_duration = 3.0  # Longer test for better data
        self.settle_time = 2.0    # Time to settle between tests
        
        self.get_logger().info("🔧 Motor CALIBRATION Diagnostic Started")
        self.get_logger().info("🎯 Focus: Detecting small motor differences, not mapping issues")
        self.get_logger().info(f"📊 Will run {len(self.test_configs)} calibration tests")
        self.get_logger().info("⚡ Using higher velocities to overcome motor deadband")
        
        # Create timer
        self.diagnostic_timer = self.create_timer(0.1, self.run_calibration_tests)
        
    def joint_state_callback(self, msg):
        """Store joint state data"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]
        
        # Record position history
        current_time = time.time()
        self.position_history.append({
            'time': current_time,
            'positions': self.joint_positions.copy()
        })
        
        # Keep recent history
        self.position_history = [
            h for h in self.position_history 
            if current_time - h['time'] < 20.0
        ]
    
    def send_velocity_command(self, vx=0.0, vy=0.0, vz=0.0):
        """Send velocity command"""
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = vz
        self.cmd_vel_pub.publish(cmd)
        
    def stop_robot(self):
        """Stop the robot"""
        self.send_velocity_command(0.0, 0.0, 0.0)
        
    def record_initial_positions(self):
        """Record initial wheel positions"""
        if self.joint_positions:
            self.initial_positions = self.joint_positions.copy()
            self.get_logger().info("📊 Recorded initial positions")
            return True
        else:
            self.get_logger().warn("⚠️  No joint positions available")
            return False
    
    def analyze_calibration(self, test_config):
        """Analyze motor calibration for this test"""
        if not self.initial_positions or not self.joint_positions:
            self.get_logger().warn(f"❌ Cannot analyze {test_config['name']} - missing position data")
            return
            
        # Calculate movements
        movements = {}
        for joint, current_pos in self.joint_positions.items():
            if joint in self.initial_positions:
                movements[joint] = current_pos - self.initial_positions[joint]
        
        self.get_logger().info(f"🔍 {test_config['name']} Calibration Analysis:")
        self.get_logger().info(f"Expected: {test_config['expected']}")
        
        # Group wheels by expected behavior
        wheel_groups = self.classify_wheel_movements(test_config, movements)
        
        # Analyze each group
        self.analyze_wheel_groups(test_config['name'], wheel_groups, movements)
        
    def classify_wheel_movements(self, test_config, movements):
        """Classify wheels by expected movement direction"""
        # Map joint names to wheel positions
        wheel_map = {
            'front_left_wheel_joint': 'FL',
            'front_right_wheel_joint': 'FR', 
            'rear_left_wheel_joint': 'RL',
            'rear_right_wheel_joint': 'RR'
        }
        
        # Determine expected movement for each wheel based on test
        vx, vy, vz = test_config['vx'], test_config['vy'], test_config['vz']
        
        # Mecanum wheel kinematics (forward kinematics)
        # For each wheel: wheel_speed = vx ± vy ± vz*(lx+ly)
        expected_directions = {}
        
        if abs(vx) > 0.1:  # Forward/backward test
            # All wheels should move same direction
            for joint in movements.keys():
                expected_directions[joint] = "forward" if vx > 0 else "backward"
                
        elif abs(vy) > 0.1:  # Strafe test
            # FL+RR move opposite to FR+RL
            for joint, wheel in wheel_map.items():
                if joint in movements:
                    if wheel in ['FL', 'RR']:
                        expected_directions[joint] = "backward" if vy > 0 else "forward"
                    else:  # FR, RL
                        expected_directions[joint] = "forward" if vy > 0 else "backward"
                        
        elif abs(vz) > 0.1:  # Rotation test
            # FL+RL move opposite to FR+RR
            for joint, wheel in wheel_map.items():
                if joint in movements:
                    if wheel in ['FL', 'RL']:
                        expected_directions[joint] = "backward" if vz > 0 else "forward"
                    else:  # FR, RR
                        expected_directions[joint] = "forward" if vz > 0 else "backward"
        
        return expected_directions
    
    def analyze_wheel_groups(self, test_name, expected_directions, movements):
        """Analyze calibration within wheel groups"""
        
        # Print individual wheel movements
        for joint, movement in movements.items():
            direction = "→" if movement > 0.005 else "←" if movement < -0.005 else "·"
            expected = expected_directions.get(joint, "unknown")
            self.get_logger().info(f"  {joint}: {movement:.4f} rad {direction} (expected: {expected})")
        
        # Group wheels by expected direction
        forward_group = []
        backward_group = []
        
        for joint, movement in movements.items():
            expected = expected_directions.get(joint, "unknown")
            if expected == "forward":
                forward_group.append((joint, movement))
            elif expected == "backward":
                backward_group.append((joint, movement))
        
        # Analyze each group for calibration differences
        self.analyze_group_calibration("Forward-moving wheels", forward_group)
        self.analyze_group_calibration("Backward-moving wheels", backward_group)
        
        # Check for cross-group issues (wheels moving wrong direction)
        self.check_wrong_directions(expected_directions, movements)
        
    def analyze_group_calibration(self, group_name, wheel_group):
        """Analyze calibration differences within a group of wheels"""
        if len(wheel_group) < 2:
            return
            
        movements = [movement for _, movement in wheel_group]
        avg_movement = sum(movements) / len(movements)
        
        self.get_logger().info(f"📊 {group_name} calibration:")
        self.get_logger().info(f"   Average movement: {avg_movement:.4f} rad")
        
        # Calculate deviations
        deviations = []
        for joint, movement in wheel_group:
            deviation = movement - avg_movement
            deviation_percent = (deviation / avg_movement * 100) if abs(avg_movement) > 0.001 else 0
            deviations.append(abs(deviation))
            
            status = "✅" if abs(deviation) < 0.01 else "⚠️" if abs(deviation) < 0.02 else "🚨"
            self.get_logger().info(f"   {joint}: {deviation:+.4f} rad ({deviation_percent:+.1f}%) {status}")
        
        # Overall assessment
        max_deviation = max(deviations) if deviations else 0
        if max_deviation < 0.01:
            self.get_logger().info(f"✅ {group_name}: Good calibration (max dev: {max_deviation:.4f})")
        elif max_deviation < 0.02:
            self.get_logger().warn(f"⚠️  {group_name}: Minor calibration issue (max dev: {max_deviation:.4f})")
        else:
            self.get_logger().error(f"🚨 {group_name}: Significant calibration issue (max dev: {max_deviation:.4f})")
            
        # Suggest calibration adjustments
        if max_deviation > 0.01:
            self.suggest_calibration_fix(wheel_group, avg_movement)
            
    def suggest_calibration_fix(self, wheel_group, avg_movement):
        """Suggest motor calibration adjustments"""
        self.get_logger().info("🔧 Suggested calibration adjustments:")
        
        for joint, movement in wheel_group:
            if abs(avg_movement) > 0.001:
                calibration_factor = avg_movement / movement if abs(movement) > 0.001 else 1.0
                motor_param = self.joint_to_motor_param(joint)
                self.get_logger().info(f"   {motor_param}: {calibration_factor:.3f}")
                
    def joint_to_motor_param(self, joint_name):
        """Map joint name to motor calibration parameter"""
        mapping = {
            'front_left_wheel_joint': 'motor_calibration_fl',
            'front_right_wheel_joint': 'motor_calibration_fr',
            'rear_left_wheel_joint': 'motor_calibration_rl', 
            'rear_right_wheel_joint': 'motor_calibration_rr'
        }
        return mapping.get(joint_name, joint_name)
        
    def check_wrong_directions(self, expected_directions, movements):
        """Check for wheels moving in wrong directions"""
        wrong_direction_count = 0
        
        for joint, movement in movements.items():
            expected = expected_directions.get(joint, "unknown")
            actual_direction = "forward" if movement > 0.005 else "backward" if movement < -0.005 else "none"
            
            if expected != "unknown" and expected != actual_direction and actual_direction != "none":
                self.get_logger().error(f"🚨 {joint}: Moving {actual_direction}, expected {expected}")
                wrong_direction_count += 1
                
        if wrong_direction_count == 0:
            self.get_logger().info("✅ All wheels moving in correct directions")
        else:
            self.get_logger().error(f"🚨 {wrong_direction_count} wheels moving in wrong directions!")
            
    def run_calibration_tests(self):
        """Run calibration test sequence"""
        if self.current_test >= len(self.test_configs):
            self.get_logger().info("🎉 All calibration tests completed!")
            self.get_logger().info("")
            self.get_logger().info("📋 Summary:")
            self.get_logger().info("- Check logs above for calibration suggestions")
            self.get_logger().info("- Update motor_calibration parameters in config file")
            self.get_logger().info("- Re-run tests to verify improvements")
            return
            
        current_time = time.time()
        elapsed = current_time - self.phase_start_time
        test_config = self.test_configs[self.current_test]
        
        if self.test_phase == "waiting":
            # Waiting phase - prepare for test
            if elapsed < 0.5:
                return
                
            self.get_logger().info(f"🔄 Test {self.current_test + 1}/{len(self.test_configs)}: {test_config['name']}")
            self.stop_robot()
            time.sleep(0.1)
            
            if self.record_initial_positions():
                self.test_phase = "moving"
                self.phase_start_time = current_time
            else:
                self.get_logger().warn("⚠️  Waiting for joint state data...")
                
        elif self.test_phase == "moving":
            # Moving phase - send commands
            if elapsed < self.test_duration:
                self.send_velocity_command(test_config['vx'], test_config['vy'], test_config['vz'])
            else:
                self.stop_robot()
                self.test_phase = "analyzing"
                self.phase_start_time = current_time
                
        elif self.test_phase == "analyzing":
            # Analysis phase
            if elapsed < 0.5:
                return
                
            self.analyze_calibration(test_config)
            self.get_logger().info("─" * 60)
            
            # Move to next test
            self.current_test += 1
            self.test_phase = "waiting"
            self.phase_start_time = current_time
            
    def destroy_node(self):
        """Clean up"""
        self.get_logger().info("🛑 Stopping calibration diagnostic")
        self.stop_robot()
        super().destroy_node()

def main():
    rclpy.init()
    diagnostic = CalibrationDiagnostic()
    
    try:
        rclpy.spin(diagnostic)
    except KeyboardInterrupt:
        diagnostic.get_logger().info("🛑 Calibration diagnostic stopped by user")
    finally:
        diagnostic.stop_robot()
        diagnostic.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()