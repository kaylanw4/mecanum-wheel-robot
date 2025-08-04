#!/usr/bin/env python3
"""
Enhanced Yahboom Hardware Driver with PROPER Motor Calibration
Fixes calibration to affect actual speed, not just acceleration
"""

import sys
import math
import time
import threading
from math import pi, sin, cos, atan2, sqrt
from time import sleep

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from sensor_msgs.msg import Imu, MagneticField, JointState
from std_msgs.msg import Float32, Int32, Bool, Float32MultiArray
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R

# Import the official Yahboom library
from Rosmaster_Lib import Rosmaster


class PIDController:
    """
    Industry-standard PID controller for velocity control
    """
    def __init__(self, kp=1.0, ki=0.1, kd=0.05, output_limits=(-100, 100)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        
    def update(self, setpoint, measured_value):
        """Update PID controller with new measurements"""
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0.0:
            return 0.0
            
        error = setpoint - measured_value
        
        # Proportional term
        proportional = self.kp * error
        
        # Integral term with windup protection
        self.integral += error * dt
        integral_term = self.ki * self.integral
        
        # Derivative term
        derivative = self.kd * (error - self.last_error) / dt
        
        # Calculate output
        output = proportional + integral_term + derivative
        
        # Apply output limits
        if self.output_limits:
            output = max(min(output, self.output_limits[1]), self.output_limits[0])
            
            # Anti-windup: reduce integral if output is saturated
            if output >= self.output_limits[1] or output <= self.output_limits[0]:
                self.integral -= error * dt
        
        self.last_error = error
        self.last_time = current_time
        
        return output
        
    def reset(self):
        """Reset PID controller state"""
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()


class MecanumKinematics:
    """
    Proper mecanum wheel kinematics with motor calibration
    """
    
    def __init__(self, wheel_radius, lx, ly, encoder_resolution):
        self.wheel_radius = wheel_radius
        self.lx = lx  # half wheelbase length
        self.ly = ly  # half wheelbase width  
        self.encoder_resolution = encoder_resolution
        
        # Current robot pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Previous encoder values and time
        self.prev_encoders = [0, 0, 0, 0]
        self.prev_time = time.time()
        self.initialized = False
        
        # Conversion factor from encoder counts to distance
        self.counts_to_meters = (2 * pi * wheel_radius) / encoder_resolution
        
    def cmd_vel_to_wheel_velocities(self, vx, vy, vz):
        """
        Convert body velocities to individual wheel velocities (rad/s)
        Uses proper mecanum wheel inverse kinematics
        """
        # Mecanum wheel inverse kinematics
        # FL, FR, RL, RR = Front Left, Front Right, Rear Left, Rear Right
        fl_vel = (vx - vy - vz * (self.lx + self.ly)) / self.wheel_radius
        fr_vel = (vx + vy + vz * (self.lx + self.ly)) / self.wheel_radius  
        rl_vel = (vx + vy - vz * (self.lx + self.ly)) / self.wheel_radius
        rr_vel = (vx - vy + vz * (self.lx + self.ly)) / self.wheel_radius
        
        return [fl_vel, fr_vel, rl_vel, rr_vel]
        
    def wheel_velocities_to_cmd_vel(self, wheel_vels):
        """
        Convert wheel velocities (rad/s) to body velocities
        Uses mecanum wheel forward kinematics
        """
        fl_vel, fr_vel, rl_vel, rr_vel = wheel_vels
        
        # Convert to linear velocities
        fl_linear = fl_vel * self.wheel_radius
        fr_linear = fr_vel * self.wheel_radius
        rl_linear = rl_vel * self.wheel_radius
        rr_linear = rr_vel * self.wheel_radius
        
        # Mecanum wheel forward kinematics
        vx = (fl_linear + fr_linear + rl_linear + rr_linear) / 4.0
        vy = (-fl_linear + fr_linear + rl_linear - rr_linear) / 4.0
        vz = (-fl_linear + fr_linear - rl_linear + rr_linear) / (4.0 * (self.lx + self.ly))
        
        return vx, vy, vz
        
    def update_odometry(self, encoder_counts):
        """
        Update odometry from encoder readings
        """
        current_time = time.time()
        dt = current_time - self.prev_time
        
        if not self.initialized or dt <= 0:
            self.prev_encoders = encoder_counts.copy()
            self.prev_time = current_time
            self.initialized = True
            return self.x, self.y, self.theta, 0.0, 0.0, 0.0
            
        # Calculate change in encoder counts
        delta_encoders = [
            encoder_counts[i] - self.prev_encoders[i] 
            for i in range(4)
        ]
        
        # Convert to wheel velocities (rad/s)
        wheel_vels = [delta * self.counts_to_meters / (dt * self.wheel_radius) for delta in delta_encoders]
        
        # Get body velocities
        vx, vy, vz = self.wheel_velocities_to_cmd_vel(wheel_vels)
        
        # Calculate displacement
        dx_robot = vx * dt
        dy_robot = vy * dt
        dtheta = vz * dt
        
        # Transform to global coordinates
        cos_theta = cos(self.theta)
        sin_theta = sin(self.theta)
        
        dx_global = dx_robot * cos_theta - dy_robot * sin_theta
        dy_global = dx_robot * sin_theta + dy_robot * cos_theta
        
        # Update pose
        self.x += dx_global
        self.y += dy_global
        self.theta += dtheta
        
        # Normalize angle
        while self.theta > pi:
            self.theta -= 2 * pi
        while self.theta < -pi:
            self.theta += 2 * pi
            
        # Store for next iteration
        self.prev_encoders = encoder_counts.copy()
        self.prev_time = current_time
        
        return self.x, self.y, self.theta, vx, vy, vz


def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles to quaternion using scipy (NumPy 2.0 compatible)
    Replaces tf_transformations.quaternion_from_euler
    """
    # Create rotation object from Euler angles (in radians)
    rotation = R.from_euler('xyz', [roll, pitch, yaw])
    
    # Get quaternion in [x, y, z, w] format
    quat = rotation.as_quat()
    
    return quat  # Returns [x, y, z, w]


class EnhancedYahboomDriver(Node):
    """
    Enhanced Yahboom Hardware Driver with PROPER Motor Calibration
    """
    
    def __init__(self):
        super().__init__('enhanced_yahboom_driver')
        
        self.get_logger().info("🚀 Starting Enhanced Yahboom Driver with PROPER Calibration...")
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('serial_baudrate', 115200)
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('publish_odom_tf', True)
        
        # Robot physical parameters
        self.declare_parameter('wheel_radius', 0.0395)  
        self.declare_parameter('wheelbase_length', 0.22)  
        self.declare_parameter('wheelbase_width', 0.22)   
        self.declare_parameter('encoder_resolution', 2464)
        
        # PID parameters for each wheel
        self.declare_parameter('pid_kp', 1.5)  
        self.declare_parameter('pid_ki', 0.3)  
        self.declare_parameter('pid_kd', 0.05) 
        
        # PROPER Motor calibration factors - now applied to motor commands, not targets
        self.declare_parameter('motor_cal_fl', 1.0)  # Front left
        self.declare_parameter('motor_cal_fr', 1.0)  # Front right
        self.declare_parameter('motor_cal_rl', 1.0)  # Rear left
        self.declare_parameter('motor_cal_rr', 1.0)  # Rear right
        
        # Control parameters
        self.declare_parameter('control_frequency', 50.0)  # Hz
        self.declare_parameter('max_velocity', 1.5)  # m/s
        self.declare_parameter('use_hybrid_control', True)  
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.serial_baudrate = self.get_parameter('serial_baudrate').value
        self.imu_frame_id = self.get_parameter('imu_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.publish_odom_tf = self.get_parameter('publish_odom_tf').value
        
        # Robot physical parameters
        self.wheel_radius = self.get_parameter('wheel_radius').value
        wheelbase_length = self.get_parameter('wheelbase_length').value
        wheelbase_width = self.get_parameter('wheelbase_width').value
        encoder_resolution = self.get_parameter('encoder_resolution').value
        
        # PID parameters
        pid_kp = self.get_parameter('pid_kp').value
        pid_ki = self.get_parameter('pid_ki').value
        pid_kd = self.get_parameter('pid_kd').value
        
        # Motor calibration - NOW PROPERLY APPLIED
        self.motor_cal = [
            self.get_parameter('motor_cal_fl').value,
            self.get_parameter('motor_cal_fr').value,
            self.get_parameter('motor_cal_rl').value,
            self.get_parameter('motor_cal_rr').value
        ]
        
        # Control parameters
        self.control_freq = self.get_parameter('control_frequency').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.use_hybrid_control = self.get_parameter('use_hybrid_control').value
        
        self.get_logger().info(f"🔧 PID gains: Kp={pid_kp}, Ki={pid_ki}, Kd={pid_kd}")
        self.get_logger().info(f"⚙️ Motor calibration: {self.motor_cal}")
        self.get_logger().info(f"🔄 Hybrid control mode: {self.use_hybrid_control}")
        
        # Initialize kinematics
        self.lx = wheelbase_length / 2.0  
        self.ly = wheelbase_width / 2.0   
        self.kinematics = MecanumKinematics(
            self.wheel_radius, self.lx, self.ly, encoder_resolution
        )
        
        # Initialize PID controllers for each wheel
        self.pid_controllers = [
            PIDController(pid_kp, pid_ki, pid_kd, (-100, 100)) for _ in range(4)
        ]
        
        # Control state - NO MORE CALIBRATION APPLIED TO TARGETS
        self.target_wheel_vels = [0.0, 0.0, 0.0, 0.0]  # rad/s (raw targets)
        self.measured_wheel_vels = [0.0, 0.0, 0.0, 0.0]  # rad/s
        self.prev_encoder_counts = [0, 0, 0, 0]
        self.prev_encoder_time = time.time()
        
        # Store last command for hybrid control
        self.last_cmd_vel = [0.0, 0.0, 0.0]  # vx, vy, vz
        
        # Initialize hardware interface
        try:
            self.hardware = Rosmaster(
                car_type=1,  # X3 robot type
                com=self.serial_port,
                debug=False
            )
            self.get_logger().info(f"✅ Hardware initialized on {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"❌ Hardware initialization failed: {e}")
            self.hardware = None
        
        # Create subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.rgb_light_sub = self.create_subscription(
            Int32, 'rgb_light', self.rgb_light_callback, 10)
        self.buzzer_sub = self.create_subscription(
            Bool, 'buzzer', self.buzzer_callback, 10)
            
        # Create publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.joint_states_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.battery_pub = self.create_publisher(Float32, 'battery_voltage', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # Wheel velocity diagnostics publishers
        self.wheel_velocities_pub = self.create_publisher(
            Float32MultiArray, 'wheel_velocities/measured', 10)
        self.wheel_targets_pub = self.create_publisher(
            Float32MultiArray, 'wheel_velocities/targets', 10)
        self.wheel_errors_pub = self.create_publisher(
            Float32MultiArray, 'wheel_velocities/errors', 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create timers
        control_period = 1.0 / self.control_freq
        self.create_timer(control_period, self.control_loop)  # Main control loop
        self.create_timer(0.05, self.publish_sensor_data)    # Sensor publishing
        self.create_timer(0.1, self.publish_wheel_diagnostics)  # Wheel diagnostics
        
        # Initialize hardware
        self.initialize_hardware()
        
    def initialize_hardware(self):
        """Initialize hardware with proper PID setup"""
        if self.hardware is None:
            return
            
        self.get_logger().info("🔌 Initializing hardware with proper calibration...")
        
        try:
            # Start communication thread
            self.hardware.create_receive_threading()
            time.sleep(0.1)
            
            # Configure PID for motion control
            self.hardware.set_pid_param(1.5, 0.3, 0.05, forever=False)
            time.sleep(0.1)
            
            # Test beep
            self.hardware.set_beep(50)
            time.sleep(0.1)
            
            # Get firmware version
            version = self.hardware.get_version()
            self.get_logger().info(f"📋 Firmware version: {version}")
            
            self.get_logger().info("✅ Enhanced hardware initialized successfully")
            
        except Exception as e:
            self.get_logger().error(f"❌ Hardware initialization failed: {e}")
            
    def cmd_vel_callback(self, msg):
        """Handle velocity commands - NO CALIBRATION APPLIED TO TARGETS"""
        if self.hardware is None:
            return
            
        # Extract commanded velocities
        vx = msg.linear.x
        vy = msg.linear.y
        vz = msg.angular.z
        
        # Apply velocity limits
        vx = max(-self.max_velocity, min(self.max_velocity, vx))
        vy = max(-self.max_velocity, min(self.max_velocity, vy))
        vz = max(-3.0, min(3.0, vz))  # Angular velocity limit
        
        # Store command for control loop
        self.last_cmd_vel = [vx, vy, vz]
        
        # Convert to wheel velocities - NO CALIBRATION APPLIED HERE
        self.target_wheel_vels = self.kinematics.cmd_vel_to_wheel_velocities(vx, vy, vz)
        
        # DO NOT apply calibration to targets - that was the bug!
        # Calibration will be applied to motor commands instead
            
    def control_loop(self):
        """Main control loop with PROPER calibration"""
        if self.hardware is None:
            return
            
        try:
            # Get current encoder readings
            m1, m2, m3, m4 = self.hardware.get_motor_encoder()
            current_encoders = [m2, m1, m4, m3]  # Map to FL, FR, RL, RR
            
            # Calculate measured wheel velocities from encoders with PROPER SIGNS
            current_time = time.time()
            dt = current_time - self.prev_encoder_time
            
            if dt > 0 and self.prev_encoder_time > 0:
                # Calculate velocities (rad/s) with consistent signs
                for i in range(4):
                    delta_counts = current_encoders[i] - self.prev_encoder_counts[i]
                    delta_radians = (delta_counts * 2 * pi) / self.kinematics.encoder_resolution
                    # FIXED: Ensure measured velocities have same sign convention as targets
                    self.measured_wheel_vels[i] = delta_radians / dt
            
            # HYBRID CONTROL with PROPER calibration
            vx, vy, vz = self.last_cmd_vel
            
            if abs(vz) > 0.1:  # Rotation command - use reliable set_car_motion
                self.hardware.set_car_motion(vx, vy, vz)
                # Reset PID controllers to prevent conflicts
                for pid in self.pid_controllers:
                    pid.reset()
            elif abs(vx) > 0.05 or abs(vy) > 0.05:  # Linear motion - use PID for precision
                if self.use_hybrid_control:
                    # PID control with PROPER calibration applied to OUTPUT
                    motor_commands = []
                    for i in range(4):
                        # PID control to match target velocity (no calibration on target)
                        pid_output = self.pid_controllers[i].update(
                            self.target_wheel_vels[i], 
                            self.measured_wheel_vels[i]
                        )
                        
                        # APPLY CALIBRATION TO MOTOR COMMAND OUTPUT - THIS IS THE FIX!
                        calibrated_command = pid_output * self.motor_cal[i]
                        motor_commands.append(int(calibrated_command))
                        
                    # Send calibrated motor commands to hardware
                    self.hardware.set_motor(
                        motor_commands[1],  # Front right
                        motor_commands[0],  # Front left  
                        motor_commands[3],  # Rear right
                        motor_commands[2]   # Rear left
                    )
                else:
                    # Fallback to set_car_motion
                    self.hardware.set_car_motion(vx, vy, vz)
            else:
                # Stop the robot
                self.hardware.set_car_motion(0, 0, 0)
                # Reset PID controllers
                for pid in self.pid_controllers:
                    pid.reset()
            
            # Store for next iteration
            self.prev_encoder_counts = current_encoders.copy()
            self.prev_encoder_time = current_time
            
        except Exception as e:
            self.get_logger().warn(f"⚠️ Control loop error: {e}")
            
    def publish_wheel_diagnostics(self):
        """Publish wheel velocity diagnostics for monitoring"""
        if self.hardware is None:
            return
            
        try:
            # Measured wheel velocities
            measured_msg = Float32MultiArray()
            measured_msg.data = [float(v) for v in self.measured_wheel_vels]
            self.wheel_velocities_pub.publish(measured_msg)
            
            # Target wheel velocities (raw, no calibration)
            targets_msg = Float32MultiArray()
            targets_msg.data = [float(v) for v in self.target_wheel_vels]
            self.wheel_targets_pub.publish(targets_msg)
            
            # Velocity errors (target - measured)
            errors_msg = Float32MultiArray()
            errors = [self.target_wheel_vels[i] - self.measured_wheel_vels[i] for i in range(4)]
            errors_msg.data = [float(e) for e in errors]
            self.wheel_errors_pub.publish(errors_msg)
            
            # Log diagnostics occasionally
            if hasattr(self, '_last_diagnostic_log'):
                if time.time() - self._last_diagnostic_log > 2.0:  # Every 2 seconds
                    self._log_wheel_diagnostics()
                    self._last_diagnostic_log = time.time()
            else:
                self._last_diagnostic_log = time.time()
                
        except Exception as e:
            self.get_logger().warn(f"⚠️ Wheel diagnostics failed: {e}")
            
    def _log_wheel_diagnostics(self):
        """Log wheel diagnostics for debugging"""
        if any(abs(v) > 0.1 for v in self.target_wheel_vels):  # Only log when moving
            self.get_logger().info("🔍 WHEEL DIAGNOSTICS (PROPER CALIBRATION):")
            self.get_logger().info(f"  Targets: FL={self.target_wheel_vels[0]:.2f}, FR={self.target_wheel_vels[1]:.2f}, "
                                 f"RL={self.target_wheel_vels[2]:.2f}, RR={self.target_wheel_vels[3]:.2f} rad/s")
            self.get_logger().info(f"  Measured: FL={self.measured_wheel_vels[0]:.2f}, FR={self.measured_wheel_vels[1]:.2f}, "
                                 f"RL={self.measured_wheel_vels[2]:.2f}, RR={self.measured_wheel_vels[3]:.2f} rad/s")
            
            # Calculate speed ranges and show calibration factors
            if self.measured_wheel_vels and any(abs(v) > 0.1 for v in self.measured_wheel_vels):
                speeds = [abs(v) for v in self.measured_wheel_vels if abs(v) > 0.1]
                if speeds:
                    min_speed = min(speeds)
                    max_speed = max(speeds)
                    speed_variation = ((max_speed - min_speed) / max_speed) * 100 if max_speed > 0 else 0
                    self.get_logger().info(f"  Speed variation: {speed_variation:.1f}%")
                    self.get_logger().info(f"  Calibration: FL={self.motor_cal[0]:.3f}, FR={self.motor_cal[1]:.3f}, "
                                         f"RL={self.motor_cal[2]:.3f}, RR={self.motor_cal[3]:.3f}")
    
    def publish_sensor_data(self):
        """Publish sensor data and odometry"""
        if self.hardware is None:
            return
            
        current_time = self.get_clock().now()
        
        try:
            # Calculate and publish odometry
            self.calculate_and_publish_odometry(current_time)
            
            # Publish other sensor data
            self.publish_imu_data(current_time)
            self.publish_mag_data(current_time)
            self.publish_joint_states(current_time)
            self.publish_battery_data()
            
        except Exception as e:
            self.get_logger().warn(f"⚠️ Sensor publishing failed: {e}")
            
    def calculate_and_publish_odometry(self, timestamp):
        """Calculate and publish accurate odometry"""
        try:
            # Get encoder readings
            m1, m2, m3, m4 = self.hardware.get_motor_encoder()
            encoder_counts = [m2, m1, m4, m3]  # Map to FL, FR, RL, RR
            
            # Update odometry
            x, y, theta, vx, vy, vtheta = self.kinematics.update_odometry(encoder_counts)
            
            # Create odometry message
            odom_msg = Odometry()
            odom_msg.header.stamp = timestamp.to_msg()
            odom_msg.header.frame_id = self.odom_frame_id
            odom_msg.child_frame_id = self.base_frame_id
            
            # Set position
            odom_msg.pose.pose.position.x = x
            odom_msg.pose.pose.position.y = y
            odom_msg.pose.pose.position.z = 0.0
            
            # Convert angle to quaternion using scipy (NumPy 2.0 compatible)
            quat = euler_to_quaternion(0, 0, theta)
            odom_msg.pose.pose.orientation.x = quat[0]
            odom_msg.pose.pose.orientation.y = quat[1]
            odom_msg.pose.pose.orientation.z = quat[2]
            odom_msg.pose.pose.orientation.w = quat[3]
            
            # Set velocity
            odom_msg.twist.twist.linear.x = vx
            odom_msg.twist.twist.linear.y = vy
            odom_msg.twist.twist.angular.z = vtheta
            
            # Set covariance
            odom_msg.pose.covariance[0] = 0.001   # x
            odom_msg.pose.covariance[7] = 0.001   # y
            odom_msg.pose.covariance[35] = 0.01   # yaw
            odom_msg.twist.covariance[0] = 0.001  # vx
            odom_msg.twist.covariance[7] = 0.001  # vy
            odom_msg.twist.covariance[35] = 0.01  # vyaw
            
            # Publish odometry
            self.odom_pub.publish(odom_msg)
            
            # Publish TF
            if self.publish_odom_tf:
                t = TransformStamped()
                t.header.stamp = timestamp.to_msg()
                t.header.frame_id = self.odom_frame_id
                t.child_frame_id = self.base_frame_id
                t.transform.translation.x = x
                t.transform.translation.y = y
                t.transform.translation.z = 0.0
                t.transform.rotation.x = quat[0]
                t.transform.rotation.y = quat[1]
                t.transform.rotation.z = quat[2]
                t.transform.rotation.w = quat[3]
                self.tf_broadcaster.sendTransform(t)
                
        except Exception as e:
            self.get_logger().warn(f"⚠️ Odometry calculation failed: {e}")
        
    def publish_imu_data(self, timestamp):
        """Publish IMU sensor data"""
        imu_msg = Imu()
        imu_msg.header.stamp = timestamp.to_msg()
        imu_msg.header.frame_id = self.imu_frame_id
        
        # Get sensor data
        ax, ay, az = self.hardware.get_accelerometer_data()
        gx, gy, gz = self.hardware.get_gyroscope_data()
        
        # Fill IMU message
        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az
        imu_msg.angular_velocity.x = gx
        imu_msg.angular_velocity.y = gy
        imu_msg.angular_velocity.z = gz
        
        # Set covariance
        imu_msg.linear_acceleration_covariance[0] = -1
        imu_msg.angular_velocity_covariance[0] = -1
        imu_msg.orientation_covariance[0] = -1
        
        self.imu_pub.publish(imu_msg)
        
    def publish_mag_data(self, timestamp):
        """Publish magnetometer data"""
        mag_msg = MagneticField()
        mag_msg.header.stamp = timestamp.to_msg()
        mag_msg.header.frame_id = self.imu_frame_id
        
        mx, my, mz = self.hardware.get_magnetometer_data()
        mag_msg.magnetic_field.x = mx
        mag_msg.magnetic_field.y = my
        mag_msg.magnetic_field.z = mz
        mag_msg.magnetic_field_covariance[0] = -1
        
        self.mag_pub.publish(mag_msg)
        
    def publish_joint_states(self, timestamp):
        """Publish joint states for visualization"""
        joint_msg = JointState()
        joint_msg.header.stamp = timestamp.to_msg()
        joint_msg.header.frame_id = 'joint_states'
        
        joint_msg.name = [
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_left_wheel_joint', 
            'rear_right_wheel_joint'
        ]
        
        try:
            # Get encoder data and convert to wheel angles
            m1, m2, m3, m4 = self.hardware.get_motor_encoder()
            encoder_to_rad = 2 * pi / self.kinematics.encoder_resolution
            
            joint_msg.position = [
                m2 * encoder_to_rad,  # front_left
                m1 * encoder_to_rad,  # front_right
                m4 * encoder_to_rad,  # rear_left
                m3 * encoder_to_rad   # rear_right
            ]
            joint_msg.velocity = self.measured_wheel_vels.copy()
            joint_msg.effort = [0.0, 0.0, 0.0, 0.0]
            
        except Exception:
            joint_msg.position = [0.0, 0.0, 0.0, 0.0]
            joint_msg.velocity = [0.0, 0.0, 0.0, 0.0]
            joint_msg.effort = [0.0, 0.0, 0.0, 0.0]
        
        self.joint_states_pub.publish(joint_msg)
        
    def publish_battery_data(self):
        """Publish battery voltage"""
        battery_msg = Float32()
        battery_msg.data = self.hardware.get_battery_voltage()
        self.battery_pub.publish(battery_msg)
        
    def rgb_light_callback(self, msg):
        """Handle RGB light control"""
        if self.hardware is None:
            return
        try:
            self.hardware.set_colorful_effect(msg.data, 6, parm=1)
        except Exception as e:
            self.get_logger().warn(f"⚠️ RGB control failed: {e}")
        
    def buzzer_callback(self, msg):
        """Handle buzzer control"""
        if self.hardware is None:
            return
        try:
            buzzer_time = 100 if msg.data else 0
            self.hardware.set_beep(buzzer_time)
        except Exception as e:
            self.get_logger().warn(f"⚠️ Buzzer control failed: {e}")
        
    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info("🛑 Shutting down enhanced driver")
        if self.hardware is not None:
            try:
                # Stop all motors
                self.last_cmd_vel = [0.0, 0.0, 0.0]
                self.hardware.set_car_motion(0, 0, 0)
                self.hardware.reset_car_state()
            except:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        driver = EnhancedYahboomDriver()
        rclpy.spin(driver)
    except KeyboardInterrupt:
        pass
    finally:
        if 'driver' in locals():
            driver.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()