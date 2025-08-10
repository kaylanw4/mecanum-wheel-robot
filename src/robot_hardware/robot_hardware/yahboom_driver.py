#!/usr/bin/env python3
"""
Simplified Yahboom Hardware Driver with Direct Velocity Control
Removes PID complexity and uses calibrated direct hardware control
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
        # Mecanum wheel inverse kinematics - Fixed strafe direction, corrected rotation for physical layout
        # FL, FR, RL, RR = Logical positions (not matching physical due to rotated layout)
        # Physical layout: Logical FL=Physical RL, FR=Physical RR, RL=Physical FL, RR=Physical FR
        # For proper rotation: Physical Left(FL+RL) forward, Physical Right(FR+RR) backward
        # So: Logical RL+RR get +vz, Logical FL+FR get -vz
        rotation_factor = (self.lx + self.ly) * 2.0  # Increase rotation torque
        fl_vel = (vx - (-vy) - vz * rotation_factor) / self.wheel_radius  # Logical FL (Phys RL): -vz
        fr_vel = (vx + (-vy) - vz * rotation_factor) / self.wheel_radius  # Logical FR (Phys RR): -vz
        rl_vel = (vx + (-vy) + vz * rotation_factor) / self.wheel_radius  # Logical RL (Phys FL): +vz  
        rr_vel = (vx - (-vy) + vz * rotation_factor) / self.wheel_radius  # Logical RR (Phys FR): +vz
        
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
        
        # Mecanum wheel forward kinematics - Fixed strafe direction, corrected rotation for physical layout
        vx = (fl_linear + fr_linear + rl_linear + rr_linear) / 4.0
        vy = -(-fl_linear + fr_linear + rl_linear - rr_linear) / 4.0  # Invert to match inverse kinematics fix
        rotation_factor = (self.lx + self.ly) * 2.0  # Match inverse kinematics rotation scaling
        vz = (-fl_linear - fr_linear + rl_linear + rr_linear) / (4.0 * rotation_factor)  # RL+RR:+ FL+FR:-
        
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


class SimplifiedYahboomDriver(Node):
    """
    Simplified Yahboom Hardware Driver with Direct Velocity Control
    Uses calibrated velocity scaling for predictable SLAM performance
    """
    
    # Encoder mapping for odometry calculation
    # Hardware returns encoders as [m1, m2, m3, m4], we need [FL, FR, RL, RR]
    ENCODER_TO_WHEEL_MAP = [1, 0, 3, 2]  # [m2→FL, m1→FR, m4→RL, m3→RR]
    
    def __init__(self):
        super().__init__('simplified_yahboom_driver')
        
        self.get_logger().info("🚀 Starting Simplified Yahboom Driver with Direct Velocity Control...")
        
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
        
        # Odometry calibration parameters
        self.declare_parameter('odom_linear_scale_x', 1.0)
        self.declare_parameter('odom_linear_scale_y', 1.0)
        self.declare_parameter('odom_angular_scale', 1.0)
        
        # Base velocity scaling for speed control
        self.declare_parameter('base_velocity_scale', 0.3)  # Overall speed scaling (0.1-1.0)
        
        # Fine-tuning calibration parameters
        self.declare_parameter('velocity_scale_x', 0.82)   # Linear X velocity scaling
        self.declare_parameter('velocity_scale_y', 0.96)   # Linear Y velocity scaling  
        self.declare_parameter('velocity_scale_z', 0.59)   # Angular Z velocity scaling
        
        # Individual motor calibration factors [FL, FR, RL, RR]
        self.declare_parameter('motor_cal_fl', 1.0)   # Front Left motor calibration
        self.declare_parameter('motor_cal_fr', 0.95)   # Front Right motor calibration  
        self.declare_parameter('motor_cal_rl', 1.05)   # Rear Left motor calibration
        self.declare_parameter('motor_cal_rr', 1.0)   # Rear Right motor calibration
        
        # Control parameters
        self.declare_parameter('max_linear_velocity', 1.5)  # m/s
        self.declare_parameter('max_angular_velocity', 3.0)  # rad/s
        self.declare_parameter('use_immediate_braking', True)  # Use reset_car_state for immediate stops  
        
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
        
        # Odometry calibration parameters
        self.odom_linear_scale_x = self.get_parameter('odom_linear_scale_x').value
        self.odom_linear_scale_y = self.get_parameter('odom_linear_scale_y').value
        self.odom_angular_scale = self.get_parameter('odom_angular_scale').value
        
        # Base velocity scaling
        self.base_velocity_scale = self.get_parameter('base_velocity_scale').value
        
        # Fine-tuning calibration parameters
        self.velocity_scale_x = self.get_parameter('velocity_scale_x').value
        self.velocity_scale_y = self.get_parameter('velocity_scale_y').value
        self.velocity_scale_z = self.get_parameter('velocity_scale_z').value
        
        # Individual motor calibration factors [FL, FR, RL, RR]
        motor_cal_fl = self.get_parameter('motor_cal_fl').value
        motor_cal_fr = self.get_parameter('motor_cal_fr').value
        motor_cal_rl = self.get_parameter('motor_cal_rl').value
        motor_cal_rr = self.get_parameter('motor_cal_rr').value
        self.motor_calibration_factors = [motor_cal_fl, motor_cal_fr, motor_cal_rl, motor_cal_rr]
        
        # Control parameters
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        self.use_immediate_braking = self.get_parameter('use_immediate_braking').value
        
        self.get_logger().info(f"🚀 Base velocity scale: {self.base_velocity_scale:.3f}")
        self.get_logger().info(f"📏 Fine-tune scaling: X={self.velocity_scale_x:.3f}, Y={self.velocity_scale_y:.3f}, Z={self.velocity_scale_z:.3f}")
        self.get_logger().info(f"🔧 Motor calibration: FL={motor_cal_fl:.3f}, FR={motor_cal_fr:.3f}, RL={motor_cal_rl:.3f}, RR={motor_cal_rr:.3f}")
        self.get_logger().info(f"🚀 Max velocities: Linear={self.max_linear_velocity:.1f} m/s, Angular={self.max_angular_velocity:.1f} rad/s")
        
        # Initialize kinematics
        self.lx = wheelbase_length / 2.0  
        self.ly = wheelbase_width / 2.0   
        self.kinematics = MecanumKinematics(
            self.wheel_radius, self.lx, self.ly, encoder_resolution
        )
        
        # Store last command for direct control
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
        
        # Command velocity diagnostics publisher
        self.cmd_vel_debug_pub = self.create_publisher(
            Float32MultiArray, 'cmd_vel_debug', 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create timers
        self.create_timer(0.02, self.control_loop)  # Simple control loop at 50Hz
        self.create_timer(0.05, self.publish_sensor_data)    # Sensor publishing
        
        # Initialize hardware
        self.initialize_hardware()
        
    def initialize_hardware(self):
        """Initialize hardware with proper PID setup"""
        if self.hardware is None:
            return
            
        self.get_logger().info("🔌 Initializing hardware with direct velocity control...")
        
        try:
            # Start communication thread
            self.hardware.create_receive_threading()
            time.sleep(0.1)
            
            # Let hardware use its default control parameters
            # No need to configure PID since we're using direct control
            
            # Test beep
            self.hardware.set_beep(50)
            time.sleep(0.1)
            
            # Get firmware version
            version = self.hardware.get_version()
            self.get_logger().info(f"📋 Firmware version: {version}")
            
            self.get_logger().info("✅ Simplified hardware initialized successfully")
            
        except Exception as e:
            self.get_logger().error(f"❌ Hardware initialization failed: {e}")
    
    def cmd_vel_callback(self, msg):
        """Handle velocity commands with direct calibrated control"""
        if self.hardware is None:
            return
            
        # Extract commanded velocities
        vx = msg.linear.x
        vy = msg.linear.y
        vz = msg.angular.z
        
        # Calibration mode: only log essential cmd_vel
        if abs(vx) > 0.01 or abs(vy) > 0.01 or abs(vz) > 0.01:
            self.get_logger().info(f"CMD_VEL: vx={vx:.3f}, vy={vy:.3f}, vz={vz:.3f}")
        
        # Apply velocity limits
        vx = max(-self.max_linear_velocity, min(self.max_linear_velocity, vx))
        vy = max(-self.max_linear_velocity, min(self.max_linear_velocity, vy))
        vz = max(-self.max_angular_velocity, min(self.max_angular_velocity, vz))
        
        # Apply base velocity scaling first (for speed control)
        vx_scaled = vx * self.base_velocity_scale
        vy_scaled = vy * self.base_velocity_scale  
        vz_scaled = vz * self.base_velocity_scale
        
        # Then apply fine-tuning calibration
        vx_calibrated = vx_scaled * self.velocity_scale_x
        vy_calibrated = vy_scaled * self.velocity_scale_y
        vz_calibrated = vz_scaled * self.velocity_scale_z
        
        # Check for stop command and handle it specially
        is_stop_command = (abs(vx_calibrated) < 0.001 and abs(vy_calibrated) < 0.001 and abs(vz_calibrated) < 0.001)
        
        # Store calibrated command
        self.last_cmd_vel = [vx_calibrated, vy_calibrated, vz_calibrated]
        
        # Handle stop command with immediate motor braking
        if is_stop_command:
            try:
                # First set motors to zero
                self.hardware.set_motor(0, 0, 0, 0)
                # Then use reset_car_state for immediate braking/parking if enabled
                if self.use_immediate_braking:
                    self.hardware.reset_car_state()
                if abs(vx) > 0.01 or abs(vy) > 0.01 or abs(vz) > 0.01:  # Only log if transition from motion to stop
                    brake_msg = "Motors braked" if self.use_immediate_braking else "Motors set to zero"
                    self.get_logger().info(f"🛑 IMMEDIATE STOP: {brake_msg}")
            except Exception as e:
                self.get_logger().warn(f"⚠️ Failed to execute immediate stop: {e}")
            return
        
        # Use direct motor control for precise velocity control
        # set_car_motion has firmware deadband issues, but set_motor gives true proportional control
        try:
            # Convert cmd_vel to individual wheel speeds using mecanum kinematics
            wheel_velocities = self.kinematics.cmd_vel_to_wheel_velocities(vx_calibrated, vy_calibrated, vz_calibrated)
            
            # Convert wheel velocities (rad/s) to PWM values (-100 to 100)
            # Use a lower max_wheel_speed for more precise control
            max_wheel_speed = 5.0  # rad/s - reduced for better precision
            motor_speeds = []
            for i, wheel_vel in enumerate(wheel_velocities):
                # Scale to -100 to 100 range for set_motor
                motor_speed = (wheel_vel / max_wheel_speed) * 100.0
                # Apply individual motor calibration if available
                if hasattr(self, 'motor_calibration_factors'):
                    motor_speed *= self.motor_calibration_factors[i]
                motor_speed = max(-100, min(100, int(motor_speed)))
                motor_speeds.append(motor_speed)
            
            # Map logical wheels to physical motors: [FL, FR, RL, RR] -> [m1, m2, m3, m4]
            # IMPORTANT: This mapping might need adjustment based on actual wiring
            m1_speed = motor_speeds[1]  # m1 = FR (Front Right)
            m2_speed = motor_speeds[0]  # m2 = FL (Front Left)  
            m3_speed = motor_speeds[3]  # m3 = RR (Rear Right)
            m4_speed = motor_speeds[2]  # m4 = RL (Rear Left)
            
            # Use direct motor control for precise proportional response
            self.hardware.set_motor(m1_speed, m2_speed, m3_speed, m4_speed)
            
        except Exception as e:
            self.get_logger().warn(f"⚠️ Failed to send motor command: {e}")
            
    def control_loop(self):
        """Simplified control loop for direct hardware control"""
        if self.hardware is None:
            return
            
        # The control loop is now much simpler - just ensure the last command
        # is still being sent in case of communication issues
        try:
            vx, vy, vz = self.last_cmd_vel
            
            # Only send command if there's actual movement requested
            if abs(vx) > 0.001 or abs(vy) > 0.001 or abs(vz) > 0.001:
                # Use same direct motor control as cmd_vel_callback
                wheel_velocities = self.kinematics.cmd_vel_to_wheel_velocities(vx, vy, vz)
                
                max_wheel_speed = 5.0  # Match cmd_vel_callback
                motor_speeds = []
                for i, wheel_vel in enumerate(wheel_velocities):
                    motor_speed = (wheel_vel / max_wheel_speed) * 100.0
                    if hasattr(self, 'motor_calibration_factors'):
                        motor_speed *= self.motor_calibration_factors[i]
                    motor_speed = max(-100, min(100, int(motor_speed)))
                    motor_speeds.append(motor_speed)
                
                # Map to physical motors
                m1_speed = motor_speeds[1]  # FR
                m2_speed = motor_speeds[0]  # FL
                m3_speed = motor_speeds[3]  # RR
                m4_speed = motor_speeds[2]  # RL
                
                self.hardware.set_motor(m1_speed, m2_speed, m3_speed, m4_speed)
            else:
                # Ensure robot stops immediately when no movement is requested
                try:
                    # First set motors to zero
                    self.hardware.set_motor(0, 0, 0, 0)
                    # Then use reset_car_state for immediate braking/parking if enabled
                    # Only call reset_car_state occasionally to avoid overwhelming the hardware
                    if self.use_immediate_braking:
                        if not hasattr(self, '_last_reset_time'):
                            self._last_reset_time = 0
                        if time.time() - self._last_reset_time > 0.1:  # Reset every 100ms when stopped
                            self.hardware.reset_car_state()
                            self._last_reset_time = time.time()
                except Exception as e:
                    self.get_logger().warn(f"⚠️ Failed to execute control loop stop: {e}")
                
        except Exception as e:
            self.get_logger().warn(f"⚠️ Control loop error: {e}")
            
    def publish_cmd_vel_debug(self):
        """Publish command velocity debug information"""
        if self.hardware is None:
            return
            
        try:
            # Publish current command velocities for debugging
            debug_msg = Float32MultiArray()
            debug_msg.data = [float(v) for v in self.last_cmd_vel]
            self.cmd_vel_debug_pub.publish(debug_msg)
            
            # Log diagnostics occasionally
            if hasattr(self, '_last_debug_log'):
                if time.time() - self._last_debug_log > 3.0:  # Every 3 seconds
                    self._log_cmd_vel_debug()
                    self._last_debug_log = time.time()
            else:
                self._last_debug_log = time.time()
                
        except Exception as e:
            self.get_logger().warn(f"⚠️ Command velocity debug failed: {e}")
            
    def _log_cmd_vel_debug(self):
        """Log simplified command velocity debug information"""
        vx, vy, vz = self.last_cmd_vel
        if abs(vx) > 0.01 or abs(vy) > 0.01 or abs(vz) > 0.01:  # Only log when moving
            self.get_logger().info(f"ACTIVE_CMD_VEL: vx={vx:.3f}, vy={vy:.3f}, vz={vz:.3f}")
    
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
            
            # Publish debug information
            self.publish_cmd_vel_debug()
            
        except Exception as e:
            self.get_logger().warn(f"⚠️ Sensor publishing failed: {e}")
            
    def calculate_and_publish_odometry(self, timestamp):
        """Calculate and publish accurate odometry"""
        try:
            # Get encoder readings
            m1, m2, m3, m4 = self.hardware.get_motor_encoder()
            raw_encoders = [m1, m2, m3, m4]
            encoder_counts = [raw_encoders[i] for i in self.ENCODER_TO_WHEEL_MAP]  # Map to FL, FR, RL, RR
            
            # Calculate individual wheel speeds for calibration debugging BEFORE updating odometry
            current_time = time.time()
            dt = current_time - self.kinematics.prev_time
            if dt > 0 and self.kinematics.initialized:
                # Calculate change in encoder counts
                delta_encoders = [
                    encoder_counts[i] - self.kinematics.prev_encoders[i] 
                    for i in range(4)
                ]
                # Convert to wheel velocities (rad/s)
                wheel_speeds = [delta * self.kinematics.counts_to_meters / (dt * self.wheel_radius) for delta in delta_encoders]
                
                # Log wheel speeds for calibration (every 5th cycle to avoid spam)
                if not hasattr(self, '_speed_log_count'):
                    self._speed_log_count = 0
                self._speed_log_count += 1
                if self._speed_log_count % 5 == 0:
                    self.get_logger().info(f"WHEEL_SPEEDS: FL={wheel_speeds[0]:.2f}, FR={wheel_speeds[1]:.2f}, RL={wheel_speeds[2]:.2f}, RR={wheel_speeds[3]:.2f} rad/s")
            
            # Update odometry
            x, y, theta, vx, vy, vtheta = self.kinematics.update_odometry(encoder_counts)
            
            # Apply odometry calibration and coordinate frame correction
            # Invert all directions to match robot coordinate frame to RViz visualization
            x_calibrated = -x * self.odom_linear_scale_x
            y_calibrated = -y * self.odom_linear_scale_y
            theta_calibrated = -theta * self.odom_angular_scale
            vx_calibrated = -vx * self.odom_linear_scale_x
            vy_calibrated = -vy * self.odom_linear_scale_y
            vtheta_calibrated = -vtheta * self.odom_angular_scale
            
            # Create odometry message
            odom_msg = Odometry()
            odom_msg.header.stamp = timestamp.to_msg()
            odom_msg.header.frame_id = self.odom_frame_id
            odom_msg.child_frame_id = self.base_frame_id
            
            # Set position (using calibrated values)
            odom_msg.pose.pose.position.x = x_calibrated
            odom_msg.pose.pose.position.y = y_calibrated
            odom_msg.pose.pose.position.z = 0.0
            
            # Convert angle to quaternion using scipy (NumPy 2.0 compatible)
            quat = euler_to_quaternion(0, 0, theta_calibrated)
            odom_msg.pose.pose.orientation.x = quat[0]
            odom_msg.pose.pose.orientation.y = quat[1]
            odom_msg.pose.pose.orientation.z = quat[2]
            odom_msg.pose.pose.orientation.w = quat[3]
            
            # Set velocity (using calibrated values)
            odom_msg.twist.twist.linear.x = vx_calibrated
            odom_msg.twist.twist.linear.y = vy_calibrated
            odom_msg.twist.twist.angular.z = vtheta_calibrated
            
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
                t.transform.translation.x = x_calibrated
                t.transform.translation.y = y_calibrated
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
            raw_encoders = [m1, m2, m3, m4]
            wheel_encoders = [raw_encoders[i] for i in self.ENCODER_TO_WHEEL_MAP]  # Map to FL, FR, RL, RR
            encoder_to_rad = 2 * pi / self.kinematics.encoder_resolution
            
            joint_msg.position = [
                wheel_encoders[0] * encoder_to_rad,  # front_left
                wheel_encoders[1] * encoder_to_rad,  # front_right
                wheel_encoders[2] * encoder_to_rad,  # rear_left
                wheel_encoders[3] * encoder_to_rad   # rear_right
            ]
            joint_msg.velocity = [0.0, 0.0, 0.0, 0.0]  # No velocity feedback in simplified control
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
        """Clean shutdown with immediate motor braking"""
        self.get_logger().info("🛑 Shutting down simplified velocity driver")
        if self.hardware is not None:
            try:
                # Stop all motors immediately
                self.last_cmd_vel = [0.0, 0.0, 0.0]
                self.hardware.set_motor(0, 0, 0, 0)  # Direct motor stop
                self.hardware.reset_car_state()      # Engage braking/parking
                self.get_logger().info("🛑 Motors stopped and braked")
            except Exception as e:
                self.get_logger().warn(f"⚠️ Error during shutdown motor stop: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        driver = SimplifiedYahboomDriver()
        rclpy.spin(driver)
    except KeyboardInterrupt:
        pass
    finally:
        if 'driver' in locals():
            driver.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()