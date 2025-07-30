#!/usr/bin/env python3
"""
CORRECTED Yahboom Hardware Driver with Fixed Directions and Motor Calibration
Fixes forward/backward direction and adds calibration support
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
from std_msgs.msg import Float32, Int32, Bool
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import tf_transformations

# Import the official Yahboom library
from Rosmaster_Lib import Rosmaster


class MecanumOdometry:
    """
    Calculate odometry for mecanum wheel robot with correct kinematics
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
        
        # Previous encoder values
        self.prev_encoders = [0, 0, 0, 0]
        self.initialized = False
        
        # Conversion factor from encoder counts to distance
        self.counts_to_meters = (2 * pi * wheel_radius) / encoder_resolution
        
    def update(self, encoder_counts, dt):
        """
        Update odometry with corrected mecanum wheel kinematics
        encoder_counts: [front_left, front_right, rear_left, rear_right]
        """
        if not self.initialized:
            self.prev_encoders = encoder_counts.copy()
            self.initialized = True
            return self.x, self.y, self.theta, 0.0, 0.0, 0.0
            
        # Calculate change in encoder counts
        delta_encoders = [
            encoder_counts[i] - self.prev_encoders[i] 
            for i in range(4)
        ]
        self.prev_encoders = encoder_counts.copy()
        
        # Convert to wheel distances (meters)
        fl_dist = delta_encoders[0] * self.counts_to_meters  # front left
        fr_dist = delta_encoders[1] * self.counts_to_meters  # front right
        rl_dist = delta_encoders[2] * self.counts_to_meters  # rear left
        rr_dist = delta_encoders[3] * self.counts_to_meters  # rear right
        
        # Mecanum wheel kinematics (corrected for your robot)
        dx_robot = (fl_dist + fr_dist + rl_dist + rr_dist) / 4.0
        dy_robot = (-fl_dist + fr_dist - rl_dist + rr_dist) / 4.0
        dtheta = (-fl_dist + fr_dist - rl_dist + rr_dist) / (4.0 * (self.lx + self.ly))
        
        # Calculate velocities
        vx_robot = dx_robot / dt if dt > 0 else 0.0
        vy_robot = dy_robot / dt if dt > 0 else 0.0
        vtheta = dtheta / dt if dt > 0 else 0.0
        
        # Transform robot-relative motion to global coordinates
        cos_theta = cos(self.theta)
        sin_theta = sin(self.theta)
        
        dx_global = dx_robot * cos_theta - dy_robot * sin_theta
        dy_global = dx_robot * sin_theta + dy_robot * cos_theta
        
        # Update pose
        self.x += dx_global
        self.y += dy_global
        self.theta += dtheta
        
        # Normalize angle to [-pi, pi]
        while self.theta > pi:
            self.theta -= 2 * pi
        while self.theta < -pi:
            self.theta += 2 * pi
            
        # Transform velocities to global frame
        vx_global = vx_robot * cos_theta - vy_robot * sin_theta
        vy_global = vx_robot * sin_theta + vy_robot * cos_theta
        
        return self.x, self.y, self.theta, vx_global, vy_global, vtheta
        
    def reset_pose(self, x=0.0, y=0.0, theta=0.0):
        """Reset robot pose to specified values"""
        self.x = x
        self.y = y
        self.theta = theta


class YahboomHardwareDriver(Node):
    """
    CORRECTED Yahboom Hardware Driver with Fixed Directions and Calibration
    """
    
    def __init__(self):
        super().__init__('yahboom_hardware_driver')
        
        self.get_logger().info("🤖 Starting CORRECTED Yahboom Driver...")
        
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
        
        # MOTOR CALIBRATION PARAMETERS
        self.declare_parameter('motor_calibration_fl', 1.0)  # Front left
        self.declare_parameter('motor_calibration_fr', 1.0)  # Front right
        self.declare_parameter('motor_calibration_rl', 1.0)  # Rear left
        self.declare_parameter('motor_calibration_rr', 1.0)  # Rear right
        
        # DIRECTION CORRECTION PARAMETER
        self.declare_parameter('invert_forward_direction', True)  # Fix forward/backward
        
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
        
        # Motor calibration factors
        self.motor_cal = [
            self.get_parameter('motor_calibration_fl').value,
            self.get_parameter('motor_calibration_fr').value,
            self.get_parameter('motor_calibration_rl').value,
            self.get_parameter('motor_calibration_rr').value
        ]
        
        # Direction correction
        self.invert_forward = self.get_parameter('invert_forward_direction').value
        
        self.get_logger().info(f"🔧 Motor calibration: FL={self.motor_cal[0]:.3f}, FR={self.motor_cal[1]:.3f}, "
                              f"RL={self.motor_cal[2]:.3f}, RR={self.motor_cal[3]:.3f}")
        self.get_logger().info(f"🔄 Forward direction inverted: {self.invert_forward}")
        
        # Calculate distances from robot center to wheels
        self.lx = wheelbase_length / 2.0  
        self.ly = wheelbase_width / 2.0   
        
        # Initialize odometry calculator
        self.odometry = MecanumOdometry(
            self.wheel_radius, self.lx, self.ly, encoder_resolution
        )
        
        # Timing for odometry calculation
        self.last_odom_time = time.time()
        
        # Command tracking for debug output
        self.last_cmd_time = time.time()
        self.cmd_count = 0
        
        # Initialize hardware interface
        try:
            self.hardware = Rosmaster(
                car_type=1,  # X3 robot type
                com=self.serial_port,
                debug=False
            )
            self.get_logger().info(f"✅ Yahboom Rosmaster initialized on {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to initialize Yahboom hardware: {e}")
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
        
        # TF broadcaster for navigation
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create timer for publishing data
        self.create_timer(0.05, self.publish_sensor_data)  # 20Hz
        
        # Initialize hardware connection
        self.initialize_hardware()
        
    def initialize_hardware(self):
        """Initialize hardware connection"""
        if self.hardware is None:
            return
            
        self.get_logger().info("🔌 Starting Yahboom communication...")
        
        try:
            # Start the receive thread for sensor data
            self.hardware.create_receive_threading()
            time.sleep(0.1)
            
            # Enable PID for better control
            current_pid = self.hardware.get_motion_pid()
            if current_pid[0] == 0:  # PID disabled
                self.get_logger().info("⚙️ Enabling PID control...")
                self.hardware.set_pid_param(1.0, 0.1, 0.05, forever=False)
                time.sleep(0.1)
            
            # Test connection
            self.hardware.set_beep(50)
            time.sleep(0.1)
            
            # Get version info
            version = self.hardware.get_version()
            self.get_logger().info(f"📋 Firmware version: {version}")
            
            self.get_logger().info("✅ CORRECTED hardware initialized with calibration")
            
        except Exception as e:
            self.get_logger().error(f"❌ Hardware initialization failed: {e}")
            
    def cmd_vel_callback(self, msg):
        """Handle incoming velocity commands with direction correction"""
        if self.hardware is None:
            return
            
        self.cmd_count += 1
        current_time = time.time()
        
        # Extract velocities
        vx = msg.linear.x
        vy = msg.linear.y
        angular = msg.angular.z
        
        # APPLY DIRECTION CORRECTION
        if self.invert_forward:
            vx = -vx  # Fix forward/backward direction
        
        # Debug output (limited frequency)
        if current_time - self.last_cmd_time > 2.0:  # Every 2 seconds
            self.get_logger().info(
                f"📊 Commands: vx={vx:.2f}m/s (corrected), vy={vy:.2f}m/s, ω={angular:.2f}rad/s"
            )
            self.last_cmd_time = current_time
            self.cmd_count = 0
        
        # Send to hardware
        try:
            self.hardware.set_car_motion(vx, vy, angular)
        except Exception as e:
            self.get_logger().warn(f"⚠️ Motion command failed: {e}")
            
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
        
    def publish_sensor_data(self):
        """Publish all sensor data including corrected odometry"""
        if self.hardware is None:
            return
            
        current_time = self.get_clock().now()
        
        try:
            # Calculate accurate odometry from encoders
            self.calculate_and_publish_odometry(current_time)
            
            # Publish other sensor data
            self.publish_imu_data(current_time)
            self.publish_mag_data(current_time)
            self.publish_joint_states(current_time)
            self.publish_battery_data()
            
        except Exception as e:
            self.get_logger().warn(f"⚠️ Sensor publishing failed: {e}")
            
    def calculate_and_publish_odometry(self, timestamp):
        """Calculate and publish corrected odometry from encoders"""
        try:
            # Get encoder readings
            m1, m2, m3, m4 = self.hardware.get_motor_encoder()
            
            # Apply motor mapping with calibration
            encoder_counts = [
                m2 * self.motor_cal[0],  # Front Left
                m1 * self.motor_cal[1],  # Front Right
                m4 * self.motor_cal[2],  # Rear Left
                m3 * self.motor_cal[3]   # Rear Right
            ]
            
            # Calculate time step
            current_time = time.time()
            dt = current_time - self.last_odom_time
            self.last_odom_time = current_time
            
            # Update odometry
            x, y, theta, vx, vy, vtheta = self.odometry.update(encoder_counts, dt)
            
            # APPLY DIRECTION CORRECTION TO ODOMETRY
            if self.invert_forward:
                vx = -vx  # Correct velocity direction to match command correction
            
            # Create odometry message
            odom_msg = Odometry()
            odom_msg.header.stamp = timestamp.to_msg()
            odom_msg.header.frame_id = self.odom_frame_id
            odom_msg.child_frame_id = self.base_frame_id
            
            # Set position
            odom_msg.pose.pose.position.x = x
            odom_msg.pose.pose.position.y = y
            odom_msg.pose.pose.position.z = 0.0
            
            # Convert angle to quaternion
            quat = tf_transformations.quaternion_from_euler(0, 0, theta)
            odom_msg.pose.pose.orientation.x = quat[0]
            odom_msg.pose.pose.orientation.y = quat[1]
            odom_msg.pose.pose.orientation.z = quat[2]
            odom_msg.pose.pose.orientation.w = quat[3]
            
            # Set velocity
            odom_msg.twist.twist.linear.x = vx
            odom_msg.twist.twist.linear.y = vy
            odom_msg.twist.twist.angular.z = vtheta
            
            # Set covariance matrices
            odom_msg.pose.covariance[0] = 0.001   # x
            odom_msg.pose.covariance[7] = 0.001   # y
            odom_msg.pose.covariance[35] = 0.01   # yaw
            odom_msg.twist.covariance[0] = 0.001  # vx
            odom_msg.twist.covariance[7] = 0.001  # vy
            odom_msg.twist.covariance[35] = 0.01  # vyaw
            
            # Publish odometry
            self.odom_pub.publish(odom_msg)
            
            # Publish TF transform for navigation
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
        """Publish joint states with calibration applied"""
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
            encoder_to_rad = 2 * pi / self.odometry.encoder_resolution
            
            # Apply motor mapping with calibration
            joint_msg.position = [
                m2 * encoder_to_rad * self.motor_cal[0],  # front_left
                m1 * encoder_to_rad * self.motor_cal[1],  # front_right
                m4 * encoder_to_rad * self.motor_cal[2],  # rear_left
                m3 * encoder_to_rad * self.motor_cal[3]   # rear_right
            ]
            joint_msg.velocity = [0.0, 0.0, 0.0, 0.0]
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
        
    def destroy_node(self):
        """Clean up when shutting down"""
        self.get_logger().info("🛑 Shutting down corrected Yahboom driver")
        if self.hardware is not None:
            try:
                self.hardware.set_car_motion(0, 0, 0)
                self.hardware.reset_car_state()
            except:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        driver = YahboomHardwareDriver()
        rclpy.spin(driver)
    except KeyboardInterrupt:
        pass
    finally:
        if 'driver' in locals():
            driver.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()