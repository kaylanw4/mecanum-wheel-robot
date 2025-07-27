#!/usr/bin/env python3
"""
Yahboom X3 Robot Hardware Driver for ROS2
Communicates with Yahboom Robot Expansion Board V3.0 over USB serial
Uses the official Yahboom Rosmaster_Lib library
"""

import sys
import math
import time
import threading
from math import pi
from time import sleep

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, MagneticField, JointState
from std_msgs.msg import Float32, Int32, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# Import the official Yahboom library
from Rosmaster_Lib import Rosmaster


class YahboomHardwareDriver(Node):
    """
    ROS2 Node for Yahboom Hardware Driver using official Rosmaster_Lib
    """
    
    def __init__(self):
        super().__init__('yahboom_hardware_driver')
        
        self.get_logger().info("🤖 Starting Yahboom Hardware Driver with Rosmaster_Lib...")
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('serial_baudrate', 115200)
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('publish_odom_tf', True)
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.serial_baudrate = self.get_parameter('serial_baudrate').value
        self.imu_frame_id = self.get_parameter('imu_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.publish_odom_tf = self.get_parameter('publish_odom_tf').value
        
        # Command tracking for debug output
        self.last_cmd_time = time.time()
        self.cmd_count = 0
        
        # Initialize hardware interface with real Yahboom library
        try:
            self.hardware = Rosmaster(
                car_type=1,  # X3 robot type
                com=self.serial_port,
                debug=True  # Enable debug output from Yahboom library
            )
            self.get_logger().info(f"✅ Yahboom Rosmaster initialized on {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to initialize Yahboom hardware: {e}")
            self.get_logger().error("🔍 Check:")
            self.get_logger().error("   - USB cable connected?")
            self.get_logger().error("   - Robot powered on?") 
            self.get_logger().error("   - Correct port? Try: ls /dev/tty{USB,ACM}*")
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
        
        # TF broadcaster for odometry
        if self.publish_odom_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create timer for publishing sensor data
        self.create_timer(0.1, self.publish_sensor_data)  # 10Hz
        
        # Initialize hardware connection
        self.initialize_hardware()
        
    def initialize_hardware(self):
        """Initialize hardware connection"""
        if self.hardware is None:
            return
            
        self.get_logger().info("🔌 Starting Yahboom communication thread...")
        
        try:
            # Start the receive thread for sensor data
            self.hardware.create_receive_threading()
            time.sleep(0.1)
            
            # Test connection with a beep
            self.hardware.set_beep(50)  # 50ms beep
            time.sleep(0.1)
            
            # Get version info
            version = self.hardware.get_version()
            self.get_logger().info(f"📋 Yahboom firmware version: {version}")
            
            self.get_logger().info("✅ Yahboom hardware driver initialized successfully")
            self.get_logger().info("🎮 Ready to receive /cmd_vel commands...")
            
        except Exception as e:
            self.get_logger().error(f"❌ Hardware initialization failed: {e}")
            
    def cmd_vel_callback(self, msg):
        """Handle incoming velocity commands"""
        if self.hardware is None:
            return
            
        self.cmd_count += 1
        current_time = time.time()
        
        # Extract velocities
        vx = msg.linear.x
        vy = msg.linear.y  # For mecanum wheels
        angular = msg.angular.z
        
        # Debug output (limited frequency to avoid spam)
        if current_time - self.last_cmd_time > 1.0:  # Every second
            self.get_logger().info(
                f"📊 Received {self.cmd_count} commands in last second. "
                f"Current: vx={vx:.2f}, vy={vy:.2f}, ω={angular:.2f}"
            )
            self.last_cmd_time = current_time
            self.cmd_count = 0
        
        # Send to hardware using real Yahboom library
        try:
            self.hardware.set_car_motion(vx, vy, angular)
        except Exception as e:
            self.get_logger().warn(f"⚠️  Failed to send motion command: {e}")
            
    def rgb_light_callback(self, msg):
        """Handle RGB light control"""
        if self.hardware is None:
            return
            
        try:
            self.get_logger().info(f"💡 Setting RGB pattern: {msg.data}")
            # Use the real Yahboom RGB control method
            self.hardware.set_colorful_effect(msg.data, 6, parm=1)
        except Exception as e:
            self.get_logger().warn(f"⚠️  RGB control failed: {e}")
        
    def buzzer_callback(self, msg):
        """Handle buzzer control"""
        if self.hardware is None:
            return
            
        try:
            # Convert boolean to buzzer time (0=off, >0=on time in ms)
            buzzer_time = 100 if msg.data else 0
            self.get_logger().info(f"🔔 Buzzer: {'ON' if msg.data else 'OFF'}")
            self.hardware.set_beep(buzzer_time)
        except Exception as e:
            self.get_logger().warn(f"⚠️  Buzzer control failed: {e}")
        
    def publish_sensor_data(self):
        """Publish all sensor data"""
        if self.hardware is None:
            return
            
        current_time = self.get_clock().now()
        
        try:
            # Publish IMU data
            self.publish_imu_data(current_time)
            
            # Publish magnetometer data
            self.publish_mag_data(current_time)
            
            # Publish joint states
            self.publish_joint_states(current_time)
            
            # Publish battery voltage
            self.publish_battery_data()
            
            # Publish odometry
            self.publish_odometry(current_time)
            
        except Exception as e:
            self.get_logger().warn(f"⚠️  Sensor data publishing failed: {e}")
        
    def publish_imu_data(self, timestamp):
        """Publish IMU sensor data"""
        imu_msg = Imu()
        imu_msg.header.stamp = timestamp.to_msg()
        imu_msg.header.frame_id = self.imu_frame_id
        
        # Get accelerometer and gyroscope data from real hardware
        ax, ay, az = self.hardware.get_accelerometer_data()
        gx, gy, gz = self.hardware.get_gyroscope_data()
        
        # Fill IMU message
        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az
        
        imu_msg.angular_velocity.x = gx
        imu_msg.angular_velocity.y = gy
        imu_msg.angular_velocity.z = gz
        
        # Set covariance matrices (unknown covariance)
        imu_msg.linear_acceleration_covariance[0] = -1
        imu_msg.angular_velocity_covariance[0] = -1
        imu_msg.orientation_covariance[0] = -1
        
        # Publish
        self.imu_pub.publish(imu_msg)
        
    def publish_mag_data(self, timestamp):
        """Publish magnetometer data"""
        mag_msg = MagneticField()
        mag_msg.header.stamp = timestamp.to_msg()
        mag_msg.header.frame_id = self.imu_frame_id
        
        # Get magnetometer data from real hardware
        mx, my, mz = self.hardware.get_magnetometer_data()
        
        mag_msg.magnetic_field.x = mx
        mag_msg.magnetic_field.y = my
        mag_msg.magnetic_field.z = mz
        
        # Set covariance (unknown)
        mag_msg.magnetic_field_covariance[0] = -1
        
        self.mag_pub.publish(mag_msg)
        
    def publish_joint_states(self, timestamp):
        """Publish joint states for the wheels"""
        joint_msg = JointState()
        joint_msg.header.stamp = timestamp.to_msg()
        joint_msg.header.frame_id = 'joint_states'
        
        # Define joint names (matching your URDF)
        joint_msg.name = [
            'front_left_wheel_joint',
            'front_right_wheel_joint', 
            'rear_left_wheel_joint',
            'rear_right_wheel_joint'
        ]
        
        # Get encoder data if available
        try:
            # The Yahboom library has get_motor_encoder() method
            m1, m2, m3, m4 = self.hardware.get_motor_encoder()
            
            # Convert encoder counts to wheel positions (radians)
            # This is a simplified conversion - you may need to adjust based on your wheel/encoder specs
            encoder_to_rad = 2 * pi / 2080  # 2080 counts per revolution (520 CPR * 4)
            
            joint_msg.position = [
                m2 * encoder_to_rad,  # front_left
                m1 * encoder_to_rad,  # front_right  
                m4 * encoder_to_rad,  # rear_left
                m3 * encoder_to_rad   # rear_right
            ]
            
            # Set velocities to 0 for now (could calculate from position changes)
            joint_msg.velocity = [0.0, 0.0, 0.0, 0.0]
            joint_msg.effort = [0.0, 0.0, 0.0, 0.0]
            
        except Exception as e:
            # Fallback to zeros if encoder reading fails
            joint_msg.position = [0.0, 0.0, 0.0, 0.0]
            joint_msg.velocity = [0.0, 0.0, 0.0, 0.0]
            joint_msg.effort = [0.0, 0.0, 0.0, 0.0]
        
        self.joint_states_pub.publish(joint_msg)
        
    def publish_battery_data(self):
        """Publish battery voltage"""
        battery_msg = Float32()
        battery_msg.data = self.hardware.get_battery_voltage()
        self.battery_pub.publish(battery_msg)
        
    def publish_odometry(self, timestamp):
        """Publish odometry data"""
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp.to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.base_frame_id
        
        # Get motion data from real hardware
        vx, vy, angular = self.hardware.get_motion_data()
        
        # Fill velocity data
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = angular
        
        # For now, position is 0 (you could integrate velocities for real odometry)
        odom_msg.pose.pose.position.x = 0.0
        odom_msg.pose.pose.position.y = 0.0
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.w = 1.0
        
        # Set covariance matrices
        odom_msg.pose.covariance[0] = 0.001   # x
        odom_msg.pose.covariance[7] = 0.001   # y
        odom_msg.pose.covariance[35] = 0.01   # yaw
        odom_msg.twist.covariance[0] = 0.001  # vx
        odom_msg.twist.covariance[7] = 0.001  # vy
        odom_msg.twist.covariance[35] = 0.01  # vyaw
        
        self.odom_pub.publish(odom_msg)
        
        # Publish TF if enabled
        if self.publish_odom_tf:
            t = TransformStamped()
            t.header.stamp = timestamp.to_msg()
            t.header.frame_id = self.odom_frame_id
            t.child_frame_id = self.base_frame_id
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t)
            
    def destroy_node(self):
        """Clean up when shutting down"""
        self.get_logger().info("🛑 Shutting down Yahboom hardware driver")
        if self.hardware is not None:
            try:
                # Stop the robot
                self.hardware.set_car_motion(0, 0, 0)
                # Reset robot state (stops, turns off lights, etc.)
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