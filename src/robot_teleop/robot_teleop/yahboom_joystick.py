#!/usr/bin/env python3
"""
Enhanced Joystick Controller for Yahboom Robot
Adds gear control, RGB lights, buzzer, and other Yahboom-specific features
"""

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32, Bool


class YahboomJoystickController(Node):
    """
    Enhanced joystick controller with Yahboom-specific features
    """
    
    def __init__(self):
        super().__init__('yahboom_joystick_controller')
        
        # Declare parameters
        self.declare_parameter('max_linear_vel', 1.0)
        self.declare_parameter('max_angular_vel', 2.0)
        self.declare_parameter('deadzone', 0.1)
        
        # Get parameters
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.deadzone = self.get_parameter('deadzone').value
        
        # Control state variables
        self.linear_gear = 1.0
        self.angular_gear = 1.0
        self.rgb_index = 0
        self.buzzer_active = False
        self.joy_active = True
        self.last_gear_change_time = 0.0
        self.last_rgb_change_time = 0.0
        self.last_buzzer_change_time = 0.0
        
        # Create subscribers
        self.joy_sub = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)
            
        # Create publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.rgb_light_pub = self.create_publisher(Int32, 'rgb_light', 10)
        self.buzzer_pub = self.create_publisher(Bool, 'buzzer', 10)
        
        self.get_logger().info("Yahboom Joystick Controller initialized")
        self.get_logger().info("Controls:")
        self.get_logger().info("  Left stick: Forward/backward and strafe")
        self.get_logger().info("  Right stick X: Rotation")
        self.get_logger().info("  L1 (Button 4): Enable movement (deadman switch)")
        self.get_logger().info("  R1 (Button 5): Turbo mode")
        self.get_logger().info("  Triangle (Button 2): Linear gear control")
        self.get_logger().info("  Square (Button 3): Angular gear control")
        self.get_logger().info("  Circle (Button 1): RGB light cycle")
        self.get_logger().info("  X (Button 0): Buzzer toggle")
        
    def joy_callback(self, joy_msg):
        """Handle joystick input"""
        if not isinstance(joy_msg, Joy):
            return
            
        current_time = time.time()
        
        # Button assignments for PS4 controller
        # Buttons: X=0, Circle=1, Triangle=2, Square=3, L1=4, R1=5, etc.
        # Axes: Left stick X=0, Left stick Y=1, Right stick X=2, Right stick Y=3, L2=4, R2=5
        
        # Check enable button (L1 - deadman switch)
        enable_pressed = len(joy_msg.buttons) > 4 and joy_msg.buttons[4] == 1
        turbo_pressed = len(joy_msg.buttons) > 5 and joy_msg.buttons[5] == 1
        
        # Handle gear controls (with debouncing)
        if (len(joy_msg.buttons) > 2 and joy_msg.buttons[2] == 1 and 
            current_time - self.last_gear_change_time > 0.5):
            self._cycle_linear_gear()
            self.last_gear_change_time = current_time
            
        if (len(joy_msg.buttons) > 3 and joy_msg.buttons[3] == 1 and 
            current_time - self.last_gear_change_time > 0.5):
            self._cycle_angular_gear()
            self.last_gear_change_time = current_time
            
        # Handle RGB light control (Circle button)
        if (len(joy_msg.buttons) > 1 and joy_msg.buttons[1] == 1 and 
            current_time - self.last_rgb_change_time > 0.5):
            self._cycle_rgb_light()
            self.last_rgb_change_time = current_time
            
        # Handle buzzer control (X button)
        if (len(joy_msg.buttons) > 0 and joy_msg.buttons[0] == 1 and 
            current_time - self.last_buzzer_change_time > 0.5):
            self._toggle_buzzer()
            self.last_buzzer_change_time = current_time
        
        # Handle movement commands
        if enable_pressed and self.joy_active:
            self._handle_movement(joy_msg, turbo_pressed)
        else:
            # Send stop command when not enabled
            self._send_stop_command()
            
    def _handle_movement(self, joy_msg, turbo_mode):
        """Process movement commands from joystick"""
        # Get stick values (with bounds checking)
        left_x = joy_msg.axes[0] if len(joy_msg.axes) > 0 else 0.0
        left_y = joy_msg.axes[1] if len(joy_msg.axes) > 1 else 0.0
        right_x = joy_msg.axes[2] if len(joy_msg.axes) > 2 else 0.0
        
        # Apply deadzone
        left_x = self._apply_deadzone(left_x)
        left_y = self._apply_deadzone(left_y)
        right_x = self._apply_deadzone(right_x)
        
        # Calculate velocities
        # Left stick Y: forward/backward (note: joystick Y is typically inverted)
        vx = -left_y * self.max_linear_vel * self.linear_gear
        
        # Left stick X: strafe left/right
        vy = left_x * self.max_linear_vel * self.linear_gear
        
        # Right stick X: rotation
        angular = right_x * self.max_angular_vel * self.angular_gear
        
        # Apply turbo multiplier
        if turbo_mode:
            vx *= 2.0
            vy *= 2.0
            angular *= 1.5
            
        # Apply limits
        vx = max(-self.max_linear_vel, min(self.max_linear_vel, vx))
        vy = max(-self.max_linear_vel, min(self.max_linear_vel, vy))
        angular = max(-self.max_angular_vel, min(self.max_angular_vel, angular))
        
        # Create and publish twist message
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = angular
        
        self.cmd_vel_pub.publish(twist)
        
        # Log movement for debugging (at reduced rate)
        if time.time() % 1.0 < 0.1:  # Log once per second
            self.get_logger().debug(
                f"Movement: vx={vx:.2f}, vy={vy:.2f}, angular={angular:.2f}, "
                f"gear=L{self.linear_gear:.2f}/A{self.angular_gear:.2f}, turbo={turbo_mode}"
            )
            
    def _send_stop_command(self):
        """Send stop command to robot"""
        twist = Twist()
        # All velocities default to 0.0
        self.cmd_vel_pub.publish(twist)
        
    def _apply_deadzone(self, value):
        """Apply deadzone to joystick input"""
        if abs(value) < self.deadzone:
            return 0.0
        return value
        
    def _cycle_linear_gear(self):
        """Cycle through linear gear ratios"""
        if self.linear_gear == 1.0:
            self.linear_gear = 1.0 / 3.0
        elif self.linear_gear == 1.0 / 3.0:
            self.linear_gear = 2.0 / 3.0
        elif self.linear_gear == 2.0 / 3.0:
            self.linear_gear = 1.0
            
        self.get_logger().info(f"Linear gear: {self.linear_gear:.2f}")
        
    def _cycle_angular_gear(self):
        """Cycle through angular gear ratios"""
        if self.angular_gear == 1.0:
            self.angular_gear = 1.0 / 4.0
        elif self.angular_gear == 1.0 / 4.0:
            self.angular_gear = 1.0 / 2.0
        elif self.angular_gear == 1.0 / 2.0:
            self.angular_gear = 3.0 / 4.0
        elif self.angular_gear == 3.0 / 4.0:
            self.angular_gear = 1.0
            
        self.get_logger().info(f"Angular gear: {self.angular_gear:.2f}")
        
    def _cycle_rgb_light(self):
        """Cycle through RGB light patterns"""
        rgb_msg = Int32()
        rgb_msg.data = self.rgb_index
        
        # Publish multiple times to ensure it's received
        for _ in range(3):
            self.rgb_light_pub.publish(rgb_msg)
            
        self.get_logger().info(f"RGB pattern: {self.rgb_index}")
        
        # Cycle to next pattern
        self.rgb_index = (self.rgb_index + 1) % 7  # 7 different patterns
        
    def _toggle_buzzer(self):
        """Toggle buzzer on/off"""
        self.buzzer_active = not self.buzzer_active
        
        buzzer_msg = Bool()
        buzzer_msg.data = self.buzzer_active
        
        # Publish multiple times to ensure it's received
        for _ in range(3):
            self.buzzer_pub.publish(buzzer_msg)
            
        self.get_logger().info(f"Buzzer: {'ON' if self.buzzer_active else 'OFF'}")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = YahboomJoystickController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        if 'controller' in locals():
            controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()