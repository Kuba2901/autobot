#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String

class PS4ControllerNode(Node):
    def __init__(self):
        super().__init__('ps4_controller_node')
        
        # Existing subscriptions and publishers
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/bicycle_steering_controller/reference', 10)
        
        # Light control publisher
        self.light_pub = self.create_publisher(String, '/light_command', 10)
        
        # Timer to publish at a fixed rate (30 Hz)
        self.timer = self.create_timer(1.0 / 30, self.publish_cmd)
        
        # Initialize storage for joystick input
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
        
        # Light control state
        self.circle_button_previous = False

    def joy_callback(self, msg):
        # Existing joystick mapping
        r2_value = msg.axes[5] * -1.0
        l2_value = msg.axes[2] * -1.0
        self.current_linear_x = (r2_value - l2_value) / 2
        self.current_angular_z = msg.axes[0] * 1.0

        # Light control using Circle button (button index 1)
        circle_button = msg.buttons[1]
    
        # Add debug print
        self.get_logger().debug(f'Circle button state: {circle_button}')
        
        # Detect button press (transition from not pressed to pressed)
        if circle_button and not self.circle_button_previous:
            light_cmd = String()
            light_cmd.data = "L,TOGGLE"
            self.light_pub.publish(light_cmd)
            # Add more detailed debug print
            self.get_logger().info('Circle button pressed - Sending light toggle command')
            
        self.circle_button_previous = circle_button

    def publish_cmd(self):
        # Create and publish a TwistStamped message
        twist_stamped = TwistStamped()
        twist_stamped.twist.linear.x = self.current_linear_x
        twist_stamped.twist.linear.y = 0.0
        twist_stamped.twist.linear.z = 0.0
        twist_stamped.twist.angular.x = 0.0
        twist_stamped.twist.angular.y = 0.0
        twist_stamped.twist.angular.z = self.current_angular_z

        self.cmd_pub.publish(twist_stamped)

def main(args=None):
    rclpy.init(args=args)
    node = PS4ControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()