#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped

class PS4ControllerNode(Node):
    def __init__(self):
        super().__init__('ps4_controller_node')
        
        # Subscription to the /joy topic for PS4 controller input
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Publisher for the bicycle steering controller
        self.cmd_pub = self.create_publisher(TwistStamped, '/bicycle_steering_controller/reference', 10)
        
        # Timer to publish at a fixed rate (30 Hz)
        self.timer = self.create_timer(1.0 / 30, self.publish_cmd)
        
        # Initialize storage for joystick input
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0

    def joy_callback(self, msg):
        # Map joystick axes to linear and angular velocities
        r2_value = msg.axes[5] * -1.0
        l2_value = msg.axes[2] * -1.0
        self.current_linear_x = (r2_value - l2_value) / 2
        self.current_angular_z = msg.axes[0] * 1.0  # Left joystick horizontal (scaled to 1.0 rad/s max)

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
