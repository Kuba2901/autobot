#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import Joy

class PS4TwistBridgeNode(Node):
    def __init__(self):
        super().__init__('ps4_twist_bridge_node')
        
        # Parameters for control scaling
        self.linear_scale = 1.0   # Max linear velocity
        self.angular_scale = 1.0  # Max angular velocity
        
        # Create subscription to joy topic
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',  # Topic published by joy_node
            self.joy_callback,
            10
        )
        
        # Create publisher for TwistStamped messages
        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/bicycle_steering_controller/reference',
            10
        )
        
        # Initialize message
        self.twist_stamped = TwistStamped()
        
        self.get_logger().info('PS4 Twist Bridge Node started')

    def joy_callback(self, msg):
        """
        PS4 controller mapping:
        Left stick up/down: Linear velocity (msg.axes[1])
        Right stick left/right: Angular velocity (msg.axes[2])
        """
        # Create and fill TwistStamped message
        self.twist_stamped.header.stamp = self.get_clock().now().to_msg()
        
        # Linear velocity (left stick up/down)
        self.twist_stamped.twist.linear.x = msg.axes[1] * self.linear_scale
        self.twist_stamped.twist.linear.y = 0.0
        self.twist_stamped.twist.linear.z = 0.0
        
        # Angular velocity (right stick left/right)
        self.twist_stamped.twist.angular.x = 0.0
        self.twist_stamped.twist.angular.y = 0.0
        self.twist_stamped.twist.angular.z = msg.axes[2] * self.angular_scale
        
        # Publish the message
        self.twist_pub.publish(self.twist_stamped)

def main(args=None):
    rclpy.init(args=args)
    node = PS4TwistBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()