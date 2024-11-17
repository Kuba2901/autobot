import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistBridgeNode(Node):
    def __init__(self):
        super().__init__('twist_bridge_node')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',  # Topic published by teleop_twist_keyboard
            self.twist_callback,
            10
        )
        self.publisher = self.create_publisher(
            TwistStamped,
            '/bicycle_steering_controller/reference',  # Target topic
            10
        )

    def twist_callback(self, msg):
        # Create a TwistStamped message
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.twist = msg  # Assign the Twist data

        # Publish the TwistStamped message
        self.publisher.publish(twist_stamped)

def main(args=None):
    rclpy.init(args=args)
    node = TwistBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
