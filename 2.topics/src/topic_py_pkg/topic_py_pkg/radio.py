#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RadioNode(Node):
    """A simple ROS2 node that subscribes to topic "news" and logs the received messages."""

    def __init__(self):
        super().__init__("radio_node")
        self.get_logger().info("Radio Node has been started!")
        # Create a subscriber
        # 10 is the queue size, which is the maximum number of messages that can be buffered before being sent to the callback function
        self.subscription = self.create_subscription(
            String, "news", self.listener_callback, 10
        )

    def listener_callback(self, msg):
        """Callback function that is called when a message is received."""
        self.get_logger().info(f"Received: {msg.data}")


def main(args=None):
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)

    # Create a ROS2 node, then spin the node to keep it alive, finally destroy the node
    node = RadioNode()
    rclpy.spin(node)
    node.destroy_node()

    # Shutdown the ROS2 Python client library
    rclpy.shutdown()


if __name__ == "__main__":
    main()
