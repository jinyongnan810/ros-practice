#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient


class AccClient(Node):
    """A simple ROS2 node that creates an action client for the action "accumulate"."""

    def __init__(self):
        super().__init__("acc_client_node")
        self.get_logger().info("Accumulate Action Client Node has been started!")
        # Create an action client


def main(args=None):
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)

    # Create a ROS2 node, then spin the node to keep it alive, finally destroy the node
    node = AccClient()
    rclpy.spin(node)
    node.destroy_node()

    # Shutdown the ROS2 Python client library
    rclpy.shutdown()


if __name__ == "__main__":
    main()
