#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class AccClient(Node):
    """A simple ROS2 node that creates a client for the service "accumulate" and accumulates the received numbers."""

    def __init__(self):
        super().__init__("acc_client_node")
        self.get_logger().info("Accumulate Client Node has been started!")
        # Create a client

    def accumulate_callback(self, request, response):
        return response


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
