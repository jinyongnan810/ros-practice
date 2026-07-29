#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


def main(args=None):
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)

    # Create a ROS2 node, and print a log message, then spin the node to keep it alive, finally destroy the node
    node = Node("first_node")
    node.get_logger().info("Hello ROS2")
    rclpy.spin(node)
    node.destroy_node()

    # Shutdown the ROS2 Python client library
    rclpy.shutdown()


if __name__ == "__main__":
    main()
