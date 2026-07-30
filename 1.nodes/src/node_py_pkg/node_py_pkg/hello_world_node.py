#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class HelloWorldNode(Node):
    """A simple ROS2 node that prints "Hello World!" every second."""

    def __init__(self):
        super().__init__("hello_world_node")
        self.get_logger().info("Hello World Node has been started!")

        # Implementation 1
        # while rclpy.ok():
        #     self.timer_callback()
        #     self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1))

        # Implementation 2
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        """Callback function that is called every second."""
        self.get_logger().info("Hello World!")


def main(args=None):
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)

    # Create a ROS2 node, then spin the node to keep it alive, finally destroy the node
    node = HelloWorldNode()
    rclpy.spin(node)
    node.destroy_node()

    # Shutdown the ROS2 Python client library
    rclpy.shutdown()


if __name__ == "__main__":
    main()
