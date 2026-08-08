#!/usr/bin/env python3
from functools import partial

import rclpy
from rclpy.node import Node

from custom_interfaces.srv import Acc


class AccClient(Node):
    """A simple ROS2 node that creates a client for the service "accumulate" and accumulates the received numbers."""

    def __init__(self):
        super().__init__("acc_client_node")
        self.get_logger().info("Accumulate Client Node has been started!")
        # Create a client
        self.cli = self.create_client(Acc, "accumulate")

    def add(self, a: int, b: int, c: int):
        """Method to call the service "accumulate" with the given numbers."""
        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")
        # Create a request
        req = Acc.Request()
        req.a = a
        req.b = b
        req.c = c
        # Call the service asynchronously
        future = self.cli.call_async(req)
        # Add a callback to handle the response
        # Use partial to pass the arguments to the callback
        future.add_done_callback(partial(self.response_callback, request=req))

    def response_callback(self, future, request):
        """Callback function for the service response."""
        try:
            response = future.result()
            self.get_logger().info(
                f"Result of {request.a} + {request.b} + {request.c}: {response.sum}"
            )
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main(args=None):
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)

    # Create a ROS2 node, then spin the node to keep it alive, finally destroy the node
    node = AccClient()
    node.add(1, 2, 3)
    node.add(4, 5, 6)
    node.add(7, 8, 9)
    rclpy.spin(node)
    node.destroy_node()

    # Shutdown the ROS2 Python client library
    rclpy.shutdown()


if __name__ == "__main__":
    main()
