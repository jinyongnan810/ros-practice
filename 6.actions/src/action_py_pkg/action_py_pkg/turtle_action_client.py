#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
class TurtleActionClient(Node):
    """ROS 2 Node for client requests to turtle action server (Action to be implemented)."""

    def __init__(self):
        super().__init__("turtle_action_client")
        self.get_logger().info("Turtle Action Client Node has been started (no actions yet)!")


def main(args=None):
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)

    # Create a ROS2 node, then spin the node to keep it alive, finally destroy the node
    node = TurtleActionClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    # Shutdown the ROS2 Python client library
    rclpy.shutdown()


if __name__ == "__main__":
    main()
