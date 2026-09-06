#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
class TurtleActionServer(Node):
    """ROS 2 Node for controlling turtlesim via action server (Action to be implemented)."""

    def __init__(self):
        super().__init__("turtle_action_server")
        self.get_logger().info("Turtle Action Server Node has been started (no actions yet)!")


def main(args=None):
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)

    # Create a ROS2 node, then spin the node to keep it alive, finally destroy the node
    node = TurtleActionServer()
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
