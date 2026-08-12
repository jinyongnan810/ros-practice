#!/usr/bin/env python3
from datetime import datetime

import rclpy
from rclpy.node import Node

from custom_interfaces.msg import News

# from std_msgs.msg import String


class NewsStationNode(Node):
    """A simple ROS2 node that periodically publishes updates to the "news" topic."""

    def __init__(self):
        super().__init__("news_station_node")
        self.counter = 0
        timer_interval = self.declare_parameter("timer_interval", 1.0).value
        if timer_interval <= 0:
            raise ValueError("timer_interval must be greater than 0 seconds")
        self.get_logger().info("News Station Node has been started!")
        # Create a publisher
        # 10 is the queue size, which is the maximum number of messages that can be buffered before being sent to subscribers
        self.publisher_ = self.create_publisher(News, "news", 10)
        # self.publisher_ = self.create_publisher(String, "news", 10)
        # Create a timer that calls the timer_callback function periodically
        self.create_timer(timer_interval, self.timer_callback)

    def timer_callback(self):
        """Publish the next news update."""
        msg = News()
        # msg = String()
        msg.datetime = datetime.now().isoformat(timespec="seconds")
        msg.title = "News update"
        msg.content = f"Counter: {self.counter}"
        # msg.data = f"News update! {self.counter}"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published [{msg.datetime}] {msg.title}: {msg.content}")
        # self.get_logger().info(f"published News update! {self.counter}")
        self.counter += 1


def main(args=None):
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)

    # Create a ROS2 node, then spin the node to keep it alive, finally destroy the node
    node = NewsStationNode()
    rclpy.spin(node)
    node.destroy_node()

    # Shutdown the ROS2 Python client library
    rclpy.shutdown()


if __name__ == "__main__":
    main()
