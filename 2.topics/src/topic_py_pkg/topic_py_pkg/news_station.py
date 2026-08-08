#!/usr/bin/env python3
from datetime import datetime

import rclpy
from rclpy.node import Node

from custom_interfaces.msg import News

# from std_msgs.msg import String


class NewsStationNode(Node):
    """A simple ROS2 node that creates topic "news" and publishes "News update!" every second."""

    def __init__(self):
        super().__init__("news_station_node")
        self.counter = 0
        self.get_logger().info("News Station Node has been started!")
        # Create a publisher
        # 10 is the queue size, which is the maximum number of messages that can be buffered before being sent to subscribers
        self.publisher_ = self.create_publisher(News, "news", 10)
        # self.publisher_ = self.create_publisher(String, "news", 10)
        # Create a timer that calls the timer_callback function every second
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        """Callback function that is called every second."""
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
