#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SensorMonitor(Node):
    def __init__(self):
        super().__init__("sensor_monitor")
        self.declare_parameter("topic_name", "sensor_data")
        topic_name = (
            self.get_parameter("topic_name").get_parameter_value().string_value
        )

        self.received_count = 0
        self.sub = self.create_subscription(
            String, topic_name, self.topic_callback, 10
        )
        self.get_logger().info(
            f"SensorMonitor started. Listening to '{topic_name}' for lifecycle publisher stream..."
        )

    def topic_callback(self, msg: String):
        self.received_count += 1
        self.get_logger().info(
            f"[MONITOR #{self.received_count}] Live message received: '{msg.data}'"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SensorMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
