#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from std_msgs.msg import String


class SensorStation(LifecycleNode):
    def __init__(self, **kwargs):
        super().__init__("sensor_station", **kwargs)
        self.declare_parameter("sensor_name", "sensor_station")
        self.declare_parameter("topic_name", "sensor_data")
        self.declare_parameter("publish_frequency", 1.0)

        self.pub = None
        self.timer = None
        self.count = 0

        self.get_logger().info(f"[{self.get_name()}] Constructed in UNCONFIGURED state.")

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.sensor_name = (
            self.get_parameter("sensor_name").get_parameter_value().string_value
        )
        self.topic_name = (
            self.get_parameter("topic_name").get_parameter_value().string_value
        )
        self.publish_frequency = (
            self.get_parameter("publish_frequency").get_parameter_value().double_value
        )

        if self.publish_frequency <= 0.0:
            self.get_logger().error(f"[{self.get_name()}] Publish frequency must be > 0.0!")
            return TransitionCallbackReturn.FAILURE

        # Create Lifecycle Publisher (inactive by default)
        self.pub = self.create_lifecycle_publisher(String, self.topic_name, 10)

        # Create periodic timer
        timer_period = 1.0 / self.publish_frequency
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f"[{self.get_name()}] on_configure: Initialized publisher on topic '{self.topic_name}' "
            f"at {self.publish_frequency:.1f} Hz. State is now INACTIVE."
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(
            f"[{self.get_name()}] on_activate: LifecyclePublisher is now ACTIVE and broadcasting messages."
        )
        # super().on_activate activates registered lifecycle publishers
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(
            f"[{self.get_name()}] on_deactivate: LifecyclePublisher is now INACTIVE (messages suppressed)."
        )
        # super().on_deactivate deactivates registered lifecycle publishers
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        if self.timer is not None:
            self.destroy_timer(self.timer)
            self.timer = None
        if self.pub is not None:
            self.destroy_publisher(self.pub)
            self.pub = None
        self.count = 0

        self.get_logger().info(
            f"[{self.get_name()}] on_cleanup: Destroyed publisher and timer. State is now UNCONFIGURED."
        )
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        if self.timer is not None:
            self.destroy_timer(self.timer)
            self.timer = None
        if self.pub is not None:
            self.destroy_publisher(self.pub)
            self.pub = None

        self.get_logger().info(
            f"[{self.get_name()}] on_shutdown: Resources cleaned up. State is now FINALIZED."
        )
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().error(f"[{self.get_name()}] on_error: Error detected, recovering...")
        if self.timer is not None:
            self.destroy_timer(self.timer)
            self.timer = None
        if self.pub is not None:
            self.destroy_publisher(self.pub)
            self.pub = None
        return TransitionCallbackReturn.SUCCESS

    def timer_callback(self):
        msg = String()
        msg.data = f"[{self.sensor_name}] reading #{self.count}"
        self.count += 1

        if self.pub is not None and self.pub.is_activated:
            self.get_logger().info(f"[ACTIVE] Publishing -> '{msg.data}'")
            self.pub.publish(msg)
        else:
            self.get_logger().info(
                f"[{self.get_name()} - INACTIVE] Timer firing, but LifecyclePublisher is disabled (messages suppressed)",
                throttle_duration_sec=2.0,
            )


def main(args=None):
    rclpy.init(args=args)
    node = SensorStation()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
