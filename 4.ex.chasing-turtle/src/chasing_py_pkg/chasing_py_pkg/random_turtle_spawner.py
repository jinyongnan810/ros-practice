#!/usr/bin/env python3
import math
import random

import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn


class RandomTurtleSpawner(Node):
    def __init__(self):
        super().__init__("random_turtle_spawner")

        duration = self.declare_parameter("duration", 2.0).value
        if duration <= 0.0:
            raise ValueError("duration must be greater than zero")

        self.spawn_client = self.create_client(Spawn, "/spawn")
        self.request_pending = False
        self.timer = self.create_timer(duration, self.spawn_turtle)
        self.get_logger().info(f"Spawning a random turtle every {duration:.2f} seconds")

    def spawn_turtle(self):
        if not self.spawn_client.service_is_ready():
            self.get_logger().warning("/spawn service is not available yet")
            return

        if self.request_pending:
            self.get_logger().warning("Previous spawn request is still pending")
            return

        request = Spawn.Request()
        request.x = random.uniform(1.0, 10.0)
        request.y = random.uniform(1.0, 10.0)
        request.theta = random.uniform(0.0, 2.0 * math.pi)
        self.request_pending = True

        future = self.spawn_client.call_async(request)
        future.add_done_callback(
            lambda completed_future: self.spawn_finished(completed_future, request)
        )

    def spawn_finished(self, future, request):
        self.request_pending = False
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"Spawn request failed: {error}")
            return

        self.get_logger().info(
            f"Spawned {response.name} at ({request.x:.2f}, {request.y:.2f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = RandomTurtleSpawner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
