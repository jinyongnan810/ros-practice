#!/usr/bin/env python3
import math
import random

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import Kill, Spawn

from chasing_interfaces.msg import Target, TargetPositions


class RandomTurtleSpawner(Node):
    def __init__(self):
        super().__init__("random_turtle_spawner")

        duration = self.declare_parameter("duration", 2.0).value
        if duration <= 0.0:
            raise ValueError("duration must be greater than zero")

        self.spawn_client = self.create_client(Spawn, "/spawn")
        self.kill_client = self.create_client(Kill, "/kill")
        self.positions_publisher = self.create_publisher(
            TargetPositions, "/spawned_target_positions", 10
        )
        self.pose_subscription = self.create_subscription(
            Pose, "/turtle1/pose", self.handle_pose, 10
        )
        self.target_positions = TargetPositions()
        self.pending_kills = set()
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

        target = Target(
            name=response.name,
            position=Point(x=float(request.x), y=float(request.y)),
        )
        self.target_positions.targets.append(target)
        self.positions_publisher.publish(self.target_positions)
        self.get_logger().info(
            f"Spawned {response.name} at ({request.x:.2f}, {request.y:.2f})"
        )

    def handle_pose(self, pose):
        if not self.kill_client.service_is_ready():
            return

        for target in self.target_positions.targets:
            delta_x = pose.x - target.position.x
            delta_y = pose.y - target.position.y
            if delta_x * delta_x + delta_y * delta_y > 0.2**2:
                continue
            if target.name in self.pending_kills:
                continue

            request = Kill.Request(name=target.name)
            self.pending_kills.add(target.name)
            future = self.kill_client.call_async(request)
            future.add_done_callback(
                lambda completed_future, name=target.name: self.kill_finished(
                    completed_future, name
                )
            )

    def kill_finished(self, future, name):
        try:
            future.result()
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"Failed to clear target {name}: {error}")
            return
        finally:
            self.pending_kills.discard(name)

        self.target_positions.targets = [
            target for target in self.target_positions.targets if target.name != name
        ]
        self.positions_publisher.publish(self.target_positions)
        self.get_logger().info(f"Cleared target {name}")


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
