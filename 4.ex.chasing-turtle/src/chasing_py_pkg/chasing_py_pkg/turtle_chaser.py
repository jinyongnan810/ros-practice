#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from turtlesim.msg import Pose

from chasing_interfaces.msg import TargetPositions


class TurtleChaser(Node):
    """Drive turtle1 toward one selected target at a time."""

    def __init__(self):
        super().__init__("turtle_chaser")
        # Cache the latest target registry and remember the active target by name.
        self.target_positions = TargetPositions()
        self.selected_target_name = None
        # Inputs provide target state and turtle pose; output controls turtle motion.
        self.target_subscription = self.create_subscription(
            TargetPositions,
            "/spawned_target_positions",
            self.update_targets,
            10,
        )
        self.pose_subscription = self.create_subscription(
            Pose, "/turtle1/pose", self.handle_pose, 10
        )
        self.velocity_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

    def update_targets(self, message):
        """Replace the local registry with the spawner's latest snapshot."""
        self.target_positions = message

    def handle_pose(self, pose):
        """Select a target if needed and publish the next velocity command."""
        command = Twist()
        if not self.target_positions.targets:
            self.selected_target_name = None
            self.velocity_publisher.publish(command)
            return

        selected_target = next(
            (
                target
                for target in self.target_positions.targets
                if target.name == self.selected_target_name
            ),
            None,
        )
        # Keep chasing the selected name; choose again only after it disappears.
        if selected_target is None:
            selected_target = min(
                self.target_positions.targets,
                key=lambda target: (target.position.x - pose.x) ** 2
                + (target.position.y - pose.y) ** 2,
            )
            self.selected_target_name = selected_target.name
            self.get_logger().info(f"Selected target {selected_target.name}")

        delta_x = selected_target.position.x - pose.x
        delta_y = selected_target.position.y - pose.y
        distance = math.hypot(delta_x, delta_y)
        desired_heading = math.atan2(delta_y, delta_x)
        # Normalize the turn error to [-pi, pi] so the turtle takes the short turn.
        heading_error = math.atan2(
            math.sin(desired_heading - pose.theta),
            math.cos(desired_heading - pose.theta),
        )

        command.angular.z = 4.0 * heading_error
        # Rotate first when badly misaligned, then advance with capped proportional speed.
        if abs(heading_error) < 0.5:
            command.linear.x = min(2.0, 1.5 * distance)
        self.velocity_publisher.publish(command)


def main(args=None):
    """Run the node until ROS shuts down."""
    rclpy.init(args=args)
    node = TurtleChaser()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
