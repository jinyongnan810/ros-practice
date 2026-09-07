#!/usr/bin/env python3
import math
import threading

import rclpy
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from turtlesim.msg import Pose

from custom_interfaces.action import MoveToGoal


class TurtleActionServer(Node):
    """Action server that drives turtle1 to a requested (target_x, target_y) coordinate."""

    def __init__(self):
        super().__init__("turtle_action_server")

        self.cb_group = ReentrantCallbackGroup()

        # Goal tracking for preemption
        self.active_goal_handle = None
        self.goal_lock = threading.Lock()

        # Turtle state
        self.current_pose = None

        # Velocity publisher
        self.cmd_vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        # Pose subscriber
        self.pose_sub = self.create_subscription(
            Pose,
            "/turtle1/pose",
            self.handle_pose,
            10,
            callback_group=self.cb_group,
        )

        # Action server
        self.action_server = ActionServer(
            self,
            MoveToGoal,
            "move_to_goal",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )

        self.get_logger().info("Turtle Action Server has been started! Action: /move_to_goal")

    def handle_pose(self, msg: Pose):
        """Update the latest known pose of turtle1."""
        self.current_pose = msg

    def goal_callback(self, goal_request: MoveToGoal.Goal):
        """Validate the requested target coordinates within the Turtlesim boundary [0, 11]."""
        self.get_logger().info(
            f"Received goal request: target=({goal_request.target_x:.2f}, {goal_request.target_y:.2f}), "
            f"velocity={goal_request.linear_velocity:.2f}"
        )

        if 0.0 <= goal_request.target_x <= 11.0 and 0.0 <= goal_request.target_y <= 11.0:
            self.get_logger().info("Target within boundary. Accepting goal.")
            return GoalResponse.ACCEPT
        else:
            self.get_logger().warn(
                f"Rejecting goal: target ({goal_request.target_x:.2f}, {goal_request.target_y:.2f}) "
                "is outside Turtlesim canvas boundary [0.0, 11.0]!"
            )
            return GoalResponse.REJECT

    def cancel_callback(self, goal_handle):
        """Accept goal cancellation requests from clients."""
        self.get_logger().info("Received goal cancellation request.")
        return CancelResponse.ACCEPT

    def stop_turtle(self):
        """Halt turtle motion by publishing zero velocity."""
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

    def execute_callback(self, goal_handle):
        """Execute the motion toward the target, publishing feedback and handling cancellation."""
        target_x = goal_handle.request.target_x
        target_y = goal_handle.request.target_y
        linear_velocity = (
            goal_handle.request.linear_velocity
            if goal_handle.request.linear_velocity > 0.0
            else 2.0
        )

        with self.goal_lock:
            if self.active_goal_handle is not None and self.active_goal_handle.is_active:
                self.get_logger().info("Preempting previous active goal for new target...")
            self.active_goal_handle = goal_handle

        feedback_msg = MoveToGoal.Feedback()
        result = MoveToGoal.Result()

        loop_rate = self.create_rate(10)
        start_time = self.get_clock().now()
        total_distance = 0.0
        prev_pose = self.current_pose

        self.get_logger().info(f"Executing goal: moving to ({target_x:.2f}, {target_y:.2f})...")

        while rclpy.ok():
            # 1. Check if preempted by a newer goal request
            with self.goal_lock:
                if goal_handle != self.active_goal_handle:
                    self.get_logger().info(
                        f"Goal to ({target_x:.2f}, {target_y:.2f}) was preempted by a new goal."
                    )
                    goal_handle.abort()
                    result.success = False
                    result.total_distance = total_distance
                    result.elapsed_time = (self.get_clock().now() - start_time).nanoseconds / 1e9
                    return result

            # 2. Check if cancellation was requested
            if goal_handle.is_cancel_requested:
                self.stop_turtle()
                goal_handle.canceled()
                self.get_logger().info("Goal canceled by client.")
                with self.goal_lock:
                    if self.active_goal_handle == goal_handle:
                        self.active_goal_handle = None
                result.success = False
                result.total_distance = total_distance
                result.elapsed_time = (self.get_clock().now() - start_time).nanoseconds / 1e9
                return result

            # Wait until initial pose is available
            if self.current_pose is None:
                loop_rate.sleep()
                continue

            # 3. Accumulate traveled distance
            if prev_pose is not None:
                total_distance += math.hypot(
                    self.current_pose.x - prev_pose.x,
                    self.current_pose.y - prev_pose.y,
                )
            prev_pose = self.current_pose

            # 4. Calculate distance and angle to target
            delta_x = target_x - self.current_pose.x
            delta_y = target_y - self.current_pose.y
            distance = math.hypot(delta_x, delta_y)

            # 5. Check if goal is reached within tolerance
            if distance < 0.1:
                self.stop_turtle()
                goal_handle.succeed()
                with self.goal_lock:
                    if self.active_goal_handle == goal_handle:
                        self.active_goal_handle = None
                elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
                self.get_logger().info(
                    f"Goal reached! Target: ({target_x:.2f}, {target_y:.2f}) | "
                    f"Distance: {total_distance:.2f}m | Time: {elapsed:.2f}s"
                )
                result.success = True
                result.total_distance = total_distance
                result.elapsed_time = elapsed
                return result

            # 5. Proportional steering and speed control
            desired_heading = math.atan2(delta_y, delta_x)
            heading_error = math.atan2(
                math.sin(desired_heading - self.current_pose.theta),
                math.cos(desired_heading - self.current_pose.theta),
            )

            cmd = Twist()
            cmd.angular.z = 4.0 * heading_error
            # Advance only when reasonably aligned with target
            if abs(heading_error) < 0.5:
                cmd.linear.x = min(linear_velocity, 1.5 * distance)

            self.cmd_vel_pub.publish(cmd)

            # 6. Publish feedback
            feedback_msg.current_distance = distance
            feedback_msg.current_x = self.current_pose.x
            feedback_msg.current_y = self.current_pose.y
            goal_handle.publish_feedback(feedback_msg)

            loop_rate.sleep()

        result.success = False
        return result


def main(args=None):
    rclpy.init(args=args)
    node = TurtleActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_turtle()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
