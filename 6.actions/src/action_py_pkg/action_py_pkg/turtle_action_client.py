#!/usr/bin/env python3
import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node

from custom_interfaces.action import MoveToGoal


class TurtleActionClient(Node):
    """Action client that sends navigation goals to turtle_action_server and monitors progress."""

    def __init__(self):
        super().__init__("turtle_action_client")

        # Declare parameters so users can customize goals via CLI or launch
        self.declare_parameter("target_x", 8.5)
        self.declare_parameter("target_y", 8.5)
        self.declare_parameter("linear_velocity", 2.0)
        self.declare_parameter("cancel_after_sec", 0.0)

        # Create action client
        self.client = ActionClient(self, MoveToGoal, "move_to_goal")
        self.goal_handle = None
        self.cancel_timer = None

        self.get_logger().info("Turtle Action Client has been started!")

        # Short timer to trigger sending goal after node initialization
        self.init_timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        """One-shot trigger to send the initial goal."""
        self.init_timer.cancel()
        target_x = float(self.get_parameter("target_x").value)
        target_y = float(self.get_parameter("target_y").value)
        linear_velocity = float(self.get_parameter("linear_velocity").value)
        self.send_goal(target_x, target_y, linear_velocity)

    def send_goal(self, target_x: float, target_y: float, linear_velocity: float = 2.0):
        """Send a MoveToGoal goal request to the action server."""
        self.get_logger().info("Waiting for action server '/move_to_goal'...")
        while not self.client.wait_for_server(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error("Interrupted while waiting for server.")
                return
            self.get_logger().info("Action server not available, waiting again...")

        goal_msg = MoveToGoal.Goal()
        goal_msg.target_x = target_x
        goal_msg.target_y = target_y
        goal_msg.linear_velocity = linear_velocity

        self.get_logger().info(
            f"Sending goal: target=({target_x:.2f}, {target_y:.2f}), velocity={linear_velocity:.2f}"
        )

        send_goal_future = self.client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Callback invoked when the server accepts or rejects the goal."""
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn("Goal was REJECTED by server!")
            return

        self.get_logger().info("Goal ACCEPTED by server! Navigating to target...")

        # If cancel_after_sec parameter is specified, schedule cancellation
        cancel_after_sec = float(self.get_parameter("cancel_after_sec").value)
        if cancel_after_sec > 0.0:
            self.get_logger().info(f"Will request cancellation in {cancel_after_sec:.1f}s...")
            self.cancel_timer = self.create_timer(cancel_after_sec, self.request_cancel)

        get_result_future = self.goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Callback invoked whenever the server publishes periodic feedback."""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"Feedback: remaining distance={feedback.current_distance:.2f}m, "
            f"current_pos=({feedback.current_x:.2f}, {feedback.current_y:.2f})"
        )

    def result_callback(self, future):
        """Callback invoked when the action execution terminates."""
        result_wrapper = future.result()
        status = result_wrapper.status
        result = result_wrapper.result

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f"Goal SUCCEEDED! Total distance: {result.total_distance:.2f}m, "
                f"Elapsed time: {result.elapsed_time:.2f}s"
            )
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(
                f"Goal was CANCELED! Partial distance: {result.total_distance:.2f}m, "
                f"Elapsed time: {result.elapsed_time:.2f}s"
            )
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn(
                f"Goal was ABORTED (preempted by another goal)! Partial distance: {result.total_distance:.2f}m"
            )
        else:
            self.get_logger().error(f"Goal terminated with status code: {status}")

    def request_cancel(self):
        """Send a cancellation request for the active goal."""
        self.cancel_timer.cancel()
        if self.goal_handle is not None:
            self.get_logger().info("Sending cancel request to action server...")
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_response_callback)

    def cancel_response_callback(self, future):
        """Callback invoked when the server responds to the cancellation request."""
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info("Cancellation request was ACCEPTED by server.")
        else:
            self.get_logger().warn("Cancellation request was REJECTED by server.")


def main(args=None):
    rclpy.init(args=args)
    node = TurtleActionClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
