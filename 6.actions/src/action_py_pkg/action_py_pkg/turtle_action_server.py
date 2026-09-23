#!/usr/bin/env python3
import math
import threading

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from turtlesim.msg import Pose

from custom_interfaces.action import MoveToGoal
import os


class TurtleActionServer(Node):
    """Action server that drives turtle1/TurtleBot3 to a requested (target_x, target_y) coordinate."""

    def __init__(self):
        super().__init__("turtle_action_server")

        self.cb_group = ReentrantCallbackGroup()

        # Coordinate boundary configuration (relaxed by default for open Gazebo worlds)
        self.declare_parameter("enforce_canvas_bounds", False)
        self.declare_parameter("min_x", -20.0)
        self.declare_parameter("max_x", 20.0)
        self.declare_parameter("min_y", -20.0)
        self.declare_parameter("max_y", 20.0)

        # Velocity message type: ROS 2 Jazzy uses TwistStamped on /cmd_vel for Gazebo Harmonic
        default_stamped = os.environ.get("ROS_DISTRO", "jazzy") != "humble"
        self.declare_parameter("use_stamped_vel", default_stamped)
        self.use_stamped_vel = self.get_parameter("use_stamped_vel").get_parameter_value().bool_value

        # Goal tracking for preemption
        self.active_goal_handle = None
        self.goal_lock = threading.Lock()

        # Turtle / Robot state
        self.current_pose = None

        # Velocity publishers:
        # - /turtle1/cmd_vel: always geometry_msgs/msg/Twist for Turtlesim
        # - /cmd_vel: TwistStamped for ROS 2 Jazzy Gazebo Harmonic bridge, or Twist for Humble
        self.cmd_vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        if self.use_stamped_vel:
            self.tb3_cmd_vel_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        else:
            self.tb3_cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Pose / Odometry subscribers (dual support: Turtlesim Pose & Gazebo Odometry)
        self.pose_sub = self.create_subscription(
            Pose,
            "/turtle1/pose",
            self.handle_pose,
            10,
            callback_group=self.cb_group,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.handle_odom,
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
        """Update the latest known pose from Turtlesim."""
        self.current_pose = msg

    def handle_odom(self, msg: Odometry):
        """Update the latest known pose from Gazebo / TurtleBot3 Odometry."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)
        self.current_pose = Pose(x=float(x), y=float(y), theta=float(theta))

    def goal_callback(self, goal_request: MoveToGoal.Goal):
        """Validate the requested target coordinates within acceptable boundaries."""
        self.get_logger().info(
            f"Received goal request: target=({goal_request.target_x:.2f}, {goal_request.target_y:.2f}), "
            f"velocity={goal_request.linear_velocity:.2f}"
        )

        enforce_bounds = self.get_parameter("enforce_canvas_bounds").get_parameter_value().bool_value
        min_x = 0.0 if enforce_bounds else self.get_parameter("min_x").get_parameter_value().double_value
        max_x = 11.0 if enforce_bounds else self.get_parameter("max_x").get_parameter_value().double_value
        min_y = 0.0 if enforce_bounds else self.get_parameter("min_y").get_parameter_value().double_value
        max_y = 11.0 if enforce_bounds else self.get_parameter("max_y").get_parameter_value().double_value

        if min_x <= goal_request.target_x <= max_x and min_y <= goal_request.target_y <= max_y:
            self.get_logger().info("Target within boundary. Accepting goal.")
            return GoalResponse.ACCEPT
        else:
            self.get_logger().warn(
                f"Rejecting goal: target ({goal_request.target_x:.2f}, {goal_request.target_y:.2f}) "
                f"is outside boundary [{min_x:.1f}, {max_x:.1f}]!"
            )
            return GoalResponse.REJECT

    def cancel_callback(self, goal_handle):
        """Accept goal cancellation requests from clients."""
        self.get_logger().info("Received goal cancellation request.")
        return CancelResponse.ACCEPT

    def stop_turtle(self):
        """Halt robot motion by publishing zero velocity to both Turtlesim and Gazebo."""
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        if self.use_stamped_vel:
            stop_stamped = TwistStamped()
            stop_stamped.header.stamp = self.get_clock().now().to_msg()
            stop_stamped.header.frame_id = "base_footprint"
            self.tb3_cmd_vel_pub.publish(stop_stamped)
        else:
            self.tb3_cmd_vel_pub.publish(stop_cmd)

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
            # Clamp angular velocity within physical limits (+-2.0 rad/s)
            cmd.angular.z = max(-2.0, min(2.0, 2.5 * heading_error))
            # Advance when reasonably aligned with target
            if abs(heading_error) < 0.5:
                cmd.linear.x = min(linear_velocity, 1.5 * distance)

            # Publish to Turtlesim (/turtle1/cmd_vel)
            self.cmd_vel_pub.publish(cmd)

            # Publish to TurtleBot3 Gazebo (/cmd_vel)
            if self.use_stamped_vel:
                stamped_cmd = TwistStamped()
                stamped_cmd.header.stamp = self.get_clock().now().to_msg()
                stamped_cmd.header.frame_id = "base_footprint"
                stamped_cmd.twist = cmd
                self.tb3_cmd_vel_pub.publish(stamped_cmd)
            else:
                self.tb3_cmd_vel_pub.publish(cmd)

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
