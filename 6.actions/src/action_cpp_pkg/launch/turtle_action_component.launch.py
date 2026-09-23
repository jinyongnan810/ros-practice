from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    launch_client_arg = DeclareLaunchArgument(
        "launch_client",
        default_value="false",
        description="Whether to load the action client component alongside turtlesim and server",
    )

    target_x_arg = DeclareLaunchArgument(
        "target_x",
        default_value="8.5",
        description="Target X coordinate for the action client",
    )

    target_y_arg = DeclareLaunchArgument(
        "target_y",
        default_value="8.5",
        description="Target Y coordinate for the action client",
    )

    linear_velocity_arg = DeclareLaunchArgument(
        "linear_velocity",
        default_value="2.0",
        description="Target linear velocity for the turtle",
    )

    cancel_after_sec_arg = DeclareLaunchArgument(
        "cancel_after_sec",
        default_value="0.0",
        description="Auto-cancellation delay in seconds (0.0 to disable)",
    )

    container_name = "turtle_action_container"

    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="turtlesim",
    )

    # Component container running the action server component
    container = ComposableNodeContainer(
        name=container_name,
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="action_cpp_pkg",
                plugin="action_cpp_pkg::TurtleActionServerNode",
                name="turtle_action_server",
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )

    # Conditionally load the action client component into the container
    load_client_component = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[
            ComposableNode(
                package="action_cpp_pkg",
                plugin="action_cpp_pkg::TurtleActionClientNode",
                name="turtle_action_client",
                parameters=[
                    {
                        "target_x": LaunchConfiguration("target_x"),
                        "target_y": LaunchConfiguration("target_y"),
                        "linear_velocity": LaunchConfiguration("linear_velocity"),
                        "cancel_after_sec": LaunchConfiguration("cancel_after_sec"),
                    }
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        condition=IfCondition(LaunchConfiguration("launch_client")),
    )

    return LaunchDescription(
        [
            launch_client_arg,
            target_x_arg,
            target_y_arg,
            linear_velocity_arg,
            cancel_after_sec_arg,
            turtlesim_node,
            container,
            load_client_component,
        ]
    )
