from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_client_arg = DeclareLaunchArgument(
        "launch_client",
        default_value="false",
        description="Whether to launch the action client node alongside turtlesim and server",
    )

    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="turtlesim",
    )

    server_node = Node(
        package="action_cpp_pkg",
        executable="turtle_action_server",
        name="turtle_action_server",
        output="screen",
    )

    client_node = Node(
        package="action_cpp_pkg",
        executable="turtle_action_client",
        name="turtle_action_client",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_client")),
    )

    return LaunchDescription(
        [
            launch_client_arg,
            turtlesim_node,
            server_node,
            client_node,
        ]
    )
