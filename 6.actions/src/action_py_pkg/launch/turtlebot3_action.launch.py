import os
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Verify turtlebot3_gazebo package is installed
    try:
        pkg_turtlebot3_gazebo = get_package_share_directory("turtlebot3_gazebo")
    except PackageNotFoundError:
        raise RuntimeError(
            "\n" + "=" * 70 + "\n"
            "ERROR: 'turtlebot3_gazebo' package is not installed on your system!\n\n"
            "Please install the TurtleBot3 simulation packages with:\n"
            "  sudo apt update\n"
            "  sudo apt install -y ros-jazzy-turtlebot3-gazebo ros-jazzy-turtlebot3-simulations\n"
            + "=" * 70 + "\n"
        )

    # Declare arguments
    model_arg = DeclareLaunchArgument(
        "model",
        default_value="burger",
        description="TurtleBot3 model type: burger, waffle, or waffle_pi",
    )

    launch_client_arg = DeclareLaunchArgument(
        "launch_client",
        default_value="false",
        description="Whether to launch the action client alongside Gazebo and server",
    )

    target_x_arg = DeclareLaunchArgument(
        "target_x",
        default_value="2.0",
        description="Target X coordinate for the action client",
    )

    target_y_arg = DeclareLaunchArgument(
        "target_y",
        default_value="2.0",
        description="Target Y coordinate for the action client",
    )

    linear_velocity_arg = DeclareLaunchArgument(
        "linear_velocity",
        default_value="0.22",
        description="Target linear velocity (default 0.22 m/s for TurtleBot3 burger)",
    )

    cancel_after_sec_arg = DeclareLaunchArgument(
        "cancel_after_sec",
        default_value="0.0",
        description="Auto-cancellation delay in seconds (0.0 to disable)",
    )

    # Set TURTLEBOT3_MODEL environment variable
    set_model_env = SetEnvironmentVariable(
        name="TURTLEBOT3_MODEL",
        value=LaunchConfiguration("model"),
    )

    # Include Gazebo simulation from turtlebot3_gazebo
    gazebo_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, "launch", "empty_world.launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # Action server node configured for simulation (/clock, /odom, /cmd_vel)
    action_server_node = Node(
        package="action_py_pkg",
        executable="turtle_action_server",
        name="turtle_action_server",
        parameters=[{"use_sim_time": True, "use_stamped_vel": True}],
        output="screen",
    )

    # Action client node (conditionally launched)
    action_client_node = Node(
        package="action_py_pkg",
        executable="turtle_action_client",
        name="turtle_action_client",
        parameters=[
            {
                "target_x": LaunchConfiguration("target_x"),
                "target_y": LaunchConfiguration("target_y"),
                "linear_velocity": LaunchConfiguration("linear_velocity"),
                "cancel_after_sec": LaunchConfiguration("cancel_after_sec"),
                "use_sim_time": True,
            }
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_client")),
    )

    return LaunchDescription(
        [
            model_arg,
            launch_client_arg,
            target_x_arg,
            target_y_arg,
            linear_velocity_arg,
            cancel_after_sec_arg,
            set_model_env,
            gazebo_simulation,
            action_server_node,
            action_client_node,
        ]
    )
