from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    auto_manage_arg = DeclareLaunchArgument(
        "auto_manage",
        default_value="false",
        description="Whether to run the lifecycle manager to automate state transitions across all nodes",
    )

    frequency_arg = DeclareLaunchArgument(
        "frequency",
        default_value="1.0",
        description="Publish frequency in Hz for the sensor stations",
    )

    # Station 1 (Alpha)
    station_1_node = LifecycleNode(
        package="lifecycle_cpp_pkg",
        executable="sensor_station",
        name="sensor_station_1",
        namespace="",
        output="screen",
        parameters=[
            {
                "sensor_name": "station_alpha",
                "topic_name": "sensor_data",
                "publish_frequency": LaunchConfiguration("frequency"),
            }
        ],
    )

    # Station 2 (Beta)
    station_2_node = LifecycleNode(
        package="lifecycle_cpp_pkg",
        executable="sensor_station",
        name="sensor_station_2",
        namespace="",
        output="screen",
        parameters=[
            {
                "sensor_name": "station_beta",
                "topic_name": "sensor_data",
                "publish_frequency": LaunchConfiguration("frequency"),
            }
        ],
    )

    # Observer Monitor
    monitor_node = Node(
        package="lifecycle_cpp_pkg",
        executable="sensor_monitor",
        name="sensor_monitor",
        output="screen",
        parameters=[{"topic_name": "sensor_data"}],
    )

    # Automated Lifecycle Manager (optional)
    manager_node = Node(
        package="lifecycle_cpp_pkg",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        output="screen",
        parameters=[{"managed_nodes": ["sensor_station_1", "sensor_station_2"]}],
        condition=IfCondition(LaunchConfiguration("auto_manage")),
    )

    return LaunchDescription(
        [
            auto_manage_arg,
            frequency_arg,
            station_1_node,
            station_2_node,
            monitor_node,
            manager_node,
        ]
    )
