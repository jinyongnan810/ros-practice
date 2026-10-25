from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory("robot_arm_description"))
    robot_description = xacro.process_file(
        str(package_share / "urdf" / "six_axis_arm.urdf.xacro")
    ).toxml()
    gui = LaunchConfiguration("gui")

    return LaunchDescription([
        DeclareLaunchArgument(
            "gui", default_value="true", choices=["true", "false"],
            description="Show sliders for the six arm joints.",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "robot_description": ParameterValue(robot_description, value_type=str),
            }],
            output="screen",
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(gui),
            output="screen",
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            condition=UnlessCondition(gui),
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(package_share / "rviz" / "display.rviz")],
            output="screen",
        ),
    ])
