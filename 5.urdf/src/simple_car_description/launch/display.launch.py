from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        "model",
        default_value="$(find simple_car_description)/urdf/simple_car.urdf",
        description="Absolute or package-relative path to the URDF file.",
    )
    rvizconfig_arg = DeclareLaunchArgument(
        "rvizconfig",
        default_value=PathJoinSubstitution(
            [FindPackageShare("simple_car_description"), "rviz", "display.rviz"]
        ),
        description="Absolute path to the RViz configuration file.",
    )

    return LaunchDescription(
        [
            model_arg,
            rvizconfig_arg,
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {
                        "robot_description": ParameterValue(
                            Command(["xacro ", LaunchConfiguration("model")]),
                            value_type=str,
                        )
                    }
                ],
                output="screen",
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", LaunchConfiguration("rvizconfig")],
                output="screen",
            ),
        ]
    )
