from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        "model",
        default_value="$(find simple_car_description)/urdf/simple_car.urdf",
        description="Absolute or package-relative path to the URDF file.",
    )

    return LaunchDescription(
        [
            model_arg,
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
            Node(package="rviz2", executable="rviz2", output="screen"),
        ]
    )
