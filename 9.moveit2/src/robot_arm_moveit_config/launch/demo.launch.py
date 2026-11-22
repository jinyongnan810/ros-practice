from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("six_axis_arm", package_name="robot_arm_moveit_config")
        .to_moveit_configs()
    )
    demo_launch = generate_demo_launch(moveit_config)

    launch_commander_arg = DeclareLaunchArgument(
        "launch_commander",
        default_value="true",
        description="Launch the robot_arm_commander_cpp commander action server node",
    )

    commander_node = Node(
        package="robot_arm_commander_cpp",
        executable="commander",
        name="commander",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_commander")),
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([
        launch_commander_arg,
        *demo_launch.entities,
        commander_node,
    ])
