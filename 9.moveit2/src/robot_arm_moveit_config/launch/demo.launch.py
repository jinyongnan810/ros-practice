from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def launch_setup(context):
    moveit_config = (
        MoveItConfigsBuilder("six_axis_arm", package_name="robot_arm_moveit_config")
        .robot_description(
            mappings={
                "use_hardware": LaunchConfiguration("use_hardware").perform(context),
                "dynamixel_port": LaunchConfiguration("dynamixel_port").perform(context),
                "baud_rate": LaunchConfiguration("baud_rate").perform(context),
            }
        )
        .to_moveit_configs()
    )
    demo_launch = generate_demo_launch(moveit_config)

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

    return [
        *demo_launch.entities,
        commander_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "launch_commander",
            default_value="true",
            description="Launch the robot_arm_commander_cpp commander action server node",
        ),
        DeclareLaunchArgument(
            "use_hardware",
            default_value="false",
            description="Use real Dynamixel hardware (arm_hardware/ArmHardwareInterface) instead of mock_components",
        ),
        DeclareLaunchArgument(
            "dynamixel_port",
            default_value="/dev/ttyUSB0",
            description="Serial port for the Dynamixel XL330 bus",
        ),
        DeclareLaunchArgument(
            "baud_rate",
            default_value="57600",
            description="Baud rate for the Dynamixel XL330 bus",
        ),
        OpaqueFunction(function=launch_setup),
    ])
