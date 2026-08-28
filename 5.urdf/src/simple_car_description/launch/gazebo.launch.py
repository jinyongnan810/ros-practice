from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_simple_car_description = FindPackageShare("simple_car_description")
    pkg_ros_gz_sim = FindPackageShare("ros_gz_sim")

    # Set Gazebo resource path to find meshes (model://simple_car_description/...)
    # FindPackageShare points to install/.../share/simple_car_description,
    # so ".." points to install/.../share where simple_car_description directory resides.
    gz_sim_resource_path = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=PathJoinSubstitution([pkg_simple_car_description, ".."]),
    )
    gz_resource_path = AppendEnvironmentVariable(
        name="GZ_RESOURCE_PATH",
        value=PathJoinSubstitution([pkg_simple_car_description, ".."]),
    )

    # Launch arguments
    model_arg = DeclareLaunchArgument(
        "model",
        default_value=PathJoinSubstitution(
            [pkg_simple_car_description, "urdf", "simple_car.urdf.xacro"]
        ),
        description="Absolute or package-relative path to the robot URDF/Xacro file.",
    )

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=PathJoinSubstitution(
            [pkg_simple_car_description, "worlds", "forest.sdf"]
        ),
        description="Gazebo world file to load (e.g. forest.sdf or empty.sdf).",
    )

    x_arg = DeclareLaunchArgument(
        "x",
        default_value="0.0",
        description="Initial X position for spawning the robot.",
    )

    y_arg = DeclareLaunchArgument(
        "y",
        default_value="0.0",
        description="Initial Y position for spawning the robot.",
    )

    z_arg = DeclareLaunchArgument(
        "z",
        default_value="0.1",
        description="Initial Z position for spawning the robot.",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true.",
    )

    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="false",
        description="Whether to start RViz alongside Gazebo.",
    )

    bridge_config_arg = DeclareLaunchArgument(
        "bridge_config",
        default_value=PathJoinSubstitution(
            [pkg_simple_car_description, "config", "gazebo_bridge.yaml"]
        ),
        description="Path to the ROS-Gazebo bridge configuration YAML file.",
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(["xacro ", LaunchConfiguration("model")]),
                    value_type=str,
                ),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
        output="screen",
    )

    # Gazebo Sim (gz_sim)
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": [LaunchConfiguration("world"), " -r"]
        }.items(),
    )

    # Spawn entity into Gazebo
    spawn_entity_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "simple_car",
            "-x",
            LaunchConfiguration("x"),
            "-y",
            LaunchConfiguration("y"),
            "-z",
            LaunchConfiguration("z"),
        ],
        output="screen",
    )

    # ROS-Gz Bridge configured via YAML file
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": LaunchConfiguration("bridge_config"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
        output="screen",
    )

    # Optional RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution([pkg_simple_car_description, "rviz", "display.rviz"]),
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    return LaunchDescription(
        [
            gz_sim_resource_path,
            gz_resource_path,
            model_arg,
            world_arg,
            x_arg,
            y_arg,
            z_arg,
            use_sim_time_arg,
            rviz_arg,
            bridge_config_arg,
            robot_state_publisher_node,
            gazebo_sim,
            spawn_entity_node,
            bridge_node,
            rviz_node,
        ]
    )
