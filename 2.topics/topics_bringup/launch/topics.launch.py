from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="topic_py_pkg",
                executable="news_station_node",
                name="news_station_py_1",
                parameters=[{"timer_interval": 1.0}],
                output="screen",
            ),
            Node(
                package="topic_py_pkg",
                executable="news_station_node",
                name="news_station_py_2",
                parameters=[{"timer_interval": 2.0}],
                output="screen",
            ),
            Node(
                package="topic_cpp_pkg",
                executable="news_station",
                name="news_station_cpp_1",
                parameters=[{"timer_interval": 3.0}],
                output="screen",
            ),
            Node(
                package="topic_py_pkg",
                executable="radio_node",
                name="radio_py_1",
                output="screen",
            ),
            Node(
                package="topic_py_pkg",
                executable="radio_node",
                name="radio_py_2",
                output="screen",
            ),
            Node(
                package="topic_py_pkg",
                executable="radio_node",
                name="radio_py_3",
                output="screen",
            ),
            Node(
                package="topic_cpp_pkg",
                executable="radio",
                name="radio_cpp_1",
                output="screen",
            ),
            Node(
                package="topic_cpp_pkg",
                executable="radio",
                name="radio_cpp_2",
                output="screen",
            ),
            # Remapping example
            Node(
                package="topic_cpp_pkg",
                executable="news_station",
                name="news_station_cpp_remapped",
                parameters=[{"timer_interval": 0.5}],
                remappings=[("news", "remapped_news")],
                output="screen",
            ),
            Node(
                package="topic_py_pkg",
                executable="radio_node",
                name="radio_py_remapped",
                remappings=[("news", "remapped_news")],
                output="screen",
            ),
        ]
    )
