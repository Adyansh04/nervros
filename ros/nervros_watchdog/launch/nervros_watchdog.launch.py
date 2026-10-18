"""Starts the watchdog that cancels NervROS's own goals when NervROS goes quiet."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("nervros_watchdog"), "config", "nervros_watchdog.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=params,
                description="The watchdog's parameters; a robot passes its own.",
            ),
            Node(
                package="nervros_watchdog",
                executable="nervros_watchdog",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            ),
        ]
    )
