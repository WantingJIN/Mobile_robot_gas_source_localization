from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="dwarf_path_planner",
            executable="path_planner",
            name="path_planner",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "rover_name": "rover_a",
                    "ignore_heading": False,
                    "configuration_file": "simulation.yaml",
                }
            ]
        )
    ])
