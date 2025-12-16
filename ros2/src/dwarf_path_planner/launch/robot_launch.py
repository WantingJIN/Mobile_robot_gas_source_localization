from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    rval = []
    rval.append(Node(
        package="dwarf_path_planner",
        executable="path_planner",
        name="path_planner",
        namespace="nori",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "rover_name": "nori",
                "ignore_heading": False,
                "configuration_file": "real.yaml",
            }]
    ))

    rval.append(Node(
        package='ros2_vicon_bridge',
        executable='vicon_bridge',
        name='vicon',
        output='screen',
        parameters=[
            {"stream_mode": "ClientPull",
             "datastream_hostport": "192.168.22.5:801",
             "tf_ref_frame_id": "/world"},
        ]
    ))

    return LaunchDescription(rval)
