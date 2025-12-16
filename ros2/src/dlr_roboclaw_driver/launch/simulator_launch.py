from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="dlr_roboclaw_driver",
            executable="simulator",
            name="robot_sim",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "wheel_diameter": 50.0,  # mm
                    "wheel_distance": 160.0,  # mm
                    "simulation_delta_t": 0.01,
                    "position_topic": "vicon/robot_a",
                    "initial_x": 0.,
                    "initial_y": 0.,
                    "initial_yaw": 0.,
                    "name": "rover_a",
                }
            ]
        )
    ])
