import pathlib
import yaml
from launch_ros.actions import Node
from launch import LaunchDescription

namespace = "jaki"
device_id = 0


def generate_launch_description():
    parameters_file_path = str(pathlib.Path(__file__).parents[1])  # get current path and go one level up
    parameters_file_path += '/config/teleop_twist_joy_config.yaml'
    parameters_file_path = parameters_file_path.replace("install", "src")
    parameters_file_path = parameters_file_path.replace("/share", "")

    with open(parameters_file_path, 'r') as stream:
        config = yaml.safe_load(stream)

    print(parameters_file_path)

    rval = []
    rval.append(
        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name="teleop_node",
            namespace=namespace,
            output="screen",
            emulate_tty=True,
            parameters=[
                config
            ]
        ))

    rval.append(
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            namespace=namespace,
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "device_id": device_id,
                }
            ]
        ))

    return LaunchDescription(rval)
