from launch import LaunchDescription
from launch_ros.actions import Node
import socket

name_dict = {"192.168.22.101": "bofur",
             "192.168.22.102": "gloin",
             "192.168.22.103": "nori",
             "192.168.22.104": "fili",
             "192.168.22.105": "kili",
             "192.168.22.106": "dwalin",
             "192.168.22.107": "uni",
             "192.168.22.108": "toki",
             "192.168.22.109": "sindri",
             "192.168.22.110": "jaki",
             "192.168.22.111": "heri",
             "192.168.22.112": "eitri",
             "192.168.22.113": "loni"}
my_ip = socket.gethostbyname(socket.getfqdn())
my_name = name_dict[my_ip]


def generate_launch_description():
    rval = []
    rval.append(
        Node(
            package="dlr_roboclaw_driver",
            executable="roboclaw",
            name="roboclaw_driver",
            namespace=my_name,
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "serial_port": "/dev/ttyAMA0",
                    "baud_rate": 38400,
                    "wheel_diameter": 50.0,  # mm
                    "gear_encoder_ratio": 1000.,  # ticks per rotation
                    "wheel_distance": 160.0,  # mm
                }
            ]
        ))

    rval.append(Node(
            package="trisonica_anemometer",
            executable="anemometer_node",
            name="anemometer",
            namespace=my_name,
            prefix=['stdbuf -o L'],
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "serial_port": "/dev/ttyAMA1",
                    "baud_rate": 38400,
                }
            ]
        ))

    return LaunchDescription(rval)
