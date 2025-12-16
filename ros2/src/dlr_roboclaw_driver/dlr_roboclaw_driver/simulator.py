import rclpy
from rclpy.node import Node
import time
import math

from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from std_msgs.msg import Float64

from dlr_roboclaw_driver.virtual_rover import Virtual_Rover

from scipy.spatial.transform import Rotation


class RoboClawSim(Node):
    def __init__(self):
        super().__init__('roboclaw_driver')

        self.declare_parameter("wheel_diameter",rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("wheel_distance",rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("simulation_delta_t",rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("position_topic",rclpy.Parameter.Type.STRING)
        self.declare_parameter("initial_x",rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("initial_y",rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("initial_yaw",rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("name",rclpy.Parameter.Type.STRING)

        self.wheel_dia = self.get_parameter("wheel_diameter").value
        self.wheel_dist = self.get_parameter("wheel_distance").value
        self.dt = self.get_parameter("simulation_delta_t").value
        self.position_topic = self.get_parameter("position_topic").value
        self.name = self.get_parameter("name").value

        self.init_pose = {}
        self.init_pose['x'] = self.get_parameter("initial_x").value
        self.init_pose['y'] = self.get_parameter("initial_y").value
        self.init_pose['yaw'] = self.get_parameter("initial_yaw").value

        self.parameter = {}
        self.parameter["acc_motor"] = 200.
        self.parameter["diameter_wheel"] = self.wheel_dia / 1000.
        self.parameter["dist_tracks"] = self.wheel_dist / 1000.

        self.rover = Virtual_Rover(self.init_pose, self.parameter)

        self.subscription_vel = self.create_subscription(Twist, 'cmd_vel', self.vel_callback, 10)
        self.subscription_vel  # prevent unused variable warning

        self.publisher_pose = self.create_publisher(TransformStamped, self.position_topic, 10)
        self.publisher_pose2 = self.create_publisher(PoseStamped, "pose", 10)

        timer_period = self.dt
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.rover.update(self.dt)
        msg = TransformStamped()
        [msg.transform.translation.x, msg.transform.translation.y, yaw] = self.rover.get_pose()

        # Create a rotation object from Euler angles specifying axes of rotation
        rot = Rotation.from_euler('xyz', [0., 0., yaw], degrees=False)

        [msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z,
         msg.transform.rotation.w] = rot.as_quat()

        msg2 = PoseStamped()
        msg2.header.frame_id = "map"
        msg2.pose.position.x = msg.transform.translation.x
        msg2.pose.position.y = msg.transform.translation.y
        msg2.pose.orientation = msg.transform.rotation

        self.publisher_pose.publish(msg)
        self.publisher_pose2.publish(msg2)

    # print("x:",msg.transform.translation.x,"   y:",msg.transform.translation.y)

    def calculate_motor_speed(self, velocity):
        speed = (60 * velocity * 1. / ((self.wheel_dia / 1000.) * 3.1415))  # rpm
        if speed > 5000: speed = 5000
        if speed < -5000: speed = -5000
        return speed

    def vel_callback(self, msg):
        lin_vel = msg.linear.x
        ang_vel = msg.angular.z

        U = (self.wheel_dist / 1000.) * 3.1415
        vel_M1 = lin_vel + ang_vel * U / 2. / 3.1415
        vel_M2 = lin_vel - ang_vel * U / 2. / 3.1415

        speed_M1 = self.calculate_motor_speed(vel_M1)
        speed_M2 = self.calculate_motor_speed(vel_M2)

        self.rover.set_nominal_speed(speed_M2, speed_M1)


def main(args=None):
    rclpy.init(args=args)
    roboclaw_sim = RoboClawSim()
    rclpy.spin(roboclaw_sim)
    roboclaw_sim.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
