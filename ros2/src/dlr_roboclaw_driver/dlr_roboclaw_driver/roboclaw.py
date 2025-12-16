import rclpy
from rclpy.node import Node
import time
import os
import yaml

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from dlr_roboclaw_driver.roboclaw_interface import Roboclaw


class RoboClawDriver(Node):
    def __init__(self):
        super().__init__('roboclaw_driver')

        self.declare_parameter("serial_port",rclpy.Parameter.Type.STRING)
        self.declare_parameter("baud_rate",rclpy.Parameter.Type.INTEGER)
        self.declare_parameter("wheel_diameter",rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("gear_encoder_ratio",rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("wheel_distance",rclpy.Parameter.Type.DOUBLE)

        self.port = self.get_parameter("serial_port").value
        self.baud_rate = int(self.get_parameter("baud_rate").value)
        self.wheel_dia = self.get_parameter("wheel_diameter").value
        self.gear_ratio = self.get_parameter("gear_encoder_ratio").value
        self.wheel_dist = self.get_parameter("wheel_distance").value

        self.name = self.get_namespace()
        config_file = os.path.dirname(os.path.abspath(__file__)) + "/../config/" + self.name + "_motor_config.yaml"
        config_file.replace("install", "src")
        config = yaml.load(open(config_file), Loader=yaml.FullLoader)

        self.direction_motor_1 = config["direction_motor_1"]
        self.direction_motor_2 = config["direction_motor_2"]
        self.motor_1_left = config["motor_1_left"]

        self.rc_address = 0x80
        self.rc = Roboclaw(self.port, self.baud_rate)
        while not self.rc.Open():
            self.get_logger().info('Error on serial interface, I try again ...')
            time.sleep(1)

        self.speed_M2 = 0.
        self.speed_M1 = 0.

        # self.subscription_M1 = self.create_subscription(Float64,'speed_motor_r',self.motor1_callback,10)
        # self.subscription_M1  # prevent unused variable warning

        # self.subscription_M2 = self.create_subscription(Float64,'speed_motor_l',self.motor2_callback,10)
        # self.subscription_M2  # prevent unused variable warning

        self.subscription_vel = self.create_subscription(Twist, 'cmd_vel', self.vel_callback, 10)
        self.subscription_vel  # prevent unused variable warning

        self.publisher_bat = self.create_publisher(Float64, 'battery', 10)

        timer_period = 1.  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # watchdog to avoid robot driving after Ctrl+C
        self.watchdog_timer = self.create_timer(0.5, self.watchdog_callback)
        self.watchdog_counter = 0

    def __del__(self):
        # stop motor
        self.rc.SpeedM1(self.rc_address, 0)
        self.rc.SpeedM2(self.rc_address, 0)

    def timer_callback(self):
        msg = Float64()
        msg.data = float(self.rc.ReadMainBatteryVoltage(self.rc_address)[1]) / 10.
        self.publisher_bat.publish(msg)

    def watchdog_callback(self):
        self.watchdog_counter += 1
        if self.watchdog_counter > 2:
            self.rc.DutyM1M2(self.rc_address, 0, 0)

    def calculate_motor_speed(self, velocity):
        speed = int(velocity * self.gear_ratio / ((self.wheel_dia / 1000.) * 3.1415))
        if speed > 5000: speed = 5000
        if speed < -5000: speed = -5000
        return speed

    """
    def motor1_callback(self, msg):
        #self.get_logger().info('set left motor speed "%f"' % msg.data)
        speed=self.calculate_motor_speed(msg.data)
        self.rc.SpeedM1(self.rc_address, speed)

    def motor2_callback(self, msg):
        #self.get_logger().info('set right motor speed "%f"' % msg.data)
        speed = self.calculate_motor_speed(msg.data)
        self.rc.SpeedM2(self.rc_address, speed)
    """

    def vel_callback(self, msg):
        self.watchdog_counter = 0  # reset watchdog

        lin_vel = msg.linear.x
        ang_vel = msg.angular.z

        U = (self.wheel_dist / 1000.) * 3.1415
        vel_M1 = lin_vel - ang_vel * U / 2. / 3.1415
        vel_M2 = lin_vel + ang_vel * U / 2. / 3.1415

        # print(self.direction_motor_1, self.direction_motor_2)
        tmp1 = int(self.direction_motor_1 * self.calculate_motor_speed(vel_M1))
        tmp2 = int(self.direction_motor_2 * self.calculate_motor_speed(vel_M2))

        if self.motor_1_left:
            self.speed_M2 = tmp2
            self.speed_M1 = tmp1
        else:
            self.speed_M1 = tmp2
            self.speed_M2 = tmp1

        self.rc.SpeedM1M2(self.rc_address, self.speed_M1, self.speed_M2)

        if self.speed_M1 == 0 and self.speed_M2 == 0:
            self.rc.DutyM1M2(self.rc_address, 0, 0)

        time.sleep(0.01)


def main(args=None):
    rclpy.init(args=args)
    roboclaw_driver = RoboClawDriver()
    rclpy.spin(roboclaw_driver)
    roboclaw_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
