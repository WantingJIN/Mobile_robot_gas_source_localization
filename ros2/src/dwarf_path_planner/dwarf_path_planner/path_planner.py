import math
import numpy as np
import os
import rclpy
import time
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist, TransformStamped, PoseArray, Pose2D, Pose
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_msgs.msg import Float64


class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')

        self.pose = Pose2D()

        self.next_poses = []  # list of Pose2D
        self.current_index = 0  # which of the poses in the list the robot drives to
        self.nominal_pose = Pose2D()
        self.nominal_pose.x = float('inf')
        self.controller_phase = 0

        self.declare_parameter("rover_name",rclpy.Parameter.Type.STRING)
        self.name = self.get_parameter("rover_name").value

        self.declare_parameter("ignore_heading",rclpy.Parameter.Type.BOOL)
        self.ignore_heading = self.get_parameter("ignore_heading").value

        self.declare_parameter('absolute_heading', False)
        self.absolute_heading = self.get_parameter("absolute_heading").value

        self.declare_parameter("configuration_file",rclpy.Parameter.Type.STRING)
        configuration_file = self.get_parameter("configuration_file").value

        config_file = os.path.dirname(os.path.abspath(__file__)) + "/../config/" + configuration_file
        config_file.replace("install", "src")

        self.config = yaml.load(open(config_file), Loader=yaml.FullLoader)

        self.vicon_topic_name = "/vicon/" + self.name + "/" + self.name
        self.vicon_median_length = self.config['vicon_median_length']

        # init vicon buffer
        self.vicon_buffer = np.zeros((3, self.vicon_median_length), dtype=float)
        self.vicon_buffer_idx = 0

        self.max_lin_vel = self.config['max_lin_vel']
        self.max_agl_vel = self.config['max_agl_vel']

        self.lin_accuracy = self.config['lin_accuracy']
        self.agl_accuracy = self.config['agl_accuracy']

        self.control_turning_P = self.config['control_turning_P']
        self.control_turning_offset = self.config['control_turning_offset']

        self.control_driving_P = self.config['control_driving_P']
        self.control_driving_offset = self.config['control_driving_offset']

        self.control_tracking_P = self.config['control_tracking_P']
        self.control_tracking_I = self.config['control_tracking_I']

        self.subscription_vicon = self.create_subscription(TransformStamped,
                                                           self.vicon_topic_name,
                                                           self.vicon_callback,
                                                           10)

        self.subscription_pose_array = self.create_subscription(PoseArray, 'pose_ctrl_array', self.pose_array_callback, 10)
        self.subscription_pose_ctrl = self.create_subscription(Pose2D, 'pose_ctrl', self.pose_callback, 10)

        self.publisher_cmd = self.create_publisher(Twist, "cmd_vel", 10)

        self.publisher_current_goal = self.create_publisher(Pose2D, "current_goal", 10)

        timer_period = 1. / self.config["control_rate"]  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.cmd_rot = 0.
        self.cmd_lin = 0.

        self.watchdog_counter = 0
        self.emergency_stop = False
        self.watchdog = self.create_timer(1, self.watchdog_callback)

    def watchdog_callback(self):
        self.watchdog_counter += 1
        if self.watchdog_counter >= 2:
            self.emergency_stop = True
        else:
            self.emergency_stop = False

    def timer_callback(self):
        self.control_loop()
        self.pub_cmd()

    def pub_cmd(self):
        msg = Twist()
        msg.linear.x = float(self.cmd_lin)
        msg.angular.z = float(self.cmd_rot)
        self.publisher_cmd.publish(msg)

    def vicon_callback(self, data):
        self.watchdog_counter = 0
        if self.vicon_buffer_idx >= self.vicon_median_length:
            self.vicon_buffer_idx = 0
        self.vicon_buffer[0, self.vicon_buffer_idx] = data.transform.translation.x
        self.vicon_buffer[1, self.vicon_buffer_idx] = data.transform.translation.y
        rot = Rotation.from_quat([data.transform.rotation.x, data.transform.rotation.y, data.transform.rotation.z,
                                  data.transform.rotation.w])
        self.vicon_buffer[2, self.vicon_buffer_idx] = rot.as_euler('zxy', degrees=False)[0]

        self.vicon_buffer_idx += 1

    # filter vicon data
    def vicon_filter(self):
        temp = np.median(self.vicon_buffer, axis=1)
        self.pose.x = temp[0]
        self.pose.y = temp[1]
        self.pose.theta = temp[2]

    def pose_callback(self, input):  # one pose sent to path planner
        if not ((input.x == self.nominal_pose.x) and (input.y == self.nominal_pose.y) and (
                input.theta == self.nominal_pose.theta)):  # check if input is new
            self.controller_phase = 1
            self.nominal_pose = input
            self.get_logger().info(self.name.upper() + ": set nominal pose to: x=" + str(self.nominal_pose.x) + "; y=" + str(
                self.nominal_pose.y) + "; theta=" + str(self.nominal_pose.theta))

    def pose_array_callback(self, input):  # multiple poses sent to path planner
        if not self.next_poses:
            if len(self.next_poses) == len(input.poses):  # check if path is new
                input_new = False
                for i in range(len(self.next_poses)):
                    if self.next_poses[i].x != input.poses[i].position.x:
                        input_new = True
                    if self.next_poses[i].y != input.poses[i].position.y:
                        input_new = True
                if not input_new:
                    return

            self.next_poses = []
            for pose in input.poses:
                pose_2d = Pose2D()
                pose_2d.x = pose.position.x
                pose_2d.y = pose.position.y
                pose_2d.theta = Rotation.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z,
                                            pose.orientation.w]).as_euler(seq='zxy', degrees=True)[0]
                self.next_poses.append(pose_2d)

            self.controller_phase = 1
            self.nominal_pose = self.next_poses[0]
            self.get_logger().info(self.name.upper() + ": set nominal pose to: x=" + str(self.nominal_pose.x) + "; y=" + str(
                self.nominal_pose.y) + "; theta=" + str(self.nominal_pose.theta))

    def control_loop(self):
        if self.emergency_stop:
            self.cmd_rot = 0
            self.cmd_lin = 0
            if self.watchdog_counter>=5:
                self.next_poses = []  # list of Pose2D
                self.nominal_pose = Pose2D()
                self.nominal_pose.x = float('inf')
                self.controller_phase = 0
            return

        self.vicon_filter()

        self.publish_current_goal()

        # distance to target
        delta_y = self.nominal_pose.y - self.pose.y
        delta_x = self.nominal_pose.x - self.pose.x
        dist_m = math.sqrt(delta_y * delta_y + delta_x * delta_x)
        # nominal heading
        nominal_theta = math.atan2(delta_y, delta_x)
        if self.controller_phase == 3:
            nominal_theta = self.nominal_pose.theta

        # dif in heading
        dif_theta_forward = self.pose.theta - nominal_theta
        # remap dif -180 to 180
        while dif_theta_forward > math.pi:
            dif_theta_forward -= 2. * math.pi
        while dif_theta_forward < -math.pi:
            dif_theta_forward += 2. * math.pi

        dif_theta_backward = self.pose.theta + math.pi - nominal_theta
        # remap dif -180 to 180
        while dif_theta_backward > math.pi:
            dif_theta_backward -= 2. * math.pi
        while dif_theta_backward < -math.pi:
            dif_theta_backward += 2. * math.pi

        if (abs(dif_theta_forward) < abs(dif_theta_backward)) or ((self.controller_phase == 3) and self.absolute_heading):
            dif_theta = dif_theta_forward
            backward = 0
        else:
            dif_theta = dif_theta_backward
            backward = 1


        # ------------------------------------------------------------------------------
        # is this a turning point?
        if (self.controller_phase == 1) or (self.controller_phase == 3):
            # calc motor input (limited)
            ctrl_cmd = math.copysign(self.control_turning_offset, dif_theta) + dif_theta * self.control_turning_P
            ctrl_cmd = min(ctrl_cmd, self.max_agl_vel)
            ctrl_cmd = max(ctrl_cmd, -self.max_agl_vel)

            self.cmd_rot = -ctrl_cmd

            if (abs(dif_theta) < self.agl_accuracy) or ((self.controller_phase == 1) and (
                    abs(dif_theta) < self.agl_accuracy)):  # nominal heading reached or in phase 1 too close to target
                self.cmd_rot = 0
                self.cmd_lin = 0
                if self.controller_phase == 1:
                    self.get_logger().info(self.name.upper() + ": direction to target reached!")
                    self.controller_phase = 2  # direction reached next phase driving (2)
                    self.i_error = 0.  # reset integral for driving controller
                    return
                if self.controller_phase == 3:
                    self.get_logger().info(self.name.upper() + ": nominal heading reached!")
                    self.controller_phase = 0  # target heading reached next phase idle (0)
                    return

        # ------------------------------------------------------------------------------
        # driving phase
        if self.controller_phase == 2:
            # if rover drives with current heading he will miss target by
            dif_miss = math.sin(dif_theta) * dist_m

            # integrate error miss
            self.i_error += dif_miss

            if backward == 0:
                # calc motor forward power (limited)
                ctrl_offset = dist_m * self.control_driving_P + self.control_driving_offset
                ctrl_offset = min(ctrl_offset, self.max_lin_vel)

            else:
                ctrl_offset = -dist_m * self.control_driving_P - self.control_driving_offset
                ctrl_offset = max(ctrl_offset, -self.max_lin_vel)

            # calc motor delta power to track target
            self.cmd_rot = -self.control_tracking_P * dif_miss - self.control_tracking_I * self.i_error

            # if(abs(dif_theta)<0.5*math.pi): target in front
            self.cmd_lin = ctrl_offset

            if (dist_m < self.lin_accuracy) or (abs(abs(dif_theta) - 0.5 * math.pi) < self.agl_accuracy):
                # target reached (dist<threshold), or target lies left/right
                self.cmd_rot = 0
                self.cmd_lin = 0

                if self.ignore_heading:
                    self.controller_phase = 0
                else:
                    if not self.next_poses or self.next_poses and self.next_poses.index(self.nominal_pose) == len(self.next_poses) - 1:
                        self.controller_phase = 3  # control heading
                    else:
                        self.controller_phase = 0

                if not (dist_m < self.lin_accuracy):
                    # distance to target still to high, start state machine at beginning
                    self.controller_phase = 1
                self.get_logger().info(self.name.upper() + ": target reached! remaining distance=" + str(dist_m))

                if self.next_poses:
                    self.select_next_nominal_pose()
                return

    def select_next_nominal_pose(self):
        self.current_index += 1  # switch to next point

        if self.current_index < len(self.next_poses):
            self.controller_phase = 1
            self.nominal_pose = self.next_poses[self.current_index]
            self.get_logger().info(self.name.upper() + ": set nominal pose to: x=" + str(self.nominal_pose.x) + "; y=" + str(
                self.nominal_pose.y) + "; theta=" + str(self.nominal_pose.theta))
        else:
            self.get_logger().info(self.name.upper() + ": final pose reached.")
            self.next_poses = []
            self.current_index = 0  # reset to start of list

    def publish_current_goal(self):
        msg = Pose2D()
        msg.x = self.nominal_pose.x
        msg.y = self.nominal_pose.y
        msg.theta = self.nominal_pose.theta
        self.publisher_current_goal.publish(msg)

    def cleanup(self):
        self.timer.cancel()
        self.cmd_rot = 0
        self.cmd_lin = 0
        self.pub_cmd()


def main(args=None):
    rclpy.init(args=args)
    planner = PathPlanner()
    try:
        rclpy.spin(planner)
    except KeyboardInterrupt as e:
        planner.get_logger().info(str(e))
        planner.cleanup()
        rclpy.spin_once(planner)
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
