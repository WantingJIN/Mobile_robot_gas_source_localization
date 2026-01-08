import rclpy
from rclpy.node import Node
import pandas as pd
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import FluidPressure
from ste_interfaces.msg import WindMeasure
from geometry_msgs.msg import PoseArray
import numpy as np

class MockRobotData(Node):
    def __init__(self):
        super().__init__('mock_robot_data_node')
        
        # Load Datasets
        self.pos_df = pd.read_csv('position_history.csv', header=None)
        self.meas_df = pd.read_csv('measurement_history.csv', header=None)
        self.current_idx = 0
        self.max_idx = min(len(self.pos_df), len(self.meas_df))

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/pose', 10)
        self.gas_pub = self.create_publisher(FluidPressure, '/gas_sensor', 10)
        
        # Separate Wind Publishers
        self.wind_pub = self.create_publisher(WindMeasure, '/wind_sensor', 10)
        
        # subscribe for the goal position set by Laptop
        self.create_subscription(PoseArray, '/pose_ctrl_array', self.goal_pose_callback, 10)
        
        self.playback_timer = None
        self.current_iter_data = []
        
        
        # self.timer = self.create_timer(0.1, self.timer_callback)
        # self.get_logger().info("Mock Node Started: Publishing pose, wind and gas measurements.")

        self.current_iter = 0
        self.get_logger().info("Mock Node Initialized: Ready to replay data upon receiving goal positions.")

    def goal_pose_callback(self, msg):
        self.get_logger().info("Received new goal positions, starting data replay.")
        self.current_goal_data = msg.poses
        
        filtered_meas = self.meas_df[self.meas_df[3] == self.current_iter]
        
        if filtered_meas.empty:
            self.get_logger().warn(f"迭代 {self.current_iter} 没有测量数据！")
            return

        self.current_iter_data = filtered_meas
        self.step_idx = 0
        
        # 启动或重置定时器，以 20Hz 回放数据
        if self.playback_timer:
            self.playback_timer.cancel()
        self.playback_timer = self.create_timer(0.1, self.publish_step)
        self.current_iter += 1

    def publish_step(self):
        if self.step_idx >= len(self.current_iter_data):
            self.playback_timer.cancel()
            self.get_logger().info("当前迭代数据回放完毕。")
            return

        # 获取当前行
        row_idx = self.current_iter_data.index[self.step_idx]
        meas_row = self.current_iter_data.iloc[self.step_idx]
        
        # 尝试匹配位置
        try:
            pos_row = self.pos_df.loc[row_idx]
            x, y = pos_row[1], pos_row[2]
        except KeyError:
            x, y = 0.0, 0.0 # 默认值

        # 1. 发布 Odom
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)

        # 2. 发布 Gas (第 2 列 -> meas_row[1])
        gas = FluidPressure()
        gas.fluid_pressure = float(meas_row[1])
        self.gas_pub.publish(gas)

        # 3. 发布 WindMeasure (第 5,6 列 -> meas_row[4], meas_row[5])
        wind = WindMeasure()
        u, v = float(meas_row[4]), float(meas_row[5])
        wind.wind_u = u
        wind.wind_v = v
        wind.wind_speed = np.sqrt(u**2 + v**2)
        wind.wind_angle = np.arctan2(v, u)
        self.wind_pub.publish(wind)

        self.step_idx += 1
        # Extract
        x = float(self.pos_df.iloc[self.current_idx, 1])
        y = float(self.pos_df.iloc[self.current_idx, 2])
        gas_val = float(self.meas_df.iloc[self.current_idx, 1])
        u_val = float(self.meas_df.iloc[self.current_idx, 4])
        v_val = float(self.meas_df.iloc[self.current_idx, 5])

        # Publish Pose
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'robot'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)

        # Publish Gas
        gas = FluidPressure()
        gas.header.stamp = self.get_clock().now().to_msg()
        gas.fluid_pressure = gas_val
        self.gas_pub.publish(gas)

        # Publish Wind U and V separately
        wind = WindMeasure()
        wind.header.stamp = self.get_clock().now().to_msg()
        wind.wind_u = u_val
        wind.wind_v = v_val
        self.wind_pub.publish(wind)

        self.current_idx += 1
        print(f"Published idx: {self.current_idx}, Pose: ({x}, {y}), Gas: {gas_val}, Wind U: {u_val}, Wind V: {v_val}")

def main(args=None):
    rclpy.init(args=args)
    node = MockRobotData()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()