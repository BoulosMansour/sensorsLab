import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import message_filters
import numpy as np
from tf_transformations import euler_from_quaternion
import numpy as np
import matplotlib.pyplot as plt


class RecorderNode(Node):
    def __init__(self):
        super().__init__("recorder_node")
        self.get_logger().info("Initializing recorder_node")

        # Create the subscribers and synchronize the messages
        self.odom_sub = message_filters.Subscriber(self, Odometry, "/diff_drive_controller/odom")
        self.ekf_vel_sub = message_filters.Subscriber(self, Odometry, "ekf_vel")
        self.ekf_odom_sub = message_filters.Subscriber(self, Odometry, "ekf_odom")
        self.ground_truth_sub = message_filters.Subscriber(self, Odometry, "/ground_truth")

        # Set the queue size according to your needs
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.odom_sub, self.ekf_vel_sub, self.ekf_odom_sub, self.ground_truth_sub], 10, 0.1
        )
        self.ts.registerCallback(self.store_data)

        self.odom_data = []
        self.ekf_vel_data = []
        self.ekf_odom_data = []
        self.ground_truth_data = []

    def store_data(self, odom: Odometry, ekf_vel: Odometry, ekf_odom: Odometry, ground_truth: Odometry):

        self.get_logger().info(f'ODOM DATA: SSSSSSSSSSSSSSSSSS')

        quat_odom = [odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w]
        _,_,odom_yaw = euler_from_quaternion(quat_odom)
        odom_x = odom.pose.pose.position.x
        odom_y = odom.pose.pose.position.y

        quat_ekf_vel = [ekf_vel.pose.pose.orientation.x,ekf_vel.pose.pose.orientation.y,ekf_vel.pose.pose.orientation.z,ekf_vel.pose.pose.orientation.w]
        _,_,ekf_vel_yaw = euler_from_quaternion(quat_ekf_vel)
        ekf_vel_x = ekf_vel.pose.pose.position.x
        ekf_vel_y = ekf_vel.pose.pose.position.y

        quat_ekf_odom = [ekf_odom.pose.pose.orientation.x,ekf_odom.pose.pose.orientation.y,ekf_odom.pose.pose.orientation.z,ekf_odom.pose.pose.orientation.w]
        _,_,ekf_odom_yaw = euler_from_quaternion(quat_ekf_odom)
        ekf_odom_x = ekf_odom.pose.pose.position.x
        ekf_odom_y = ekf_odom.pose.pose.position.y

        quat_ground_truth = [ground_truth.pose.pose.orientation.x,ground_truth.pose.pose.orientation.y,ground_truth.pose.pose.orientation.z,ground_truth.pose.pose.orientation.w]
        _,_,ground_yaw = euler_from_quaternion(quat_ground_truth)
        ground_x = ground_truth.pose.pose.position.x
        ground_y = ground_truth.pose.pose.position.y

        # Store data for later analysis
        self.odom_data.append([odom_x, odom_y, odom_yaw])
        self.ekf_vel_data.append([ekf_vel_x, ekf_vel_y, ekf_vel_yaw])
        self.ekf_odom_data.append([ekf_odom_x, ekf_odom_y, ekf_odom_yaw])
        self.ground_truth_data.append([ground_x, ground_y, ground_yaw])
        self.get_logger().info(f'ODOM DATA: {self.odom_data}')

    # Save data to compute metrics later 
    def save_data(self):
        np.save("/workspaces/vscode_ros2_workspace/src/robot_localization_project/saved_data/odom.npy", self.odom_data, allow_pickle=True)
        np.save("/workspaces/vscode_ros2_workspace/src/robot_localization_project/saved_data/ekf_vel.npy", self.ekf_vel_data, allow_pickle=True)
        np.save("/workspaces/vscode_ros2_workspace/src/robot_localization_project/saved_data/ekf_odom.npy", self.ekf_odom_data, allow_pickle=True)
        np.save("/workspaces/vscode_ros2_workspace/src/robot_localization_project/saved_data/ground_truth.npy", self.ground_truth_data, allow_pickle=True)        


def main(args=None):
    rclpy.init(args=args)
    node = RecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_data()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()