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
        self.filter_sub = message_filters.Subscriber(self, Odometry, "/odometry/filtered")
        self.ground_truth_sub = message_filters.Subscriber(self, Odometry, "/ground_truth")

        # Set the queue size according to your needs
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.odom_sub, self.filter_sub, self.ground_truth_sub], 10, 0.01
        )
        self.ts.registerCallback(self.store_data)

        self.odom_data = []
        self.filter_data = []
        self.ground_truth_data = []
        self.odom_distance = 0
        self.ground_truth_distance = 0

        # Additional variables for metrics calculation
        self.total_distance = 0.0
        self.last_ground_truth_position = None

    def store_data(self, odom: Odometry, filter: Odometry, ground_truth: Odometry):

        quat_odom = [odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w]
        _,_,odom_yaw = euler_from_quaternion(quat_odom)
        odom_x = odom.pose.pose.position.x
        odom_y = odom.pose.pose.position.y

        quat_filter = [filter.pose.pose.orientation.x,filter.pose.pose.orientation.y,filter.pose.pose.orientation.z,filter.pose.pose.orientation.w]
        _,_,filter_yaw = euler_from_quaternion(quat_filter)
        filter_x = filter.pose.pose.position.x
        filter_y = filter.pose.pose.position.y

        quat_ground_truth = [ground_truth.pose.pose.orientation.x,ground_truth.pose.pose.orientation.y,ground_truth.pose.pose.orientation.z,ground_truth.pose.pose.orientation.w]
        _,_,ground_yaw = euler_from_quaternion(quat_ground_truth)
        ground_x = ground_truth.pose.pose.position.x
        ground_y = ground_truth.pose.pose.position.y

        # Store data for later analysis
        self.odom_data.append([odom_x, odom_y, odom_yaw])
        self.filter_data.append([filter_x, filter_y, filter_yaw])
        self.ground_truth_data.append([ground_x, ground_y, ground_yaw])
        self.get_logger().info(f'ODOM DATA: {self.odom_data}')

    # Save data to compute metrics later 
    def save_data(self):
        np.save("/workspaces/vscode_ros2_workspace/src/lab05/lab05_pkg/saved_bag/odom.npy", self.odom_data, allow_pickle=True)
        np.save("/workspaces/vscode_ros2_workspace/src/lab05/lab05_pkg/saved_bag/filter.npy", self.filter_data, allow_pickle=True)
        np.save("/workspaces/vscode_ros2_workspace/src/lab05/lab05_pkg/saved_bag/ground_truth.npy", self.ground_truth_data, allow_pickle=True)        


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