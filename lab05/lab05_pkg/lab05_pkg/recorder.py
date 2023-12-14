import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import message_filters
import numpy as np
from tf_transformations import euler_from_quaternion


class RecorderNode(Node):
    def _init_(self):
        super()._init_("recorder_node")
        self.get_logger().info("Initializing recorder_node")

        # Create the subscribers and synchronize the messages
        self.odom_sub = message_filters.Subscriber(self, Odometry, "/diff_drive_controller/odom",self.odom_listener,10)
        self.filter_sub = message_filters.Subscriber(self,store_data Odometry, "/odometry/filtered",self.filter_listener,10)
        self.ground_truth_sub = message_filters.Subscriber(self, Odometry, "/ground_truth",self.ground_listener,10)

        # Set the queue size according to your needs
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.odom_sub, self.filter_sub, self.ground_truth_sub], 10, 0.01
        )
        self.ts.registerCallback(self.store_data)

        self.odom_data = {'x': [], 'y': [], 'yaw': []}
        self.filter_data = {'x': [], 'y': [], 'yaw': []}
        self.ground_truth_data = {'x': [], 'y': [], 'yaw': []}
        self.odom_distance = 0


    def odom_listener(self,msg:Odometry):
        quat = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
        _,_,self.odom.yaw = euler_from_quaternion(quat)
        self.odom.x = msg.pose.pose.position.x
        self.odom.y = msg.pose.pose.position.y
        
    def filter_listener(self,msg:Odometry):
        quat = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
        _,_,self.filter.yaw = euler_from_quaternion(quat)
        self.filter.x = msg.pose.pose.position.x
        self.filter.y = msg.pose.pose.position.y
        
    def ground_listener(self,msg:Odometry):
        quat = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
        _,_,self.ground.yaw = euler_from_quaternion(quat)
        self.ground.x = msg.pose.pose.position.x
        self.ground.y = msg.pose.pose.position.y



        # Additional variables for metrics calculation
        self.total_distance = 0.0
        self.last_ground_truth_position = None

    def store_data(self):

        # Store data for later analysis
        self.odom_data['x'].append(self.odom.x)
        self.odom_data['y'].append(self.odom.y)
        self.odom_data['yaw'].append(self.odom.yaw)

        self.filter_data['x'].append(self.filter.x)
        self.filter_data['y'].append(self.filter.y)
        self.filter_data['yaw'].append(self.filter.yaw)

        self.ground_truth_data['x'].append(self.ground.x)
        self.ground_truth_data['y'].append(self.ground.y)
        self.ground_truth_data['yaw'].append(self.ground.yaw)

        # Update total traveled distance
        if len(self.odom_data)>1:
            self.odom_distance += np.sqrt((self.odom_data[x][len(self.odom_data)-1]-self.odom_data[x][len(self.odom_data)-2])**2 +
                                     (self.odom_data[y][len(self.odom_data)-1]-self.odom_data[y][len(self.odom_data)-2])**2)

        if len(self.odom_data)>1:
            self.ground_truth_distance += np.sqrt((self.ground_truth_data[x][len(self.ground_truth_data)-1]-self.ground_truth_data[x][len(self.ground_truth_data)-2])**2 +
                                     (self.ground_truth_data[y][len(self.ground_truth_data)-1]-self.ground_truth_data[y][len(self.ground_truth_data)-2])**2)


        # Insert your additional metrics calculations here if needed

    def save_data(self):
        np.save("odom.npy", self.odom_data)
        np.save("filter.npy", self.filter_data)
        np.save("ground_truth.npy", self.ground_truth_data)

        # Compute and print the requested metrics
        self.compute_and_print_metrics()

    def compute_and_print_metrics(self):
        # Calculate and print the specified metrics
        # You need to implement the calculation of RMSE, Maximum Absolute Error, etc.
        # Here's an example using NumPy for RMSE calculation:

        odom_array = np.array([self.odom_data['x'], self.odom_data['y']])
        filter_array = np.array([self.filter_data['x'], self.filter_data['y']])
        ground_truth_array = np.array([self.ground_truth_data['x'], self.ground_truth_data['y']])

        rmse = np.sqrt(np.mean((filter_array - ground_truth_array)**2))

        # Print the metrics or save them as needed
        self.get_logger().info(f"RMSE: {rmse}")

    def compute_metrics(self):
        # Load data from .npy files
        odom_data = np.load("odom.npy")
        filter_data = np.load("filter.npy")
        ground_truth_data = np.load("ground_truth.npy")

        # Calculate metrics for position (x, y)
        position_rmse = np.sqrt(np.mean(np.square(filter_data - ground_truth_data)))
        position_max_abs_error = np.max(np.abs(filter_data - ground_truth_data))
        position_total_cumulative_error = np.sum(np.linalg.norm(filter_data - ground_truth_data, axis=1))
        total_distance = np.sum(np.linalg.norm(ground_truth_data[:-1] - ground_truth_data[1:], axis=1))
        position_error_percentage = (position_total_cumulative_error / total_distance) * 100.0

        # Calculate metrics for orientation (yaw angle)
        # Assuming orientation is represented as quaternion in the messages
        odom_quat = np.array([R.from_quat(q).as_euler('xyz')[2] for q in odom_data[:, 3:7]])
        filter_quat = np.array([R.from_quat(q).as_euler('xyz')[2] for q in filter_data[:, 3:7]])
        gt_quat = np.array([R.from_quat(q).as_euler('xyz')[2] for q in ground_truth_data[:, 3:7]])

        orientation_rmse = np.sqrt(mean_squared_error(filter_quat, gt_quat))
        orientation_max_abs_error = np.max(np.abs(filter_quat - gt_quat))
        orientation_total_cumulative_error = np.sum(np.abs(filter_quat - gt_quat))

def main(args=None):
    rclpy.init(args=args)
    node = RecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_data()
        node.compute_metrics()
        rclpy.try_shutdown()

if _name_ == "_main_":
    main()