import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import message_filters
import numpy as np
from tf_transformations import euler_from_quaternion


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

        self.odom_data = {'x': [], 'y': [], 'yaw': []}
        self.filter_data = {'x': [], 'y': [], 'yaw': []}
        self.ground_truth_data = {'x': [], 'y': [], 'yaw': []}
        self.odom_distance = 0
        self.ground_truth_distance = 0


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

    def store_data(self, odom: Odometry, filter: Odometry, ground_truth: Odometry):

        quat_odom = [odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w]
        _,_,self.odom.yaw = euler_from_quaternion(quat_odom)
        self.odom.x = odom.pose.pose.position.x
        self.odom.y = odom.pose.pose.position.y

        quat_filter = [filter.pose.pose.orientation.x,filter.pose.pose.orientation.y,filter.pose.pose.orientation.z,filter.pose.pose.orientation.w]
        _,_,self.filter.yaw = euler_from_quaternion(quat_filter)
        self.filter.x = filter.pose.pose.position.x
        self.filter.y = filter.pose.pose.position.y

        quat_ground_truth = [ground_truth.pose.pose.orientation.x,ground_truth.pose.pose.orientation.y,ground_truth.pose.pose.orientation.z,ground_truth.pose.pose.orientation.w]
        _,_,self.ground.yaw = euler_from_quaternion(quat_ground_truth)
        self.ground.x = ground_truth.pose.pose.position.x
        self.ground.y = ground_truth.pose.pose.position.y

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

        # # Update total traveled distance
        # if len(self.odom_data)>1:
        #     self.odom_distance += np.sqrt((self.odom_data[x][len(self.odom_data)-1]-self.odom_data[x][len(self.odom_data)-2])**2 +
        #                              (self.odom_data[y][len(self.odom_data)-1]-self.odom_data[y][len(self.odom_data)-2])**2)

        # if len(self.odom_data)>1:
        #     self.ground_truth_distance += np.sqrt((self.ground_truth_data[x][len(self.ground_truth_data)-1]-self.ground_truth_data[x][len(self.ground_truth_data)-2])**2 +
        #                              (self.ground_truth_data[y][len(self.ground_truth_data)-1]-self.ground_truth_data[y][len(self.ground_truth_data)-2])**2)


        # Insert your additional metrics calculations here if needed

    def save_data(self):
        np.save("odom.npy", self.odom_data)
        np.save("filter.npy", self.filter_data)
        np.save("ground_truth.npy", self.ground_truth_data)



        # Compute and print the requested metrics
        self.compute_and_print_metrics()

   # def compute_and_print_metrics(self):
        # Calculate and print the specified metrics
        # You need to implement the calculation of RMSE, Maximum Absolute Error, etc.
        # Here's an example using NumPy for RMSE calculation:

        # odom_array = np.array([self.odom_data['x'], self.odom_data['y']])
        # filter_array = np.array([self.filter_data['x'], self.filter_data['y']])
        # ground_truth_array = np.array([self.ground_truth_data['x'], self.ground_truth_data['y']])

        # rmse = np.sqrt(np.mean((filter_array - ground_truth_array)**2))

        # Print the metrics or save them as needed
        ##self.get_logger().info(f"RMSE: {rmse}")

    def compute_metrics(self):
        # Load data from .npy files
        odom_data = np.load("odom.npy")
        filter_data = np.load("filter.npy")
        ground_truth_data = np.load("ground_truth.npy")

        #filter data
        # Calculate metrics for position (x, y)
        pos_error_f_x = filter_data[0] - ground_truth_data[0]
        pos_error_f_y = filter_data[1] - ground_truth_data[1]
        ang_error_f = filter_data[2] - ground_truth_data[2]

        pos_error_d_x = odom_data[0] - ground_truth_data[0]
        pos_error_d_y = odom_data[1] - ground_truth_data[1]
        ang_error_d = odom_data[2] - ground_truth_data[2]


        self.error_f = np.array[pos_error_f_x, pos_error_f_y, ang_error_f]
        self.error_d = np.array[pos_error_d_x, pos_error_d_y, ang_error_d]

        position_rmse_f_x = np.sqrt(np.mean(np.square(filter_data[0] - ground_truth_data[0])))
        position_rmse_f_y = np.sqrt(np.mean(np.square(filter_data[1] - ground_truth_data[1])))
        position_max_abs_error_f_x = np.max(np.abs(filter_data[0] - ground_truth_data[0]))
        position_max_abs_error_f_y = np.max(np.abs(filter_data[1] - ground_truth_data[1]))
        position_total_cumulative_error_f_x = np.sum(np.linalg.norm(filter_data[0] - ground_truth_data[0]))
        position_total_cumulative_error_f_y = np.sum(np.linalg.norm(filter_data[1] - ground_truth_data[1]))
        total_distance = np.sum( \
            np.sqrt( \
                np.square(ground_truth_data[0][:-1]) - np.square(ground_truth_data[0][1:])+ \
                np.square(ground_truth_data[1][:-1]) - np.square(ground_truth_data[1][1:]) \
                ) \
            )
        position_error_percentage_f_x = ((filter_data[0][len(filter_data[0])-1] - ground_truth_data[0][len(ground_truth_data[0])-1]) / total_distance) * 100.0
        position_error_percentage_f_y = ((filter_data[1][len(filter_data[1])-1] - ground_truth_data[1][len(ground_truth_data[1])-1]) / total_distance) * 100.0

        # Calculate metrics for orientation (yaw angle)
        # Assuming orientation is represented as quaternion in the messages
        #odom data
        position_rmse_d_x = np.sqrt(np.mean(np.square(odom_data[0] - ground_truth_data[0])))
        position_rmse_d_y = np.sqrt(np.mean(np.square(odom_data[1] - ground_truth_data[1])))
        position_max_abs_error_d_x = np.max(np.abs(odom_data[0] - ground_truth_data[0]))
        position_max_abs_error_d_y = np.max(np.abs(odom_data[1] - ground_truth_data[1]))
        position_total_cumulative_error_d_x = np.sum(np.linalg.norm(odom_data[0] - ground_truth_data[0]))
        position_total_cumulative_error_d_y = np.sum(np.linalg.norm(odom_data[1] - ground_truth_data[1]))
        position_error_percentage_d_x = ((odom_data[0][len(odom_data[0])-1] - ground_truth_data[0][len(ground_truth_data[0])-1]) / total_distance) * 100.0
        position_error_percentage_d_y = ((odom_data[1][len(odom_data[1])-1] - ground_truth_data[1][len(ground_truth_data[1])-1]) / total_distance) * 100.0



#        odom_quat = np.array([R.from_quat(q).as_euler('xyz')[2] for q in odom_data[:, 3:7]])
#        filter_quat = np.array([R.from_quat(q).as_euler('xyz')[2] for q in filter_data[:, 3:7]])
#        gt_quat = np.array([R.from_quat(q).as_euler('xyz')[2] for q in ground_truth_data[:, 3:7]])

        orientation_rmse_f = np.sqrt(np.mean(np.square(filter_data[2] - ground_truth_data[2])))
        orientation_max_abs_error_f = np.max(np.abs(filter_data[2] - ground_truth_data[2]))
        orientation_total_cumulative_error_f = np.sum(np.abs(filter_data[2] - ground_truth_data[2]))

        orientation_rmse_d = np.sqrt(np.mean(np.square(odom_data[2] - ground_truth_data[2])))
        orientation_max_abs_error_d = np.max(np.abs(odom_data[2] - ground_truth_data[2]))
        orientation_total_cumulative_error_d = np.sum(np.abs(odom_data[2] - ground_truth_data[2]))

        self.get_logger().info(f'rmse_F_x: {position_rmse_f_x}, rmse_F_y: {position_rmse_f_y}') 
        self.get_logger().info(f'max_error_F_x: {position_max_abs_error_f_x}, max_error_F_y: {position_max_abs_error_f_y}') 
        self.get_logger().info(f'cum_F_x: {position_total_cumulative_error_f_x}, cum_F_y: {position_total_cumulative_error_f_y}')
        self.get_logger().info(f'final_error_F_x: {position_error_percentage_f_x}, final_error_F_y: {position_error_percentage_f_y}')
    
        self.get_logger().info(f'rmse_D_x: {position_rmse_d_x}, rmse_D_y: {position_rmse_d_y}') 
        self.get_logger().info(f'max_error_D_x: {position_max_abs_error_d_x}, max_error_D_y: {position_max_abs_error_d_y}')
        self.get_logger().info(f'cum_D_x: {position_total_cumulative_error_d_x}, cum_D_y: {position_total_cumulative_error_d_y}')
        self.get_logger().info(f'final_error_D_x: {position_error_percentage_d_x}, final_error_D_y: {position_error_percentage_d_y}')

        self.get_logger().info(f'rmse_f_yaw: {orientation_rmse_f}, max_Error_f_yaw: {orientation_max_abs_error_f}, cum_f_yaw: {orientation_total_cumulative_error_f}')
        self.get_logger().info(f'rmse_d_yaw: {orientation_rmse_d}, max_Error_d_yaw: {orientation_max_abs_error_d}, cum_d_yaw: {orientation_total_cumulative_error_d}')

    def save_metrics(self):
        np.save('error_f.npy', self.error_f)
        np.save('error_d.npy', self.error_d)
        


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
        node.save_metrics()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()