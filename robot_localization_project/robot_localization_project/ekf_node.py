from sqlite3 import Time
import time
from arrow import now
from attr import s
from click import Parameter
from jinja2 import Undefined
from sympy import Quaternion, false
import rclpy.logging
import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Time

from tf_transformations import euler_from_quaternion, quaternion_from_euler

import numpy as np

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry

from robot_localization_project.ekf import RobotEKF
from robot_localization_project.motion_models import velocity_mm, odometry_mm, get_odometry_input
from robot_localization_project.measurement_model import landmarks_mm, z_landmark, residual

class EKF_node(Node):

    def __init__(self):
        super().__init__('ekf_node')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.create_subscription(Odometry, '/ground_truth', self.ground_odom_listener_callback, 10)
        self.create_subscription(Odometry, '/diff_drive_controller/odom', self.diff_odom_listener_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_listener_callback, 10)
        self.publisher_vel = self.create_publisher(Odometry, 'ekf_vel', 10)
        self.publisher_odom = self.create_publisher(Odometry, 'ekf_odom', 10)
        publish_period = 1.0  # seconds
        self.create_timer(publish_period, self.vel_filter_publisher_callback)
        self.create_timer(publish_period, self.odom_filter_publisher_callback)        

        self.start_ground_truth = False
        self.start_cmd_vel = False
        self.start_diff_odom = False
        self.start_ekf_vel = False
        self.start_ekf_odom = False

        self.declare_parameter('ekf_period_s', 1)
        self.declare_parameter('initial_pose', [2.0, 6.0, 0.3])
        self.declare_parameter('initial_covariance', [0.1, 0.1, 0.1])
        self.declare_parameter('landmarks', [5, 12, 10, 5, 15, 15, 10, 14, 6, 6, 12, 9])
        self.declare_parameters('odometry',[('std_rot1', 0.01),('std_transl', 0.1),('std_rot2', 0.01)])
        self.declare_parameters('velocity', [('std_lin_vel', 0.1),('std_ang_vel', 1.0)])
        self.declare_parameters('noise',[('std_rng', 0.3),('std_brg', 1.0)])
        self.declare_parameters('landmark_sim',[('max_range', 8.0),('fov_deg', 45)])

        #initialize required common parameters
        self.mu = np.array([self.get_parameter('initial_pose').value]).T
        self.Sigma = np.diag(self.get_parameter('initial_covariance').value)
        self.Qt = np.diag([self.get_parameter('noise.std_rng').value**2, self.get_parameter('noise.std_brg').value**2])
        self.ekf_period_s = self.get_parameter('ekf_period_s').value
        self.std_range = self.get_parameter('noise.std_rng').value
        self.std_bearing = self.get_parameter('noise.std_brg').value
        self.max_range = self.get_parameter('landmark_sim.max_range').value
        self.fov_deg = self.get_parameter('landmark_sim.fov_deg').value
        self.create_timer(self.ekf_period_s, self.ekf_filter_vel)
        self.create_timer(self.ekf_period_s, self.ekf_filter_odom)
        self.eval_hx, self.eval_Ht = landmarks_mm()

        #extract landmarks:
        self.landmarks = []
        lmarks = self.get_parameter('landmarks').value
        for i in range(int(len(lmarks)/2)):
            self.landmarks.append([lmarks[2*i],lmarks[2*i + 1]])

        #odom model init
        self.Mt_odom = np.diag([self.get_parameter('odometry.std_rot1').value**2, np.deg2rad(self.get_parameter('odometry.std_transl').value)**2, np.deg2rad(self.get_parameter('odometry.std_rot2').value)**2])
        self.eval_gux_odom, self.eval_Gt_odom, self.eval_Vt_odom = odometry_mm()
        self.ekf_odom = RobotEKF(self.mu, self.Sigma, self.Mt_odom, self.Qt, 3, self.eval_gux_odom, self.eval_Gt_odom, self.eval_Vt_odom, self.eval_hx, self.eval_Ht)
        self.odom_pose_prev_empty = True

        #velocity model init
        self.Mt_vel = np.diag([self.get_parameter('velocity.std_lin_vel').value**2, np.deg2rad(self.get_parameter('velocity.std_ang_vel').value)**2])
        self.eval_gux, self.eval_Gt, self.eval_Vt, self.eval_gux_w_zero, self.eval_Gt_w_zero, self.eval_Vt_w_zero = velocity_mm()
        self.ekf_vel = RobotEKF(self.mu, self.Sigma, self.Mt_vel, self.Qt, 3, self.eval_gux, self.eval_Gt, self.eval_Vt, self.eval_hx, self.eval_Ht, self.eval_gux_w_zero, self.eval_Gt_w_zero, self.eval_Vt_w_zero)

        #store data init
        self.ekf_vel_Sigma_prediction_data = []
        self.ekf_vel_Sigma_correction_data = []
        self.ekf_odom_Sigma_prediction_data = []
        self.ekf_odom_Sigma_correction_data = []
        self.odom_data = []
        self.ekf_vel_data = []
        self.ekf_odom_data = []
        self.ground_truth_data = []
        odom_init = Odometry()
        odom_init.pose.pose.position.x = self.mu.flatten()[0]
        odom_init.pose.pose.position.y = self.mu.flatten()[1]
        quat = quaternion_from_euler(0.0,0.0,self.mu.flatten()[2])
        odom_init.pose.pose.orientation.x = quat[0]
        odom_init.pose.pose.orientation.y = quat[1]
        odom_init.pose.pose.orientation.z = quat[2]
        odom_init.pose.pose.orientation.w = quat[3]

        self.ekf_odom_save_sigma_prediction = self.Sigma
        self.ekf_odom_save_sigma_correction = self.Sigma
        self.ekf_vel_save_sigma_prediction = self.Sigma
        self.ekf_vel_save_sigma_correction = self.Sigma

        self.ekf_vel_save = odom_init
        self.ekf_odom_save = odom_init
        self.odom_data_save = Odometry()
        self.ground_truth_save = Odometry()
        self.store_data_period = 0.1
        self.create_timer(self.store_data_period, self.store)

    def store(self):
        if self.start_ekf_vel and self.start_ground_truth and self.start_ekf_odom and self.start_diff_odom and self.start_cmd_vel:
            self.store_data(self.odom_data_save, self.ekf_vel_save, self.ekf_odom_save, self.ground_truth_save, self.ekf_odom_save_sigma_prediction, self.ekf_odom_save_sigma_correction, self.ekf_vel_save_sigma_prediction, self.ekf_vel_save_sigma_correction)

    def cmd_vel_listener_callback(self, msg: Twist):
        self.start_cmd_vel = True
        v = msg.linear.x
        w = msg.angular.z
        self.cmd_vel = np.array([[v, w]]).T

    def diff_odom_listener_callback(self, msg:Odometry):
        self.odom_data_save = msg
        self.start_diff_odom = True
        #odom
        x= msg.pose.pose.position.x
        y= msg.pose.pose.position.y
        quat= [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
        _,_,yaw= euler_from_quaternion(quat)
        self.odom_pose = np.array([[x,y,yaw]]).T
        if self.odom_pose_prev_empty:
            self.odom_pose_prev = self.odom_pose.copy()
            self.odom_pose_prev_empty = False   

        #velocity
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        self.odom_vel = np.array([[v, w]]).T

    def ground_odom_listener_callback(self, msg:Odometry):
        self.ground_truth_save = msg
        self.start_ground_truth = True
        #odom
        x= msg.pose.pose.position.x
        y= msg.pose.pose.position.y
        quat= [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
        _,_,yaw= euler_from_quaternion(quat)
        self.ground_pose = np.array([[x,y,yaw]]).T

        #velocity
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        self.ground_vel = [v,w]    
        
    def vel_filter_publisher_callback(self):        
        msg = Odometry()
        msg.pose.pose.position.x = self.ekf_vel.mu.flatten()[0]
        msg.pose.pose.position.y = self.ekf_vel.mu.flatten()[1]
        quat = quaternion_from_euler(0,0,self.ekf_vel.mu.flatten()[2])
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]

        msg.pose.covariance[0] = self.ekf_vel.Sigma[0][0]
        msg.pose.covariance[1] = self.ekf_vel.Sigma[0][1]
        msg.pose.covariance[5] = self.ekf_vel.Sigma[0][2]
        msg.pose.covariance[6] = self.ekf_vel.Sigma[1][0]
        msg.pose.covariance[7] = self.ekf_vel.Sigma[1][1]
        msg.pose.covariance[11] = self.ekf_vel.Sigma[1][2]
        msg.pose.covariance[30] = self.ekf_vel.Sigma[2][0]
        msg.pose.covariance[31] = self.ekf_vel.Sigma[2][1]
        msg.pose.covariance[35] = self.ekf_vel.Sigma[2][2]
        
        self.ekf_vel_save = msg
        self.start_ekf_vel = True
        self.publisher_vel.publish(msg)
        #self.get_logger().info(f'Publishing: X = {msg.pose.pose.position.x} Y = {msg.pose.pose.position.y}')
        self.get_logger().debug(f'Publishing ekf_vel: X = {msg.pose.pose.position.x} Y = {msg.pose.pose.position.y}, yaw = {self.ekf_vel.mu.flatten()[2]}')

    def odom_filter_publisher_callback(self):      
        msg = Odometry()
        msg.pose.pose.position.x = self.ekf_odom.mu.flatten()[0]
        msg.pose.pose.position.y = self.ekf_odom.mu.flatten()[1]
        quat = quaternion_from_euler(0,0,self.ekf_odom.mu.flatten()[2])
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]

        msg.pose.covariance[0] = self.ekf_odom.Sigma[0][0]
        msg.pose.covariance[1] = self.ekf_odom.Sigma[0][1]
        msg.pose.covariance[5] = self.ekf_odom.Sigma[0][2]
        msg.pose.covariance[6] = self.ekf_odom.Sigma[1][0]
        msg.pose.covariance[7] = self.ekf_odom.Sigma[1][1]
        msg.pose.covariance[11] = self.ekf_odom.Sigma[1][2]
        msg.pose.covariance[30] = self.ekf_odom.Sigma[2][0]
        msg.pose.covariance[31] = self.ekf_odom.Sigma[2][1]
        msg.pose.covariance[35] = self.ekf_odom.Sigma[2][2]
        
        self.publisher_odom.publish(msg)
        self.ekf_odom_save = msg
        self.start_ekf_odom = True
        #self.get_logger().info(f'Publishing: X = {msg.pose.pose.position.x} Y = {msg.pose.pose.position.y}')
        self.get_logger().debug(f'Publishing ekf_odom: X = {msg.pose.pose.position.x} Y = {msg.pose.pose.position.y}, yaw = {self.ekf_odom.mu.flatten()[2]}')


    def ekf_filter_odom(self):
        if self.start_cmd_vel and self.start_diff_odom and self.start_ground_truth:
            u = get_odometry_input(self.odom_pose, self.odom_pose_prev).flatten().astype('float64') 
            self.odom_pose_prev = self.odom_pose.copy()
            self.ekf_odom.predict(u = u.flatten())
            self.ekf_odom_save_sigma_prediction = self.ekf_odom.Sigma.copy()
            for lmark in self.landmarks:
                z = z_landmark(self.ground_pose, lmark, self.max_range, self.fov_deg, self.std_range, self.std_bearing, self.eval_hx)
                if z is not None:
                    self.ekf_odom.update(z, lmark, residual=residual)
                    self.get_logger().info('updateddddddddddddddddddddddddddd')
            self.ekf_odom_save_sigma_correction = self.ekf_odom.Sigma.copy()

    def ekf_filter_vel(self):
        if self.start_cmd_vel and self.start_diff_odom and self.start_ground_truth:
            self.ekf_vel.predict(u = self.odom_vel.flatten(), g_extra_args=(self.ekf_period_s))
            #self.ekf_vel.predict(u = self.cmd_vel.flatten(), g_extra_args=(self.ekf_period_s))
            self.ekf_vel_save_sigma_prediction = self.ekf_vel.Sigma.copy()
            for lmark in self.landmarks:
                z = z_landmark(self.ground_pose, lmark, self.max_range, self.fov_deg, self.std_range, self.std_bearing, self.eval_hx)
                if z is not None:
                    self.ekf_vel.update(z, lmark, residual=residual)
            self.ekf_vel_save_sigma_correction = self.ekf_vel.Sigma.copy()

    def store_data(self, odom: Odometry, ekf_vel: Odometry, ekf_odom: Odometry, ground_truth: Odometry, ekf_odom_Sigma_prediction, ekf_odom_Sigma_correction, ekf_vel_Sigma_prediction, ekf_vel_Sigma_correction):
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
        self.ekf_vel_Sigma_prediction_data.append(ekf_vel_Sigma_prediction)
        self.ekf_vel_Sigma_correction_data.append(ekf_vel_Sigma_correction)
        self.ekf_odom_Sigma_prediction_data.append(ekf_odom_Sigma_prediction)
        self.ekf_odom_Sigma_correction_data.append(ekf_odom_Sigma_correction)
        self.odom_data.append([odom_x, odom_y, odom_yaw])
        self.ekf_vel_data.append([ekf_vel_x, ekf_vel_y, ekf_vel_yaw])
        self.ekf_odom_data.append([ekf_odom_x, ekf_odom_y, ekf_odom_yaw])
        self.ground_truth_data.append([ground_x, ground_y, ground_yaw])       
    
    def save_data(self):
        np.save("/workspaces/vscode_ros2_workspace/src/robot_localization_project/saved_data/odom.npy", self.odom_data, allow_pickle=True)
        np.save("/workspaces/vscode_ros2_workspace/src/robot_localization_project/saved_data/ekf_vel.npy", self.ekf_vel_data, allow_pickle=True)
        np.save("/workspaces/vscode_ros2_workspace/src/robot_localization_project/saved_data/ekf_odom.npy", self.ekf_odom_data, allow_pickle=True)
        np.save("/workspaces/vscode_ros2_workspace/src/robot_localization_project/saved_data/ground_truth.npy", self.ground_truth_data, allow_pickle=True)   
        np.save("/workspaces/vscode_ros2_workspace/src/robot_localization_project/saved_data/ekf_vel_Sigma_prediction.npy", self.ekf_vel_Sigma_prediction_data, allow_pickle=True)
        np.save("/workspaces/vscode_ros2_workspace/src/robot_localization_project/saved_data/ekf_vel_Sigma_correction.npy", self.ekf_vel_Sigma_correction_data, allow_pickle=True)
        np.save("/workspaces/vscode_ros2_workspace/src/robot_localization_project/saved_data/ekf_odom_Sigma_prediction.npy", self.ekf_odom_Sigma_prediction_data, allow_pickle=True)
        np.save("/workspaces/vscode_ros2_workspace/src/robot_localization_project/saved_data/ekf_odom_Sigma_correction.npy", self.ekf_odom_Sigma_correction_data, allow_pickle=True)        

def main(args=None):

    rclpy.init(args=args)
    ekf_node = EKF_node()
    sim_start = time.time()
    try:
        rclpy.spin(ekf_node)
    except KeyboardInterrupt:
        sim_time = time.time() - sim_start
        np.save("/workspaces/vscode_ros2_workspace/src/robot_localization_project/saved_data/sim_prop.npy",[sim_time, ekf_node.ekf_period_s, ekf_node.store_data_period], allow_pickle=True)
        ekf_node.save_data()
        ekf_node.get_logger().info('saved successfully')
    finally:
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
