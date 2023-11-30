import rclpy
from rclpy.node import Node
import rclpy.logging
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle

import time

from tf_transformations import euler_from_quaternion
from math import sqrt, radians
from lab03_interfaces.action import MoveForward
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class MoveForwardActionServer(Node):
    def __init__(self):
        super().__init__("move_forward_action_server")

        # CALLBACK GROUPS
        # Define a new callback group for the action server.
        # The action server will be assigned to this group, while the subscribers, publishers and
        # timers will be assigned to the default callback group (which is MutuallyExclusive)
        self.action_server_group = ReentrantCallbackGroup()
        self.topic_group = MutuallyExclusiveCallbackGroup()

        # SUBSCRIBERS, PUBLISHERS, TIMERS...
        # Subscribers and publishers can stay in the default group since they are non-blocking, to 
        # do so just declare them as usual.
        #
        # You can also assign them explicitely to a MutuallyExclusiveCallbackGroup if you want to by
        # passing the callback_group argument.
        self.pose_subscriber = self.create_subscription(Odometry, "diff_drive_controller/odom", self.pose_callback, 10, callback_group=self.topic_group)
        self.laser_scan_subscriber = self.create_subscription(LaserScan,"scan", self.scan_callback,10,callback_group=self.topic_group)
        self.cmd_vel_publisher = self.create_publisher(Twist, "cmd_vel", 10, callback_group=self.topic_group)
        
        self.vel = Twist()
        self.condition = False

        publish_period = 0.1 #seconds
        self.timer = self.create_timer(publish_period, self.pub_velocity_callback)

        # The action server must be in a different callback group since it is blocking.
        # The action server callback will sleep at a fixed rate until the goal is completed, failed 
        # or canceled. The action server will use a ReentrantCallbackGroup so that multiple goals 
        # can be handled simultaneously.

        self.action_server = ActionServer(
            self,
            MoveForward,
            "/move_forward",
            self.execute_action_callback,
            callback_group=self.action_server_group,
        )

        # This rate object is used to sleep the control loop at a fixed rate in the action server callback
        control_rate_hz = 20.0
        self.control_rate = self.create_rate(control_rate_hz)

        self.get_logger().info(
            "Move distance action server has been started, in namespace: "
            + self.get_namespace()
            + " ..."
        )

    def pub_velocity_callback(self):
        if self.condition:
            msg=self.vel
            self.cmd_vel_publisher.publish(msg)

    def scan_callback(self, msg: LaserScan):
        self.scan = msg.ranges[0]

    def pose_callback(self, msg: Odometry):
        self.pose = msg.pose.pose.position
        quat = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
        _,_,self.yaw = euler_from_quaternion(quat)
        

    def execute_action_callback(self, goal_handle: ServerGoalHandle):
        '''
        This function is called when a new goal is received by the action server.
        Pay attention to return the correct result object, otherwise the action client will crash.
        
        Returns:
            result {YourAction.Result} -- Result object
        '''
        self.get_logger().info("Executing goal...")
        feedback_msg = MoveForward.Feedback()
        result = MoveForward.Result()

        # Create a condition to break the control loop, it should be updated at each iteration
        goal = goal_handle.request
        self.vel.linear.x = 0.22
        start_time = time.time()
        self.condition=True
        while self.condition:
            # Execute the control loop at a fixed rate
            dist_x = abs(goal.x-self.pose.x)
            dist_y = abs(goal.y-self.pose.y)
            if (dist_x<0.5 and dist_y<0.5
                or goal.step == 1 and (dist_x<0.5 or dist_y<0.5)
                or goal.step == 2 and dist_y<0.5
                or goal.step == 3 and dist_x<0.5):

                self.vel.linear.x = 0.0
                self.condition = False
                self.cmd_vel_publisher.publish(self.vel)
            
            if self.scan < 1:
                self.vel.linear.x=0.0
                self.condition = False
                self.cmd_vel_publisher.publish(self.vel)
                
            feedback_msg.linear_velocity = self.vel.linear.x        
            goal_handle.publish_feedback(feedback_msg)
            self.control_rate.sleep()
            self.get_logger().info(f'publishing distance_to_wall : {self.scan}')

        # Notify the action client that the goal has been completed
        goal_handle.succeed()
        end_time = time.time()


        #result.elapsed_time_s = end_time - start_time
        #result.traveled_distance = result.elapsed_time_s * 0.5
        
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MoveForwardActionServer()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()