import rclpy
from rclpy.node import Node
import rclpy.logging
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle

import time
from tf_transformations import euler_from_quaternion

from math import sqrt, radians, degrees
from numpy import sign
from lab03_interfaces.action import RotateAbsolute
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion


class RotateAbsoluteActionServer(Node):
    def __init__(self):
        super().__init__("rotate_absolute_action_server")

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
        self.yaw_subscriber = self.create_subscription(Odometry, "/diff_drive_controller/odom", self.yaw_callback, 10, callback_group=self.topic_group)
        self.cmd_vel_publisher = self.create_publisher(Twist, "cmd_vel", 10, callback_group=self.topic_group)
        
        self.condition = False
        self.rotate_vel = Twist()
        publish_period = 0.1 #seconds
        self.timer = self.create_timer(publish_period, self.pub_velocity_callback)

        # The action server must be in a different callback group since it is blocking.
        # The action server callback will sleep at a fixed rate until the goal is completed, failed 
        # or canceled. The action server will use a ReentrantCallbackGroup so that multiple goals 
        # can be handled simultaneously.

        self.action_server = ActionServer(
            self,
            RotateAbsolute,
            "/rotate_absolute",
            self.execute_action_callback,
            callback_group=self.action_server_group,
        )

        # This rate object is used to sleep the control loop at a fixed rate in the action server callback
        control_rate_hz = 200.0
        self.control_rate = self.create_rate(control_rate_hz)

        self.get_logger().info(
            "rotate absolute action server has been started, in namespace: "
            + self.get_namespace()
            + " ..."
        )

    def pub_velocity_callback(self):
        if self.condition:
            msg=self.rotate_vel
            self.cmd_vel_publisher.publish(msg)

    def yaw_callback(self, msg: Odometry):
        quat = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
        _,_,self.yaw = euler_from_quaternion(quat)
        self.yaw = degrees(self.yaw+180)%360-180
        

    def execute_action_callback(self, goal_handle: ServerGoalHandle):
        '''
        This function is called when a new goal is received by the action server.
        Pay attention to return the correct result object, otherwise the action client will crash.
        
        Returns:
            result {YourAction.Result} -- Result object
        '''
        self.get_logger().info("Executing goal...")
        feedback_msg = RotateAbsolute.Feedback()
        result = RotateAbsolute.Result()

        # Create a condition to break the control loop, it should be updated at each iteration
        init_yaw = self.yaw
        self.condition = True
        goal = (goal_handle.request.angle+180)%360-180
        start_time = time.time()
        while self.condition:
            # Execute the control loop at a fixed rate
            passed_angle = self.yaw-init_yaw
            self.remaining_angle = abs(goal-passed_angle)
            feedback_msg.remaining_angle = self.remaining_angle
            if self.remaining_angle < 10:
                self.rotate_vel.angular.z = 0.0
                self.condition = False
                self.cmd_vel_publisher.publish(self.rotate_vel) #make sure to publish vel 0 
                end_time = time.time()
            else:
                self.rotate_vel.angular.z = 1.5 * sign(goal)
            goal_handle.publish_feedback(feedback_msg)
            self.control_rate.sleep()
            self.get_logger().info(f'yaw: {self.yaw}, reamining: {self.remaining_angle}, goal:{goal}, passed:{passed_angle}')
            self.get_logger().info(f'response_vel = {self.rotate_vel.angular.z}')

        # Notify the action client that the goal has been completed
        goal_handle.succeed()

        result.elapsed_time_s = end_time - start_time
        #result.traveled_distance = result.elapsed_time_s * 0.5

        return result


def main(args=None):
    rclpy.init(args=args)
    node = RotateAbsoluteActionServer()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()