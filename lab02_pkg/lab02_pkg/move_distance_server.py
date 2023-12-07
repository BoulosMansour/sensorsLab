import rclpy
from rclpy.node import Node
import rclpy.logging
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle

import time

from math import sqrt
from lab02_interfaces.action import MoveDistance
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class MoveDistanceActionServer(Node):
    def __init__(self):
        super().__init__("move_distance_action_server", namespace="/turtle1")

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
        self.pose_subscriber = self.create_subscription(Pose, "pose", self.pose_callback, 10, callback_group=self.topic_group)
        self.cmd_vel_publisher = self.create_publisher(Twist, "cmd_vel", 10, callback_group=self.topic_group)

        #parameters
        self.declare_parameter('max_vel',0.5)

        self.max_vel = self.get_parameter('max_vel').value
        self.vel = Twist()
        self.publish = False
        self.pose = Pose()

        publish_period = 0.1 #seconds
        self.timer = self.create_timer(publish_period, self.pub_velocity_callback)

        # The action server must be in a different callback group since it is blocking.
        # The action server callback will sleep at a fixed rate until the goal is completed, failed 
        # or canceled. The action server will use a ReentrantCallbackGroup so that multiple goals 
        # can be handled simultaneously.

        self.action_server = ActionServer(
            self,
            MoveDistance,
            "/turtle1/move_distance",
            self.execute_action_callback,
            callback_group=self.action_server_group,
        )

        # This rate object is used to sleep the control loop at a fixed rate in the action server callback
        control_rate_hz = 2.0
        self.control_rate = self.create_rate(control_rate_hz)

        self.get_logger().info(
            "Move distance action server has been started, in namespace: "
            + self.get_namespace()
            + " ..."
        )

    def pub_velocity_callback(self):
            if (self.publish):
                msg = self.vel
                self.cmd_vel_publisher.publish(msg)

    def pose_callback(self, msg: Pose):
        self.pose = msg
        

    def execute_action_callback(self, goal_handle: ServerGoalHandle):
        '''
        This function is called when a new goal is received by the action server.
        Pay attention to return the correct result object, otherwise the action client will crash.
        
        Returns:
            result {YourAction.Result} -- Result object
        '''
        self.get_logger().info("Executing goal...")
        feedback_msg = MoveDistance.Feedback()
        result = MoveDistance.Result()

        # Create a condition to break the control loop, it should be updated at each iteration
        init_pose = self.pose
        condition = True
        goal = goal_handle.request.distance
        self.vel.linear.x = 0.5
        start_time = time.time()
        self.publish =True
        while condition:
            # Execute the control loop at a fixed rate
            passed_distance = sqrt(((self.pose.x-init_pose.x)**2) + ((self.pose.y-init_pose.y)**2))
            self.remaining_distance = goal-passed_distance
            feedback_msg.remaining_distance = self.remaining_distance
            if self.remaining_distance < 0.3:
                self.vel.linear.x = 0.0
                condition = False
                self.publish = False
                end_time = time.time()
            else:
                self.vel.linear.x = self.max_vel
            goal_handle.publish_feedback(feedback_msg)
            self.control_rate.sleep()

        # Notify the action client that the goal has been completed
        goal_handle.succeed()

        result.elapsed_time_s = end_time - start_time
        result.traveled_distance = result.elapsed_time_s * 0.5
        self.publish = False
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MoveDistanceActionServer()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()