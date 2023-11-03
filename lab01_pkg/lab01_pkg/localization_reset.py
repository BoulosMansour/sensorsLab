# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import rclpy.logging
from rclpy.node import Node

from geometry_msgs.msg import Pose,Twist
from std_msgs.msg import Bool


class Localization_reset(Node):

    def __init__(self):
        super().__init__('localization_reset')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.reset_subscription = self.create_subscription(Bool, 'reset', self.reset_listener_callback, 10)
        self.subscription = self.create_subscription(Twist, 'cmd_topic', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.reset_subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Pose,'pose',10)
        timer_period = 1
        self.timer = self.create_timer(timer_period,self.timer_callback)
        self.pose = Pose()
        self.reset = Bool()

    def reset_listener_callback(self, msg:Bool):
        self.reset = msg.data
        if (self.reset):
            self.pose = Pose()
            self.get_logger().info('reset')

    def move(self,speed:list):
        if speed[0] != 0:
            self.pose.position.x += speed[0]
        if speed[1] != 0:
            self.pose.position.y += speed[1]

    def listener_callback(self, msg: Twist):
        speed = [msg.linear.x,msg.linear.y]
        self.move(speed)
        #self.get_logger().info(f'I heard: X = {msg.linear.x} Y={msg.linear.y}')
        self.get_logger().debug(f'I heard: X = {msg.linear.x} Y={msg.linear.y}')

    def timer_callback(self):
        msg = Pose()
        msg = self.pose
        self.publisher_.publish(msg)
        #self.get_logger().info(f'Publishing: position ({msg._position.x},{msg.position.y})')
        self.get_logger().debug(f'Publishing: position ({msg._position.x},{msg.position.y})')


def main(args=None):
    rclpy.init(args=args)

    localization_reset = Localization_reset()


    rclpy.spin(localization_reset)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    localization_reset.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
