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
from rclpy.node import Node

from geometry_msgs.msg import Pose,Twist



class localization(Node):

    def __init__(self):
        super().__init__('localization')
        self.subscription = self.create_subscription(Twist, 'cmd_topic', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Pose,'pose',10)
        timer_period = 1
        self.timer = self.create_timer(timer_period,self.timer_callback)
        self.pose = Pose()

    def move(self,speed:list):
        if speed[0]!= 0:
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

    localization_ = localization()


    rclpy.spin(localization_)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    localization_.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
