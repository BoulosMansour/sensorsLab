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
import rclpy.logging
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class Controller(Node):

    def __init__(self):
        super().__init__('controller')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.publisher_ = self.create_publisher(Twist, 'cmd_topic', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.N = 1
        self.counter=0
        self.output = Twist()
        self.direction =1

    def changeDirection(self):
        match self.direction:
            case 1:
                self.output.linear.x = 1.0
                self.output.linear.y = 0.0
            case 2:
                self.output.linear.x = 0.0
                self.output.linear.y = 1.0
            case 3:
                self.output.linear.x = -1.0
                self.output.linear.y = 0.0
            case 0:
                self.output.linear.x = 0.0
                self.output.linear.y = -1.0

    def timer_callback(self):
        if self.counter == self.N:
            if self.direction==0:
                self.N +=1
            self.counter = 0
            self.direction = (self.direction+1)%4
        self.changeDirection()
        self.counter += 1
        
        msg = Twist()
        msg = self.output
        self.publisher_.publish(msg)
        #self.get_logger().info(f'Publishing: X = {msg.linear.x} Y = {msg.linear.y}')
        self.get_logger().debug(f'Publishing: X = {msg.linear.x} Y = {msg.linear.y}')

def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
