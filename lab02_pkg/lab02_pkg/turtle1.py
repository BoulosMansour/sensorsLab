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

from geometry_msgs.msg import Pose


class Turtle1(Node):

    def __init__(self):
        super().__init__('turtle1')
        self.publisher = self.create_publisher(Pose, 'position', 10)
        timer_period = 1 #seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.pose = Pose()

    def timer_callback(self):
        msg = Pose()
        msg = self.pose
        self.publisher.publish(msg)  
        self.get_logger().info(f'publishing position: ({msg.position.x},{msg.position.y}')

def main(args=None):
    rclpy.init(args=args)

    turtle1 = Turtle1()

    rclpy.spin(turtle1)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    turtle1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()