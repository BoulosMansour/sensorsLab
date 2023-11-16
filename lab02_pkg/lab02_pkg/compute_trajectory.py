from lab02_interfaces.srv import ComputeTrajectory

import rclpy
from rclpy.node import Node
from math import atan2,degrees
from turtlesim.msg import Pose


class ComputeTrajectoryService(Node):

    def __init__(self):
        super().__init__('compute_trajectory')
        self.srv = self.create_service(ComputeTrajectory, 'compute_trajectory', self.compute_trajectory_callback)
        self.declare_parameter('turtle1/compute_trajectory','initial position')
        self.subscription = self.create_subscription(Pose, 'turtle1/pose', self.listener_callback,10)
        self.pose = Pose() #actual position 

    def compute_trajectory_callback(self, request, response):
        #put your code here
        response.distance = ((self.pose.x-request.x)**2 + (self.pose.y-request.y)**2)**(1/2)
        response.direction = degrees(atan2((self.pose.y-request.y),(self.pose.x-request.x)))
        return response
    
    def listener_callback(self, msg:Pose):
        self.pose = msg
        self.get_logger().info(f'I heard position ({self.pose.x},{self.pose.y})')


def main():
    rclpy.init()

    compute_trajectory = ComputeTrajectoryService()

    rclpy.spin(compute_trajectory)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
