from lab02_interfaces.srv import ComputeTrajectory

import rclpy
from rclpy.node import Node
from math import atan2, sqrt, radians, degrees
from turtlesim.msg import Pose


class ComputeTrajectoryService(Node):

    def __init__(self):
        super().__init__('compute_trajectory', namespace="/turtle1")
        self.subscription = self.create_subscription(Pose, 'pose', self.listener_callback,10)
        self.srv = self.create_service(ComputeTrajectory, 'compute_trajectory', self.compute_trajectory_callback)
        self.pose = Pose() #actual position 

    def compute_trajectory_callback(self, request: ComputeTrajectory.Request, response: ComputeTrajectory.Response):
        #put your code here
        response.distance = sqrt((self.pose.x-request.x)**2 + (self.pose.y-request.y)**2)
        response.direction = atan2((self.pose.y-request.y),(self.pose.x-request.x))+radians(180.0)
        self.get_logger().debug(f'distance= {response.distance} direction= {response.direction}')
        return response
    
    def listener_callback(self, msg:Pose):
        self.pose = msg
        self.get_logger().info(f'I heard position ({self.pose.x},{self.pose.y})', throttle_duration_sec = 0.5)


def main():
    rclpy.init()

    compute_trajectory = ComputeTrajectoryService()

    rclpy.spin(compute_trajectory)

    rclpy.shutdown()


if __name__ == '__main__':
    main()