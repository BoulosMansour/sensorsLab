from lab03_interfaces.srv import StartTrajectory

import tf_transformations
from nav_msgs.msg import Odometry
from numpy import sign
import rclpy
from rclpy.node import Node
from math import atan2, sqrt, degrees, radians
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan


class ComputeTrajectoryService(Node):

    def __init__(self):
        super().__init__('compute_trajectory')
        self.srv = self.create_service(StartTrajectory, 'start_trajectory', self.start_trajectory_callback)
        self.sensor_subscriber = self.create_subscription(LaserScan, '/scan', self.sensor_listener_callback, 10)
        self.pose = Pose()

    def close_to_wall_callback(self, scan_angle):
        warning = 0
        scan_angle = scan_angle
        for i in range(scan_angle-1,scan_angle+2):
            self.get_logger().info(f'scan_i = {i%360}')
            if self.sensor.ranges[i%360]<3.5:
                warning+=1
        return warning>=2

    def start_trajectory_callback(self, request: StartTrajectory.Request, response: StartTrajectory.Response):
        #put your code here
        if not self.close_to_wall_callback(0):
            response_angle = 0
        elif not self.close_to_wall_callback(90):
            response_angle = 90
        elif not self.close_to_wall_callback(-90):
            response_angle = -90
        elif not self.close_to_wall_callback(180):
            response_angle = 180
        response.angle = int(response_angle%360)
        self.get_logger().debug(f'direction= {response.angle}')
        return response
        
    def sensor_listener_callback(self,msg:LaserScan):
        self.sensor= msg
        self.get_logger().info(f"I heard: dist(0)={self.sensor.ranges[0]}, dist(90)={self.sensor.ranges[90]}, dist(-90)={self.sensor.ranges[(-90%360)]}",throttle_duration_sec=1)


def main():
    rclpy.init()

    compute_trajectory = ComputeTrajectoryService()

    rclpy.spin(compute_trajectory)

    rclpy.shutdown()


if __name__ == '__main__':
    main()