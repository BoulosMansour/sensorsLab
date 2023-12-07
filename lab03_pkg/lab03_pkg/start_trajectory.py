from lab03_interfaces.srv import StartTrajectory
from numpy import sign
from statistics import mean
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class StartTrajectoryService(Node):

    def __init__(self):
        super().__init__('start_trajectory')
        self.sensor_subscriber = self.create_subscription(LaserScan, '/scan', self.sensor_listener_callback, 10)
        self.srv = self.create_service(StartTrajectory, 'start_trajectory', self.start_trajectory_callback)

    def sensor_listener_callback(self,msg:LaserScan):
        self.scan_ranges= msg
        self.get_logger().info(f"I heard: dist(0)={msg.ranges[0]}, dist(90)={msg.ranges[90]}, dist(-90)={msg.ranges[(-90%360)]}",throttle_duration_sec=1)

    
    def distance_to_wall_callback(self,angle):
        distance = mean ([self.scan_ranges.ranges[(angle-1)%360], self.scan_ranges.ranges[angle%360], self.scan_ranges.ranges[(angle+1)%360]])
        return distance

    def start_trajectory_callback(self, request: StartTrajectory.Request, response: StartTrajectory.Response):
        #put your code here
        response.angle = int(self.best_direction()%360)
        self.get_logger().debug(f'direction= {response.angle}')
        return response
        
    def best_direction(self):
        direction = 0
        dist_front = self.distance_to_wall_callback(0)
        dist_left = self.distance_to_wall_callback(90)
        dist_right = self.distance_to_wall_callback(-90)
        dist_back = self.distance_to_wall_callback(180)
        max_dist = max([dist_front,dist_left,dist_right,dist_back])
        if dist_front == max_dist:
            direction = 0
        elif dist_left == max_dist:
            direction = 90
        elif dist_right == max_dist:
            direction =-90
        elif dist_back == max_dist:
            direction = 180
        return direction

def main():
    rclpy.init()

    start_trajectory = StartTrajectoryService()

    rclpy.spin(start_trajectory)

    rclpy.shutdown()


if __name__ == '__main__':
    main()