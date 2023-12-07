from lab03_interfaces.srv import ComputeTrajectory

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

        self.srv = self.create_service(ComputeTrajectory, 'compute_trajectory', self.compute_trajectory_callback)
        self.sensor_subscriber = self.create_subscription(LaserScan, '/scan', self.sensor_listener_callback, 10)
        self.subscription = self.create_subscription(Odometry, '/diff_drive_controller/odom', self.listener_callback,10)
        self.pose = Pose()

    def close_to_wall_callback(self, scan_angle, radius):
        warning = 0
        scan_angle = scan_angle
        for i in range(scan_angle-1,scan_angle+2):
            if self.sensor.ranges[i%360]<radius:
                warning+=1
        return warning>=2

    def compute_trajectory_callback(self, request: ComputeTrajectory.Request, response: ComputeTrajectory.Response):
        #put your code here
        response_angle = 0
        diff_x = request.x-self.pose.position.x
        diff_y = request.y-self.pose.position.y
        distance = sqrt((diff_x)**2 + (diff_y)**2)
        direction = int(degrees(atan2(diff_y, diff_x)))
        current_direction = self.current_direction_callback()
        absolute_direction = (direction-int(self.yaw))%360

        if self.sensor.ranges[absolute_direction] < distance: #distance to goal larger than available path in goal direction
            response.step = 0
            if not self.close_to_wall_callback(0,1.0):
                response_angle=0
            elif not self.close_to_wall_callback(90,1.0):
                response_angle = 90 #degrees
            elif not self.close_to_wall_callback(-90,1.0):
                response_angle = -90 #degrees
        elif self.close_to_wall_callback(0,1.0) and self.close_to_wall_callback(90,2.0) and self.close_to_wall_callback(-90,2.0):
            response.step=5
            response_angle = 0
        else:
            if abs(diff_x)<1.0 and abs(diff_y)<1.0: #arrived to goal
                response.step=4
                response_angle = (request.angle%360)-(self.yaw%360)
            elif abs(diff_x)<1.0:
                response.step = 2
                if diff_y>=0 and abs(current_direction)==1:
                    response_angle = int(90*sign(current_direction))#degrees
                elif diff_y<0 and abs(current_direction)==1:
                    response_angle = -90*sign(current_direction)#degrees    
            elif abs(diff_y)<1.0:
                response.step = 3
                if diff_x>=0 and abs(current_direction)==2:
                    response_angle= -90*sign(current_direction)#degrees
                elif diff_x<0 and abs(current_direction)==2:
                    response_angle = 90*sign(current_direction)#degrees
            else:
                response.step = 1
                if not self.close_to_wall_callback(0,1.1):
                    response_angle = 0 #degrees
                if not self.close_to_wall_callback(90,3.0):
                    response_angle = 90 #degrees
                elif not self.close_to_wall_callback(-90,3.0):
                    response_angle = -90 #degrees      

        response_angle = response_angle-self.rectify_angle()
        response.angle = int(response_angle%360)
        self.get_logger().info(f'called, diff_x = {diff_x}, diff_y = {diff_y})')
        self.get_logger().info(f'direction= {response.angle}, step= {response.step}')
        return response
    
    def rectify_angle(self):
        rect = self.yaw%90
        if rect>45:
            rect -= 90
        return rect
        

    def current_direction_callback(self):
        direction = 1
        if abs(self.yaw)<10:
            direction = 1 #positive x
        elif abs(self.yaw)>170:
            direction = -1 #negative x
        elif 80<self.yaw<100:
            direction = 2 #positive y
        elif -100<self.yaw<-80:
            direction = -2 #negative y 
        return direction          

    def listener_callback(self, msg:Odometry):
        self.pose.position.x = msg.pose.pose.position.x
        self.pose.position.y = msg.pose.pose.position.y
        quat = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
        _,_,self.yaw = tf_transformations.euler_from_quaternion(quat)
        self.yaw = degrees(self.yaw)
        self.get_logger().info(f'I heard position ({self.pose.position.x},{self.pose.position.y}), yaw = {self.yaw}', throttle_duration_sec = 1.5)
        
    def sensor_listener_callback(self,msg:LaserScan):
        self.sensor= msg
        #self.get_logger().info(f"I heard: dist(0)={self.sensor.ranges[0]}, dist(90)={self.sensor.ranges[90]}, dist(-90)={self.sensor.ranges[(-90%360)]}",throttle_duration_sec=1)




        



def main():
    rclpy.init()

    compute_trajectory = ComputeTrajectoryService()

    rclpy.spin(compute_trajectory)

    rclpy.shutdown()


if __name__ == '__main__':
    main()