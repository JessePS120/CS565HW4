# HW4
# Wesley Junkins
# Jesse Seidel
# Austin Smith
# Chanakya Setty

import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np 
from apriltag_msgs.msg import AprilTagDetectionArray
from rclpy.qos import qos_profile_sensor_data

#Set to true to enable debugging via print().
DEBUG = False 

class Walk(Node):
    
    def __init__(self):
        super().__init__('Track')
        self.isStart = False 
        self.startTime = time.time()
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.subscription = self.create_subscription( 
            LaserScan, 
            '/scan', 
            self.sensor_callback, 
            qos_profile_sensor_data
        )
        self.mode = "NAVIGATE"  # NAVIGATE or SCAN
        #April tag Subscription. 
        self.subscription = self.create_subscription(AprilTagDetectionArray, '/detections', self.listener_callback, 1)
        self.subscription  # prevent unused variable warning
        
        # Scanning mode variables
        self.scan_interval = 10.0  # Time in seconds between scans
        self.last_scan_time = time.time()
        self.scan_duration = 5.0  # Time to complete 360 degree scan
        self.scan_start_time = None
        self.scan_angular_velocity = 0.5  # Angular velocity for scanning (rad/s)

    def listener_callback(self, msg):
        if len(msg.detections) > 0:
            print("Tag detected %s" % (msg.detections[0].id))
        
    # Use 180-degree lidar for obstacle avoidance and steering
    def calculate_180_degree_velocity(self, front_180_data):
        # Split the 180 degrees into left and right halves
        left_half = front_180_data[:90]
        right_half = front_180_data[90:]
        
        # Calculate average distance for each half (more distance = more openness)
        left_openness = np.mean(left_half)
        right_openness = np.mean(right_half)
        
        # Calculate bias: positive means turn right, negative means turn left
        bias = right_openness - left_openness
        
        # Angular velocity calculation
        max_angular_vel = 1.0  # Maximum angular velocity
        scale_factor = 0.5      # How sensitive the turning is
        angular_velocity = bias * scale_factor
        
        # Limit the angular velocity to prevent excessive turning
        if angular_velocity > max_angular_vel:
            angular_velocity = max_angular_vel
        elif angular_velocity < -max_angular_vel:
            angular_velocity = -max_angular_vel
        
        # Linear velocity calculation based on overall openness
        min_distance = np.min(front_180_data)
        
        # Base linear velocity
        base_linear_vel = 0.8  # Base speed for 180-degree mode
        
        # Adjust speed based on obstacles
        if min_distance < 0.4:
            linear_velocity = 0.0
            angular_velocity = 1.0
        elif min_distance < 1.0:
            linear_velocity = 0.2
        elif min_distance < 2.0:
            linear_velocity = 0.5
        else:
            linear_velocity = base_linear_vel
            
        return angular_velocity, linear_velocity

    def sensor_callback(self, msg):
        current_time = time.time()
        
        # Check if we should enter scanning mode
        if self.mode == "NAVIGATE" and (current_time - self.last_scan_time) >= self.scan_interval:
            self.mode = "SCAN"
            self.scan_start_time = current_time
            if DEBUG:
                print("Entering scan mode for April tag detection")
        
        # Check if we should exit scanning mode
        if self.mode == "SCAN" and (current_time - self.scan_start_time) >= self.scan_duration:
            self.mode = "NAVIGATE"
            self.last_scan_time = current_time
            if DEBUG:
                print("Exiting scan mode, resuming navigation")
        
        # Handle scanning mode - rotate 360 degrees
        if self.mode == "SCAN":
            angular_velocity = self.scan_angular_velocity
            linear_velocity = 0.0  # Stop forward movement during scan
        else:
            # Navigation mode - use 180-degree lidar for obstacle avoidance
            laser_ranges = np.array(msg.ranges)
            middle_index = len(laser_ranges) // 2
            start_index = middle_index - 90
            end_index = middle_index + 90
            lidar_front_180 = laser_ranges[start_index:end_index]
            
            angular_velocity, linear_velocity = self.calculate_180_degree_velocity(lidar_front_180)
        
        # Create and send twist command
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = angular_velocity
        
        # Publish the twist command
        self.publisher.publish(twist_msg)
        
        # Print debug information
        if DEBUG: 
        	print(f"Mode: {self.mode} \t | \t Linear: {linear_velocity:.2f} \t | \t Angular: {angular_velocity:.2f} \t | \t Time: {time.time() - self.startTime:.2f}")

def main(args=None):
    rclpy.init(args=args)
    Walk_Node = Walk()
    rclpy.spin(Walk_Node)
    Walk_Node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
