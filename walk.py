# HW4
# Wesley Junkins
# Jesse Seidel
# Austin Smith
# Chanakya Setty

import rclpy
from rclpy.node import Node
import time
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from enum import Enum 
import numpy as np 
import random 

#Set to true to enable debugging via print().
DEBUG = False 

class Walk(Node):
    
    def __init__(self):
        super().__init__('Track')
        self.isStart = False 
        self.startTime = time.time()
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription( 
            LaserScan, 
            '/base_scan', 
            self.sensor_callback, 
            10
        )
        self.mode = "FULL"
        #Variables for robot spinning 
        self.forceSpin = True 
        self.start_spin_time = 60 #The time in seconds befores the robot begins spinning randomly.
        self.last_spin_time = time.time(); 
        self.spin_rate = 20 #Time in seconds between spins. 
        self.spin_velo = 0.5 #Angular velocity in rad/sec. 
        self.exploration_period = 25.0  # WES ADDED
        self.exploration_duration = 5.0  # WES ADDED
        self.exploration_end_time = self.startTime  # WES ADDED
        self.next_exploration_time = self.startTime + self.exploration_period  # WES ADDED
        self.exploration_turn_bias = 0.3  # WES ADDED
        self.heading_bias_end = self.startTime  # WES ADDED
        self.heading_bias_rate = 0.0  # WES ADDED

    def spin(self): 
        spin_time = float((2 * math.pi) / (self.spin_velo) + float(random.randint(0, 1)))
        temp_time = time.time()
        #Wait to spin 
        while(time.time() < temp_time + spin_time): 
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = self.spin_velo 
            self.publisher.publish(twist_msg)
        self.initialize_heading_bias()  # WES ADDED
         
    # Scan the front 180 degrees of lidar range and calculate angular velocity needed to go in the direction of more openness
    def scan_area(self, laser_data):
        # Get the front 180 degrees of lidar range
        laser_ranges = np.array(laser_data.ranges)
        middle_index = len(laser_ranges) // 2
        start_index = middle_index - 90
        end_index = middle_index + 90
        lidar_front_180 = laser_ranges[start_index:end_index]
        exploring = self.update_exploration_state()  # WES ADDED
        
        # Check if we can go straight through a 20-degree cone
        cone_clear = self.check_front_cone_clear(laser_ranges, middle_index, exploring)  # WES ADDED
        
        if cone_clear:
            # Use 70-degree cone for fine adjustments
            self.mode = "CONE"
            angular_velocity, linear_velocity = self.calculate_cone_velocity(laser_ranges, middle_index, exploring)  # WES ADDED
        else:
            # Use 180-degree bias for obstacle avoidance
            self.mode = "FULL"
            angular_velocity, linear_velocity = self.calculate_180_degree_velocity(lidar_front_180, exploring)  # WES ADDED
        
        angular_velocity, heading_active = self.apply_heading_bias(angular_velocity)  # WES ADDED
        if heading_active and linear_velocity > 0.0:  # WES ADDED
            linear_velocity = max(0.2, linear_velocity - 0.05)  # WES ADDED
        return angular_velocity, linear_velocity
    
    def update_exploration_state(self):  # WES ADDED
        current_time = time.time()  # WES ADDED
        if current_time >= self.next_exploration_time:  # WES ADDED
            self.exploration_end_time = current_time + self.exploration_duration  # WES ADDED
            self.next_exploration_time = current_time + self.exploration_period  # WES ADDED
        return current_time < self.exploration_end_time  # WES ADDED
        
    def check_front_cone_clear(self, laser_ranges, middle_index, exploring=False):  # WES ADDED
        # Check 70-degree cone in the center (35 degrees on each side)
        cone_start = middle_index - 35
        cone_end = middle_index + 35
        cone_data = laser_ranges[cone_start:cone_end]
        
        # Check if there's clear space (2 meters) in the front cone
        clear_distance = 1.5 if exploring else 2.0  # WES ADDED
        max_distance_in_cone = np.mean(cone_data)
        
        # Return True if there's at least one clear path (2+ meters) in the cone
        # This allows us to detect doors even if part of the cone faces a wall
        return max_distance_in_cone > clear_distance
        
    def calculate_cone_velocity(self, laser_ranges, middle_index, exploring=False):  # WES ADDED
        # Get 90-degree cone data (45 degrees on each side of center)
        cone_start = middle_index - 45
        cone_end = middle_index + 45
        cone_data = laser_ranges[cone_start:cone_end]
        
        # Split cone into left and right halves (45 measurements each)
        left_cone = cone_data[:45]
        right_cone = cone_data[45:]
        
        # Calculate average distance for each half
        left_openness = np.mean(left_cone)
        right_openness = np.mean(right_cone)
        
        # Calculate bias for fine adjustments
        bias = right_openness - left_openness
        exploratory_turn = 0.0  # WES ADDED
        if exploring:  # WES ADDED
            bias *= 0.4  # WES ADDED
            exploratory_turn = self.exploration_turn_bias * random.uniform(-1.0, 1.0)  # WES ADDED
        
        # Angular velocity calculation (fine adjustments)
        max_angular_vel = 0.4  # Much smaller than 180-degree version
        scale_factor = 0.1     # Much more gentle for fine adjustments
        if laser_ranges[cone_start] > 0.5 and laser_ranges[cone_end] > 0.5: 
            angular_velocity = bias * random.betavariate(0.3, 0.3) 
        else:  
            angular_velocity = bias * scale_factor 
        angular_velocity += exploratory_turn  # WES ADDED
        
        # Limit the angular velocity for fine adjustments
        if angular_velocity > max_angular_vel:
            angular_velocity = max_angular_vel
        elif angular_velocity < -max_angular_vel:
            angular_velocity = -max_angular_vel
        
        # Linear velocity calculation based on cone openness
        # Use the minimum distance in cone to determine how fast to go
        min_distance_in_cone = np.min(cone_data)
        
        # Base linear velocity
        base_linear_vel = 0.5  # Base speed when cone is clear
        
        # Slow down as we get closer to obstacles
        if min_distance_in_cone < 1.0:
            linear_velocity = 0.3
        elif min_distance_in_cone < 2.0:
            linear_velocity = 0.5
        else:
            linear_velocity = base_linear_vel
        if exploring and min_distance_in_cone > 0.7:  # WES ADDED
            linear_velocity = min(linear_velocity + 0.1, base_linear_vel + 0.2)  # WES ADDED
            
        return angular_velocity, linear_velocity
        
    def calculate_180_degree_velocity(self, front_180_data, exploring=False):  # WES ADDED
        # Split the 180 degrees into left and right halves
        left_half = front_180_data[:90]
        right_half = front_180_data[90:]
        
        # Calculate average distance for each half (more distance = more openness)
        left_openness = np.mean(left_half)
        right_openness = np.mean(right_half)
        
        # Calculate bias: positive means turn right, negative means turn left
        bias = right_openness - left_openness
        if exploring:  # WES ADDED
            bias *= 0.5  # WES ADDED
        
        # Angular velocity calculation
        max_angular_vel = 1.0  # Maximum angular velocity
        scale_factor = 0.5      # How sensitive the turning is
        angular_velocity = bias * scale_factor
        if exploring:  # WES ADDED
            angular_velocity += self.exploration_turn_bias * random.uniform(-0.8, 0.8)  # WES ADDED
        
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
        if exploring and min_distance > 0.6:  # WES ADDED
            linear_velocity = min(linear_velocity + 0.1, base_linear_vel)  # WES ADDED
            
        return angular_velocity, linear_velocity
    
    def initialize_heading_bias(self):  # WES ADDED
        target_angle = random.uniform(-math.pi, math.pi)  # WES ADDED
        bias_magnitude = random.uniform(0.25, 0.6)  # WES ADDED
        duration = abs(target_angle) / bias_magnitude if bias_magnitude > 0 else 0.0  # WES ADDED
        direction = 1.0 if target_angle >= 0 else -1.0  # WES ADDED
        self.heading_bias_rate = bias_magnitude * direction  # WES ADDED
        self.heading_bias_end = time.time() + duration  # WES ADDED
    
    def apply_heading_bias(self, angular_velocity):  # WES ADDED
        current_time = time.time()  # WES ADDED
        if current_time < self.heading_bias_end:  # WES ADDED
            return angular_velocity + self.heading_bias_rate, True  # WES ADDED
        self.heading_bias_rate = 0.0  # WES ADDED
        return angular_velocity, False  # WES ADDED

    def sensor_callback(self, msg):
        #Tell the robot to spin based off of the spin rate. 
        print(time.time())
        if (time.time() >= self.last_spin_time + self.spin_rate and time.time() >= self.startTime + self.start_spin_time) or (self.forceSpin):
            self.forceSpin = False  
            self.spin() 
            self.last_spin_time = time.time() 
    
        # Get lidar data and calculate velocities using our scan_area function
        angular_velocity, linear_velocity = self.scan_area(msg)
        
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
