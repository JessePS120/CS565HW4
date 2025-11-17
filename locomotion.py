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
            print(f"[DEBUG] Tag detected: ID={msg.detections[0].id}")
        
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
        
        print(f"[DEBUG] Left openness: {left_openness:.3f}m, Right openness: {right_openness:.3f}m, Bias: {bias:.3f}")
        
        # Angular velocity calculation
        max_angular_vel = 1.0  # Maximum angular velocity
        scale_factor = 0.5      # How sensitive the turning is
        angular_velocity = bias * scale_factor
        
        # Limit the angular velocity to prevent excessive turning
        if angular_velocity > max_angular_vel:
            angular_velocity = max_angular_vel
            print(f"[DEBUG] Angular velocity capped at max: {max_angular_vel}")
        elif angular_velocity < -max_angular_vel:
            angular_velocity = -max_angular_vel
            print(f"[DEBUG] Angular velocity capped at min: {-max_angular_vel}")
        
        # Linear velocity calculation based on overall openness
        min_distance = np.mean(front_180_data)
        
        print(f"[DEBUG] Minimum distance in front 180°: {min_distance:.3f}m")
        
        # Base linear velocity
        base_linear_vel = 0.8  # Base speed for 180-degree mode
        
        # Adjust speed based on obstacles
        if min_distance < 0.4:
            linear_velocity = 0.0
            angular_velocity = 1.0
            print(f"[DEBUG] Very close obstacle detected (<0.4m), stopping and turning")
        elif min_distance < 1.0:
            linear_velocity = 0.2
            print(f"[DEBUG] Close obstacle detected (<1.0m), slowing down")
        elif min_distance < 2.0:
            linear_velocity = 0.5
            print(f"[DEBUG] Moderate obstacle distance (<2.0m), moderate speed")
        else:
            linear_velocity = base_linear_vel
            print(f"[DEBUG] Clear path ahead (>2.0m), full speed")
            
        return angular_velocity, linear_velocity

    def sensor_callback(self, msg):
        current_time = time.time()
        
        # Get full lidar scan and print minimum distance
        laser_ranges = np.array(msg.ranges)
        # Filter out invalid readings (inf, nan, or out of range)
        valid_ranges = laser_ranges[np.isfinite(laser_ranges) & (laser_ranges > 0) & (laser_ranges < msg.range_max)]
        if len(valid_ranges) > 0:
            min_distance_full_scan = np.min(valid_ranges)
            print(f"[LIDAR] Minimum distance in full 360° scan: {min_distance_full_scan:.3f}m")
        else:
            print(f"[LIDAR] No valid lidar readings in scan")
        
        # Check if we should enter scanning mode
        time_since_last_scan = current_time - self.last_scan_time
        if self.mode == "NAVIGATE" and time_since_last_scan >= self.scan_interval:
            self.mode = "SCAN"
            self.scan_start_time = current_time
            print(f"[MODE] Entering SCAN mode for April tag detection (time since last scan: {time_since_last_scan:.2f}s)")
        elif self.mode == "NAVIGATE":
            time_until_scan = self.scan_interval - time_since_last_scan
            print(f"[DEBUG] Navigation mode - Time until next scan: {time_until_scan:.2f}s")
        
        # Check if we should exit scanning mode
        if self.mode == "SCAN":
            scan_elapsed = current_time - self.scan_start_time
            scan_remaining = self.scan_duration - scan_elapsed
            print(f"[DEBUG] Scan mode - Elapsed: {scan_elapsed:.2f}s, Remaining: {scan_remaining:.2f}s")
            
            if scan_elapsed >= self.scan_duration:
                self.mode = "NAVIGATE"
                self.last_scan_time = current_time
                print(f"[MODE] Exiting SCAN mode, resuming NAVIGATE mode")
        
        # Handle scanning mode - rotate 360 degrees
        if self.mode == "SCAN":
            angular_velocity = self.scan_angular_velocity
            linear_velocity = 0.0  # Stop forward movement during scan
            print(f"[SCAN] Rotating at {angular_velocity:.2f} rad/s for April tag detection")
        else:
            # Navigation mode - use 180-degree lidar for obstacle avoidance
            middle_index = len(laser_ranges) // 2
            start_index = middle_index - 90
            end_index = middle_index + 90
            lidar_front_180 = laser_ranges[start_index:end_index]
            
            print(f"[DEBUG] Using front 180° lidar data (indices {start_index} to {end_index} of {len(laser_ranges)})")
            
            angular_velocity, linear_velocity = self.calculate_180_degree_velocity(lidar_front_180)
        
        # Create and send twist command
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = angular_velocity
        
        print(f"[CMD] Publishing command - Linear: {linear_velocity:.3f} m/s, Angular: {angular_velocity:.3f} rad/s")
        
        # Publish the twist command
        self.publisher.publish(twist_msg)
        
        # Print debug information
        if DEBUG: 
        	print(f"[DEBUG] Mode: {self.mode} | Linear: {linear_velocity:.2f} | Angular: {angular_velocity:.2f} | Time: {time.time() - self.startTime:.2f}")

def main(args=None):
    rclpy.init(args=args)
    Walk_Node = Walk()
    rclpy.spin(Walk_Node)
    Walk_Node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
