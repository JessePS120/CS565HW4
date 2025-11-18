import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
import math
import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from enum import Enum 
import numpy as np 
import random 
from rclpy.qos import qos_profile_sensor_data
from apriltag_msgs.msg import AprilTagDetectionArray



class Locomotion(Node):

    def __init__(self):
        super().__init__('turtlebot3_locomotion')
        self.subscription = self.create_subscription(
            String,
            '/front_state',
            self.listener_callback,
            10)
        qos = QoSProfile(depth=10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.subscription  # prevent unused variable warning

        self.isStart = False
        self.startTime = time.time()
        self.forceSpin = True 
        self.start_spin_time = 5 #The time in seconds befores the robot begins spinning randomly.
        self.last_spin_time = time.time(); 
        self.spin_rate = 15 #Time in seconds between spins. 
        self.spin_velo = 1.0 #Angular velocity in rad/sec.

    def spin(self): 
        spin_time = float((2 * math.pi) / (self.spin_velo) + float(random.randint(0, 3)))
        temp_time = time.time()
        #Wait to spin 
        while(time.time() < temp_time + spin_time): 
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = self.spin_velo 
            self.pub.publish(twist_msg)

    def create_twist_msg(self, x, z):
        t = Twist()
        t.linear.x = x
        t.angular.z = z
        return t

    def log_message(self, msg):
        self.get_logger().info(str(msg))
    
    def listener_callback(self, msg):
        if (time.time() >= self.last_spin_time + self.spin_rate and time.time() >= self.startTime + self.start_spin_time) or (self.forceSpin):
            self.forceSpin = False  
            self.spin() 
            self.last_spin_time = time.time() 



        if msg.data == "unblocked":
           _twist = self.create_twist_msg(5.0, 0.0)
           self.pub.publish(_twist)
           self.log_message("Forward")
        else: 
           _twist = self.create_twist_msg(0.0, 0.5)
           self.pub.publish(_twist)
           self.log_message("Turn")

def main(args=None):
    rclpy.init(args=args)

    tb3_locomotion = Locomotion()

    rclpy.spin(tb3_locomotion)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

