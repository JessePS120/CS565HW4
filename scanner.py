import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data



class Scanner(Node):

    def __init__(self):
        super().__init__('turtlebot3_scanner')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos_profile_sensor_data)
        self.front_state_publisher = self.create_publisher(String, '/front_state', 10)
        self.threshold = 0.7
        self.get_logger().info("Starting Scanner...")
        self.subscription  # prevent unused variable warning

    def log_message(self, msg):
        self.get_logger().info(str(msg))
    
    def listener_callback(self, msg):
        first_15 = msg.ranges[0 : 5]
        last_15 = msg.ranges[355 : 360]
        front_state = first_15 + last_15
        out_msg = String()
        out_msg.data = "unblocked"
        for state in front_state:
            if state > 0.1 and state < self.threshold:
                out_msg.data = "blocked"

        
        self.front_state_publisher.publish(out_msg)
        self.log_message(front_state)
        self.log_message(out_msg.data)

        
        #self.get_logger().info(str(msg))
        #self.log_message(msg)
    	# print(msg)
        # self.get_logger().info("Testing")
        # self.scan = msg
    	# front_state = self.scan.ranges[0]
    	# out_msg = String()
    	# if front_state < self.threshold:
    	#     out_msg.data = "blocked"
    	# else:
    	#     out_msg.data = "unblocked"
    	# self.front_state_publisher.publish(out_msg)
        # self.log_message(out_msg.data)
        #print("hello")
        #self.get_logger().info("Running")
        #self.get_logger().info("front = %s" % (front_state)) 
        #self.get_logger().info(out_msg.data) 

    	# print("front = %s" %(self.scan.ranges[0]))
        # print("left  = %s" %(self.scan.ranges[90]))
        # print("back  = %s" %(self.scan.ranges[180]))
        # print("right = %s" %(self.scan.ranges[270]))

def main(args=None):
    rclpy.init(args=args)

    tb3_scanner = Scanner()

    rclpy.spin(tb3_scanner)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
