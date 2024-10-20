#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import random
from geometry_msgs.msg import Twist

class RandomWalk(Node):
    def __init__(self):
        super().__init__('random_walk')
        
        # Create a publisher to publish velocity commands to the /cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Set the rate of publishing messages (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 0.1 seconds = 10 Hz
        
        # Create a Twist message to store the linear and angular velocities
        self.twist = Twist()

    def timer_callback(self):
        # Random linear velocity between 0 and 0.3 m/s
        self.twist.linear.x = random.uniform(0.0, 0.3)
        
        # Random angular velocity between -1.0 and 1.0 rad/s
        self.twist.angular.z = random.uniform(-1.0, 1.0)
        
        # Publish the velocity command to the /cmd_vel topic
        self.publisher_.publish(self.twist)

def main(args=None):
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create the RandomWalk node
    random_walk = RandomWalk()
    
    # Spin the node to keep it alive and processing callbacks
    rclpy.spin(random_walk)
    
    # Shutdown the node once finished
    random_walk.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        # Call the main function to start the node
        main()
    except KeyboardInterrupt:
        pass
