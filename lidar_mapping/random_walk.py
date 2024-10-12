#!/usr/bin/env python3

import rospy
import random
from geometry_msgs.msg import Twist

def random_walk():
    # Initialize the ROS node
    rospy.init_node('random_walk', anonymous=True)
    
    # Create a publisher to publish velocity commands to the /cmd_vel topic
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # Set the rate of publishing messages (10 Hz)
    rate = rospy.Rate(10)  # 10 Hz
    
    # Create a Twist message to store the linear and angular velocities
    twist = Twist()

    # Loop to keep sending random velocity commands until the node is shut down
    while not rospy.is_shutdown():
        # Random linear velocity between 0 and 0.3 m/s
        twist.linear.x = random.uniform(0.0, 0.3)
        
        # Random angular velocity between -1.0 and 1.0 rad/s (to turn the robot)
        twist.angular.z = random.uniform(-1.0, 1.0)
        
        # Publish the velocity command to the /cmd_vel topic
        pub.publish(twist)
        
        # Sleep for a while to maintain the 10 Hz rate
        rate.sleep()

if __name__ == '__main__':
    try:
        # Call the random_walk function to start the autonomous movement
        random_walk()
    except rospy.ROSInterruptException:
        pass
