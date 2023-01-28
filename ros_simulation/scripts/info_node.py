#! /usr/bin/env python

import rospy
import math
import time

from ros_simulation.msg import Pos

class InfoNode:
    def __init__(self):
        self.frequency = 1.0 # Frequency of the node in Hz
        self.old_time = 0    # Time of the last print

        # Get publish frequency
        self.frequency = rospy.get_param('publish_frequency')
        # Subscribe to the msg
        self.odom = rospy.Subscriber('/pos', Pos, self.callback)

    def callback(self, data):
        # Get the current time in milliseconds
        current_time = time.time() * 1000

        # If the time difference is greater than the period print the info
        if current_time - self.old_time > 1000 / self.frequency:
            # Get desired position
            des_x = rospy.get_param('des_x')
            des_y = rospy.get_param('des_y')

            # Calculate the (Euclidean) distance from the desired position
            distance = math.sqrt((des_x - data.x)**2 + (des_y - data.y)**2)

            # Calculate the average velocity
            velocity = math.sqrt(data.vx**2 + data.vy**2)

            # Print the info
            rospy.loginfo('Distance from desired position: %f', distance)
            rospy.loginfo('Average velocity: %f', velocity)

            # Update old time
            self.old_time = current_time

def main():
    # Initializate the node
    rospy.init_node("info_node")
    
    # Create an instance of the class
    info_node = InfoNode()
    
    # Spin
    rospy.spin()
    
if __name__ == '__main__':
    main()

