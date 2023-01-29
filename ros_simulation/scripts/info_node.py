#! /usr/bin/env python

import rospy
import math
import time

from ros_simulation.msg import Pos
from geometry_msgs.msg import Point

class InfoNode:
	
	def __init__(self):
		# Get publish frequency
		self.frequency = rospy.get_param('publish_frequency')
		
		# Time of the last print
		self.old_time = 0
		# Inizialize the variables
		self.des_x = rospy.get_param('des_x')
		self.des_y = rospy.get_param('des_y')
		
		# Subscriber to the target
		rospy.Subscriber('/goal_topic', Point, self.goal_callback)
		
		# Subscribe to the msg
		self.sub = rospy.Subscriber('/pos', Pos, self.callback)
		
	def goal_callback(self, msg):
		# Upload the target position
		self.des_x = msg.x
		self.des_y = msg.y
		
		
	def callback(self, data):
		# Get the current time in milliseconds
		current_time = time.time() * 1000
		
		# If the time difference is gretaer than the period, print the info
		if current_time - self.old_time > 1000 / self.frequency:
						
			# Calculate the (Eucledean) distance from the desired position
			distance = math.sqrt((self.des_x - data.x)**2 + (self.des_y - data.y)**2)
			
			# Calculate the avverage velocity
			velocity = math.sqrt(data.vx**2 + data.vy**2)
			
			# Print the info
			rospy.loginfo('Distance from desired position: %f', distance)
			rospy.loginfo('Average velocity: %f', velocity)
			
			# Update old time
			self.old_time = current_time


def main():
	# Initialize the node
	rospy.init_node('info_node')
	# Create an istance of InfoNode class
	InfoNode()
	# Keeps the communucation with the ROS network open
	rospy.spin()

if __name__ == '__main__':
	main()
