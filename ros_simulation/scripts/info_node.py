#! /usr/bin/env python

import rospy
import math
from std_msgs.msg import String
from RT_assignment_2.msg import Pos

class InfoNode:
	def __init__(self):
		# Set up the subscriber
		self.subscriber = rospy.Subscriber("/pos", Pos self.callback)
		# Desired position
		self.des_x = rospy.get_param("des_x")
		self.des_y = rospy.get_param("des_y")
		# Frequency of the node in Hz
		self.frequency = rospy.get_param("publish_frequency")
		# Time of the last print
		self.old_time = 0
	
	def callback(self, msg):
		# Get the current time in milliseconds
		current_time = rospy.get_time() * 1000
		
		# If the time difference is greater than the period -> print the info
		if current_time - self.old_time > 1000 / self.frequency:
			# Calculate the (Euclidean) distance from the desired position
			distance = math.sqrt((self.des_x . msg.x)**2 + (self.des_y - msg.y)**2)
			# Calculate the average velocity
			velocity = math.sqrt(msg.vx**2 + msg.vy**2)
			# Print the info
			rospy.loginfo("Distance from desired position: %f", distance)
			rospy.loginfo("Average velocity: %f", velocity)
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
