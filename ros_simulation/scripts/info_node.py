#! /usr/bin/env python

##
# \file info_node.py
# \brief Subscribes to odometry information, publishes position and velocity information, sends goals to the action server, and listens for user input to set the goal coordinates. It calculates the distance from the desired position and the average velocity, and logs this information at a specified frequency.
# \author Galvagni Gianluca
# \version 0.1
# \date 14/03/2023
#
# \details
#
# Subscribes to: <BR>
#	- '/msg/Pos': Odometry information.
#	- '/goal_topic' (geometry_msgs/Point) : Desired position to reach.
#
# Publishes to: <BR>
#	[None]
#
# Services: <BR>
#	[None]
##

import rospy
import math
import time

from ros_simulation.msg import Pos
from geometry_msgs.msg import Point

##
# \class InfoNode
# \brief ROS node to the robots' information (x,y,vx and vy) and prints the distance between robot and target.
#
# This class subscribes to odometry information, sends goals to the action server and listens for user input to set the goal coordinates.
class InfoNode:

	##
	# \brief Constructor for InfoNode class.
	#
	# This function initializes the InfoNode class and its variables. It gets the publish frequency and initializes the target 
	# position variables to their default values. It also subscribes to the /goal_topic and /pos topics.
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
	
	##
	# \brief Callback function for goal topic subscriber.
    # \param msg: A Point message containing the goal coordinates.
	#
	# This function is the callback for the /goal_topic subscriber. It updates the target position variables with the values received in the message.
	def goal_callback(self, msg):
		# Upload the target position
		self.des_x = msg.x
		self.des_y = msg.y
		
	##
	# \brief Callback function for pos subscriber.
    # \param data: A Pos message containing the current position and velocity information.
	#
	# This function is the callback for the /pos subscriber. It calculates the Euclidean distance between the current position and the desired position,
	# as well as the average velocity, and prints this information if enough time has passed since the last print.
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
