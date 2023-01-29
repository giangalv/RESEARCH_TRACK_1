#! /usr/bin/env python

import rospy
import math
import sys
import select
import actionlib
import actionlib.msg
import ros_simulation.msg

from nav_msgs.msg import Odometry
from std_srvs.srv import *
from geometry_msgs.msg import Point, Pose, Twist
from ros_simulation.msg import Pos

class Client:

	def __init__(self):
		# Set two variables with a null value
		self.position = None
		self.linear_velocity = None
		
		# Create a publisher to publish position information
		self.pub = rospy.Publisher('/pos', ros_simulation.msg.Pos, queue_size = 1)
		
		# Create a subscriber to listen for odometry information
		rospy.Subscriber('/odom', Odometry, self.odom_callback)
		
		# Create an action client for the reaching_goal 
		self.action_client = actionlib.SimpleActionClient('/reaching_goal', ros_simulation.msg.PlanningAction)
		
		# Wait for the action server to come usable
		self.action_client.wait_for_server()
		
		# Create a goal message
		self.goal = ros_simulation.msg.PlanningGoal()
		
		# Create a publisher to send the target
		self.goal_pub = rospy.Publisher('/goal_topic', Point, queue_size = 1)
		

	def odom_callback(self, data):
		# Store the position and linear velocity information from the odometry message
		self.position = data.pose.pose.position
		self.linear_velocity = data.twist.twist.linear
		
		# Create a new message to publish position and velocity information
		msg = ros_simulation.msg.Pos()
		msg.x = self.position.x
		msg.y = self.position.y
		msg.vx = self.linear_velocity.x
		msg.vy = self.linear_velocity.y
		
		# Publish the message
		self.pub.publish(msg)
		
		
	def goal_input(self):
		
		print("Insert the goal coordinates (x,y) or type 'c' to cancel the goal, then press ENTER:")
		
		# infinite loop for listen the user's input
		while not rospy.is_shutdown():
			input, o, e = select.select([sys.stdin], [], [], 1)
			
			if (input):
				# Read the input
				input = sys.stdin.readline().rstrip()
				
				# If the user inputs is 'c', cancel the current goal 
				if input == 'c':
					self.action_client.cancel_goal()
					print("Goal cancelled")
					
				# Otherwise
				else:
					# Check if the input is float value and set the goal coordinates 
					# and send the goal to the action server and to the messsage
					try:
						x, y = [float(val) for val in input.split(',')]
						self.goal.target_pose.pose.position.x = x
						self.goal.target_pose.pose.position.y = y  
					
						goal_msg = Point()
						goal_msg.x = self.goal.target_pose.pose.position.x
						goal_msg.y = self.goal.target_pose.pose.position.y
						self.goal_pub.publish(goal_msg)                                    
								  
						self.action_client.send_goal(self.goal)
					# Print a error message if the input is not a float value
					except ValueError:
						print("Invalid input. Please enter the goal coordinates in the format 'x,y'")												
									

def main():
	# Initialize the node
	rospy.init_node('client')
	# Create an object for the class Client
	client = Client()
	# call the goal_input() metod on the client object
	client.goal_input()
	# Keeps the communucation with the ROS network open
	rospy.spin()

if __name__ == '__main__':
	main()
