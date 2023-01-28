#! /usr/bin/env python

import rospy
import math
import time
import sys
import select
import actionlib
import actionlib.msg

from std_srvs.srv import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Twist
import ros_simulation.msg
from ros_simulation.msg import Pos

class Client():
	def __init__(self):
		# Publisher for the position of the robot
		self.pub = rospy.Publisher('/pos', Pos, queue_size=1)
		# Subscriber to the odometry of the robot to get the position and velocity
		self.odom = rospy.Subscriber('/odom', Odometry, self.callback)
		# Create an action client
		self.client = actionlib.SimpleActionClient('/reaching_goal', ros_simulation.msg.PlanningAction)
		# Wait for the action server to start
		self.client.wait_for_server()

	def callback(self, data):
		"""
		Get position and linear velocity from odometry
		Create a message of type Pos
		Fill the message with the position and linear velocity
		Publish the message
		"""
		position = data.pose.pose.position
		linear_velocity = data.twist.twist.linear
		msg = Pos()
		msg.x = position.x
		msg.y = position.y
		msg.vx = linear_velocity.x
		msg.vy = linear_velocity.y
		self.pub.publish(msg)

	def action_client(self):
		"""
		Print instructions
		Wait for input of goal coordinates or 'c' to cancel goal
		Create a message of type PlanningGoal
		Fill the message with the goal coordinates
		Send the goal to the action server
		"""
		print("Insert the goal coordinates (x,y) or type 'c' to cancel the goal, then press ENTER:")

		while not rospy.is_shutdown():
			input, o, e = select.select([sys.stdin], [], [], 1)
			if (input):
				input = sys.stdin.readline().rstrip()
			if input == 'c':
				self.client.cancel_goal()
				print("Goal cancelled")
			else:
				goal = ros_simulation.msg.PlanningGoal()
				goal.target_pose.pose.position.x = float(input.split(',')[0])
				goal.target_pose.pose.position.y = float(input.split(',')[1])
				self.client.send_goal(goal)

def main():
	# Initialize the node
	rospy.init_node('client')
	Client()
	rospy.spin()

if __name__ == '__main__':
	main()

