#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
from ros_simulation.msg import PlanningGoal, Pos
import actionlib

class Client:
    def __init__(self):
        self.pub = rospy.Publisher('/pos', Pos, queue_size=1)
        self.client = actionlib.SimpleActionClient('/reaching_goal', ros_simulation.msg.PlanningAction)
        self.client.wait_for_server()
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.goal = PlanningGoal()
        
    def odom_callback(self, data):
        position = data.pose.pose.position
        linear_velocity = data.twist.twist.linear
        pos_msg = Pos()
        pos_msg.x = position.x
        pos_msg.y = position.y
        pos_msg.vx = linear_velocity.x
        pos_msg.vy = linear_velocity.y
        self.pub.publish(pos_msg)
        
    def input_callback(self):
        while not rospy.is_shutdown():
            input_str = input("Insert the goal coordinates (x,y) or type 'c' to cancel the goal, then press ENTER:")
            if input_str == 'c':
                self.client.cancel_goal()
                print("Goal cancelled")
            else:
                try:
                    x, y = map(float, input_str.split(','))
                    self.goal.target_pose.pose.position.x = x
                    self.goal.target_pose.pose.position.y = y
                    self.client.send_goal(self.goal)
                except ValueError:
                    print("Invalid input, please try again")

def main():
	# Initialize the node
	rospy.init_node('client')
	client = Client()
	client.input_callback()
	rospy.spin()

if __name__ == '__main__':
	main()

