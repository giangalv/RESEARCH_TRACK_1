#! /usr/bin/env python

import rospy
import actionlib
import ros_simulation.msg

class Client:

    def __init__(self):
        self.position = None
        self.linear_velocity = None
        self.pub = rospy.Publisher('/pos', ros_simulation.msg.Pos, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.action_client = actionlib.SimpleActionClient('/reaching_goal', ros_simulation.msg.PlanningAction)
        self.action_client.wait_for_server()
        self.goal = ros_simulation.msg.PlanningGoal()

    def odom_callback(self, data):
        self.position = data.pose.pose.position
        self.linear_velocity = data.twist.twist.linear
        msg = ros_simulation.msg.Pos()
        msg.x = self.position.x
        msg.y = self.position.y
        msg.vx = self.linear_velocity.x
        msg.vy = self.linear_velocity.y
        self.pub.publish(msg)

    def goal_input(self):
        print("Insert the goal coordinates (x,y) or type 'c' to cancel the goal, then press ENTER:")
        while not rospy.is_shutdown():
            input, o, e = select.select([sys.stdin], [], [], 1)
            if (input):
                input = sys.stdin.readline().rstrip()
                if input == 'c':
                    self.action_client.cancel_goal()
                    print("Goal cancelled")
                else:
                    self.goal.target_pose.pose.position.x = float(input.split(',')[0])
                    self.goal.target_pose.pose.position.y = float(input.split(',')[1])
                    self.action_client.send_goal(self.goal)

def main():
	# Initialize the node
	rospy.init_node('client')
	client = Client()
	client.goal_input()
	rospy.spin()

if __name__ == '__main__':
	main()

