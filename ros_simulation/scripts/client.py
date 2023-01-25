import rospy
import actionlib
import actionlib.msg
import ros_simulation.msg
from ros_simulation.srv import Goals, GoalsResponse

class Client:
	def __init__(self):
		# Publisher for the position of the robot
		self.publisher = rospy.Publisher("/pos", Pos, queue_size = 1)
		# Subscriber to the odometry of the robot to get the position and velocity
		self.odometry_subscriber = rospy.Subscriber("/odom", Odometry, self.odometry_callback)
		# Create an action client
		self.client = actionlib.SimpleActionClient("/reaching_goal", ros_simulation.msg.PlanningAction)
		# Wait for the action server to start
		self.client_wait_for_server()
		
	def odometry_callback(self, data):
		# Get position and linear velocity from odometry
		position = data.pose.pose.position
		linear_velocity = data.twist.twist.linear
		# Create a message of type Pos
		msg = Pos()
		# Fill the message with the position and linear velocity
		msg.x = position.x
		msg.y = position.y
		msg.vx = linear_velocity.x
		msg.vy = linear_velocity.y
		# Publish the message
		self.publisher.publish(msg)
		
	def run(self):
		# Print the instructions
		print("Insert the goal coordinates (x,y) or type on the keyboard 'c' to cancel the goal, then press ENTER:")
		
		while not rospy.is_shutdown():
			value = input()
			# If the input is 'c' cancel the goal
			if value == 'c':
				self.client.cancel_goal()
				print("Goal cancelled")
			else:
				# Create a message of type PlanningGoal
				goal = ros_simulation.msg.PlanningGoal()
				# Fill the message with the goal coordinates
				goal.target_pose.pose.position.x = float(value.split(','[0])
				goal.target_pose.pose.position.t = float(value.split(','[1])
				# Send the goal to the action server
				self.client.send_goal(goal)
				
def main():
	client = Client()
	client.run()
	
if __name__ == '__main__':
	rospy.init_node("client")
	main()
