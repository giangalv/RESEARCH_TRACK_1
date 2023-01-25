import rospy
from RT_assignment_2.srv import Goals, GoalsResponse

class GoalService:
	def __init__(self):
		# Create the service
		self.service = rospy.Service("goals_srv", Goals, self.handle_goals)
		# Counter for reached and cancelled goal positions
		self.reached = 0
		self.cancelled = 0
		# Subscribe to the action server
		self.subscriber = rospy.Subscriber("/reaching_goal/result", ros_simulation.msg.Pos, self.callback)
		
	def callback(self, data):
	# Get status of the goal
	status = data.status.status
	# If the goal is reached
	if status == 3:
		self.reached += 1
	# If the goal is cancelled
	elif status == 2
		self.cancelled += 1
		
	def handle_goals(self, req):
		# Return the responde
		return GoalsResponse(self.reached, self.cancelled)
		
def main():
	# Initiallize the node
	rospy.init_node("goals_srv")
	# Create an instance of the class
	goal_service() = GoalService()
	# Spin
	rospy.spin()
	
if _name__ == '__main__':
	main()
