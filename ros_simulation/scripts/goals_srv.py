#! /usr/bin/env python

import rospy
import actionlib
import actionlib.msg
import ros_simulation.msg
from ros_simulation.srv import Goals, GoalsResponse

class GoalCounter:
    def __init__(self):
        self.reached = 0
        self.cancelled = 0
        # Subscribe to the action server
        self.action_sub = rospy.Subscriber('/reaching_goal/result', ros_simulation.msg.PlanningActionResult, self.callback)
        # Create the service
        self.goals_srv = rospy.Service('goals_srv', Goals, self.get_goals)
        
    def callback(self, data):
        # Get status of the goal
        status = data.status.status
        # If the goal is reached
        if status == 3:
            self.reached += 1
        # If the goal is cancelled
        elif status == 2:
            self.cancelled += 1
        
    def get_goals(self, req):
        # Return the response
        return GoalsResponse(self.reached, self.cancelled)
    
def main():
    # Initialize the node
    rospy.init_node('goals_srv')
    goal_counter = GoalCounter()
    # Spin
    rospy.spin()

if __name__ == '__main__':
    main()

