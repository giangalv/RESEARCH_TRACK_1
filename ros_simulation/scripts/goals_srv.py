#! /usr/bin/env python

import rospy

from ros_simulation.srv import Goals, GoalsResponse
from ros_simulation.msg import Pos, PlanningAction, PlanningGoal, PlanningActionResult

class GoalsService:
    def __init__(self):
        self.reached = 0
        self.cancelled = 0
        # Initialize the node
        rospy.init_node('goals_srv')
        # Create the service
        srv = rospy.Service('goals_srv', Goals, self.get_goals)
        # Subscribe to the action server
        action = rospy.Subscriber('/reaching_goal/result', ros_simulation.msg.PlanningActionResult, self.callback)
        # Spin
        rospy.spin()

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
    GoalsService()

if __name__ == '__main__':
    main()
