#! /usr/bin/env python

## @package ros_simulation
# \file goals_srv.py
# \brief Provides a service to return the number of goals reached and cancelled by the action server.
# \author Galvagni Gianluca
# \version 0.1
# \date 14/03/2023
#
# \details
#
# Subscribes to: 
#   - '/reaching_goal/result': topic that publishes the result of a goal reached or cancelled.
#
# Publishes to: 
#   [None]
#
# Service: 
#   - '/goals_srv': service that returns the number of goals reached and cancelled.
#
##

import rospy
import actionlib
import actionlib.msg
import ros_simulation.msg

from ros_simulation.srv import Goals, GoalsResponse

##
# \class GoalCounter
# \brief Counts the goals reached and cancelled.
#
# # This class counts the number of goals reached and cancelled by subscribing to the /reaching_goal/result topic, 
# and provides a service to return these values.
class GoalCounter:

	##
    # \brief Initializes the GoalCounter class and sets the initial values of the reached and cancelled variables to zero.
    #
    def __init__(self):
		# Set to zero the two variables
        self.reached = 0
        self.cancelled = 0
        
        # Create the service to print the variables
        self.goals_srv = rospy.Service('goals_srv', Goals, self.get_goals)
        
        # Subscribe to the action server
        self.action_sub = rospy.Subscriber('/reaching_goal/result', ros_simulation.msg.PlanningActionResult, self.callback) 

    ##
    # \brief Callback function for the /reaching_goal/result topic subscription.
    # \param data: The message received from the topic subscription.
    #
    # This function updates the values of the reached and cancelled variables based on the status of the received goal.   
    def callback(self, data):
        # Get status of the goal
        status = data.status.status
        
        # If the goal is reached
        if status == 3:
            self.reached += 1
            
        # If the goal is cancelled
        elif status == 2:
            self.cancelled += 1

    ##
    # \brief Service function to return the number of goals reached and cancelled.
    # \param req: The service request (which is not used in this case).
    # \return GoalsResponse: A message containing the values of the reached and cancelled variables.
    #
    # This function returns the values of the reached and cancelled variables in a GoalsResponse message.  
    def get_goals(self,req):
        # Return the variables
        return GoalsResponse(self.reached, self.cancelled)
    
def main():
    # Initialize the node
    rospy.init_node('goals_srv')
    # Create an object for the class GoalCounter
    goal_counter = GoalCounter()
    # Keeps the communucation with the ROS network open
    rospy.spin()

if __name__ == '__main__':
    main()
