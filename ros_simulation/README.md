ROS SIMULATION
===================

In this assignment we are going to learn how to operate ROS costum messages, costum services, and actions. We are also going to use the graphical interfaces (Rviz and Gazebo) to view the robot's simulation. The new package was built starting from the template provided, assignment_2_2022.

Second Assignment
===================
The task of this assignment was to implement three new nodes in the robot simulation:

* A node (a) that implements an action client, allowing the user to set a target (x, y) or to cancel it. Then publish the robot position and velocity as a custom message by relying on the values published on the topic /odom;
* A service node (b) that, when called, prints the number of goals reached and cancelled;
* A node (c) that subscribes to the robot’s position and velocity (using a custom message) and prints the distance of the robot from the target and the robot’s average speed. Use a parameter to set how fast the node publishes the information.
It is also required to create a launch file to start the simulation (assignment1.launch).

Installing and running
-----------------------

Before running the program it is required to install the xterm library, if it is not already installed on the system:

> sudo apt install xterm

We can then install the module. Go inside the root directory of your ROS workspace and run the command:

> catkin_make

Now we need to run the ROS master in a separete terminal:

> roscore

Finally, launch the simulation with the roslaunch command:

> roslaunch ros_simulation assignment1.launch

If you want to read the service's value (reached ad cancelled goals), run the command:

> rosservice call /goals_srv

Inside the "assignmet1.launch" launch file, you can set the value for the frequency with which node (c) publishes the information.

> param name="publish_frequency" type="double" value="number_value" 

Action client's pseudocode [node (a)]:
----------------------------

1. Initialize a node named "client" in ROS
2. Create an object of class "Client"
3. Within the class "Client":
   - Set two variables "position" and "linear_velocity" to None
   - Create a publisher to publish position information
   - Create a subscriber to listen for odometry information and store the received position and linear velocity information
   - Create an action client for the "reaching_goal" action server
   - Wait for the action server to become usable
   - Create a goal message for the "reaching_goal" action
   - Create a publisher to send the target
4. Implement the "odom_callback" function:
   - Store the received position and linear velocity information from the odometry message
   - Create a new message to publish the position and velocity information
   - Publish the message
5. Implement the "goal_input" function:
   - Print a prompt to ask the user to insert the goal coordinates or cancel the goal
   - Listen for user input in an infinite loop
   - If the user inputs 'c', cancel the current goal
   - If the user inputs is a float value, set the goal coordinates and send the goal to the action server and to the message
   - If the input is not a float value, print an error message
6. Call the "goal_input" function
7. Run the node

