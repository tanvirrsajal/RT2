# RT1 Assignment2 - Robot Simulation Control Using ROS

The goal for this project was to develop 3 nodes to send target destination to the robot, storing the information to the server and showing the average velocity & distance of the robot from the target. There are other node files which were already developed.

## Permissions
Ensure executable permissions for Python files inside the 'scripts' folder:

```bash
    chmod +x scripts/nodeA.py
    chmod +x scripts/nodeB.py
    chmod +x scripts/nodeC.py
    chmod +x scripts/bug_as.py
    chmod +x scripts/go_to_point_service.py
    chmod +x scripts/wall_follow_service.py
```
## Installation
 ```bash
    sudo apt-get -y install xterm
 ```

## Seting up the environment
 ```bash
    mkdir RT1
    cd RT1
    mkdir src
    cd src
    git clone https://github.com/tanvirrsajal/RT1_Assignment2.git
    cd ..
    catkin_make
    source devel/setup.bash
    roslaunch assignment_2_2023 assignment1.launch
```
Please note you need to change of the name of the folder from RT1_Assignment2 to assignment_2_2023. If the last command does not work please go inside the launch folder(RT1/src/assignment_2_2023/launch) and run roslaunch assignment1.launch. When running roslaunch please make sure you are running roscore in another terminator window. You also need to modify your .bashrc file and add the path of this folder to your .bashrc file.
You can follow the following step to edit your bash file.

```bash
gedit .bashrc
```
```
source your-path/devel/setup.bash
```
Instead of your-path it should be the original path of your folder.

## Nodes

### **1. nodeA.py**
This Python script defines a ROS (Robot Operating System) node that serves as a client for a simple robotic goal-setting and cancellation system. The node is designed to interact with an action server responsible for planning and executing robot movements to reach specified goals.

Here's a breakdown of the main components and functionalities of the node:

Imports: The necessary ROS and custom message imports are included, such as messages for position and velocity (Vel), goal planning (PlanningAction, PlanningGoal, PlanningResult), and standard ROS messages like Odometry, Point, Pose, Twist.

Class Definition - GoalHandler: The main class GoalHandler is defined, encapsulating the functionality of handling user commands related to setting and canceling goals.

Initialization: The class initializes a ROS publisher (/pos_vel topic) for sending velocity and position information, and an action client (/reaching_goal action server) for interacting with the goal planning system.

handle_goal_commands() Method: This method runs in a loop, subscribing to the /odom topic to obtain the robot's odometry information. It prompts the user to input commands ('s' for setting a new goal or 'q' for canceling the current goal) and processes these commands accordingly.

publish_position_velocity() Method: Extracts current position and velocity information from the /odom topic and publishes this information on the /pos_vel topic.

Main Function - main(): Initializes the ROS node named 'set_target_client' and creates an instance of the GoalHandler class. It then calls the handle_goal_commands() method to start the user interaction loop.
Node Execution: The script checks if it's the main module (__name__ == '__main__') and, if so, calls the main() function to execute the node.

In summary, this ROS node provides a simple interface for users to set new goals for a robot or cancel the current goal. It utilizes an action client to communicate with a goal planning action server, and it continuously publishes the robot's position and velocity information on a specific topic.

### **2. node_b.py**
This Python script defines a ROS node, that acts as a service client in a robotic system. Its primary purpose is to handle service requests for the last desired x and y positions. Here's a concise overview:

Imports: The script imports necessary ROS modules and custom message types (Vel, Input, InputResponse).
    Class Definition - LastTargetService:
        Initialization: Initializes class variables and provides a 'input' service using the Input service type.
        Callback Function (result_callback): Retrieves the last desired x and y positions from ROS parameters and sends them as a response.
        spin() Method: Keeps the node running.
Main Function:
        Creates an instance of LastTargetService and starts the node.
Execution Flow:
        Node initializes, providing the 'input' service.
        When a service request is received, the result_callback function retrieves the last desired x and y positions.
        A response is created and sent back to the requester.
        The node continues running, awaiting new service requests.

In summary, this ROS node serves as a client, responding to requests for the last desired x and y positions, contributing to a broader robotic system.

To visualize the information, you can call the service using the following command in a new terminal:
```bash
    rosservice call /input
```
3. node_c.py


This Python script defines a ROS node that acts as both a service client and a subscriber. The node calculates the distance between desired and actual positions and computes the average velocity within a specified window. Key components include:
Components:

Imports:
    ROS modules and custom message types are imported.
    The math module is used for distance calculations.

Class Definition - InfoService:
Initialization:
Initializes variables for average velocity and distance.
Node initialization as 'info_service.'
Provides a 'info_service' service and subscribes to '/pos_vel.'
Callback Function (get_distance_and_averagevelocity):
Extracts desired and actual positions, velocity window size, and computes distance and average velocity.
Callback Function for the Service (get_values):
Returns a service response with the calculated distance and average velocity.
spin() Method:
Keeps the node running using rospy.spin().
Main Function:
Creates an instance of the InfoService class.
Utilizes a service proxy to call 'info_service' in a loop, logging the responses.
Execution Flow:
Node initializes, providing 'info_service' service and subscribing to '/pos_vel.'
Subscriber callback calculates distance and average velocity upon receiving position and velocity information.
Service callback responds to service requests with the calculated distance and average velocity.
The main function continuously calls the service, logging responses.

In summary, this ROS node provides real-time information about distance and average velocity in a robotic system, serving as both a service client and a subscriber.


## Pseudocode for nodeA.py
```
1. Initialize ROS node 'set_target_client'.
2. Create an instance of the GoalHandler class.
3. Initialize the action client for '/reaching_goal' action server.
4. Wait for the action server to become available.
5. Initialize the goal_cancelled flag to track the current goal status.

6. While not rospy.is_shutdown():
    7. Subscribe to '/odom' topic to get odometry information.
    8. Publish current position and velocity on '/pos_vel' topic.
    9. Prompt the user for a command ('s' to set a new goal, 'q' to cancel the current goal).
    10. Get the current target position from ROS parameters ('/des_pos_x', '/des_pos_y').

    11. If command is 's':
        12. Prompt the user for new goal coordinates.
        13. Update target position parameters and create a new PlanningGoal.
        14. Send the new goal to the action server.
        15. Set goal_cancelled to False.

    16. Else, if command is 'q':
        17. If a goal is active (goal_cancelled is False), cancel the current goal.
        18. Set goal_cancelled to True.

    19. Else (invalid command):
        20. Log a warning about the invalid command.

    21. Log information about the last goal's target coordinates.

22. Define a callback function publish_position_velocity(msg) to extract and publish current position and velocity.
23. Define the main() function:
    24. Initialize the ROS node 'set_target_client'.
    25. Create an instance of the GoalHandler class.
    26. Call the handle_goal_commands() method to start handling goal commands.

27. If __name__ == '__main__':
    28. Call the main() function to execute the node.

```

## Possible Improvements

When the robot hits a wall, it turns in a one direction only. We can make it find the shortest path and follow that side.
