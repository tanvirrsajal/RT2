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
This Python script serves as a client within a robotic system, facilitating goal-setting and cancellation interactions with an action server responsible for planning and executing robot movements. The script imports essential ROS and custom message types, including Vel for position and velocity, PlanningAction for goal planning, and standard ROS messages like Odometry, Point, Pose, and Twist. The primary class, GoalHandler, initializes a ROS publisher (/pos_vel topic) for sending velocity and position information and an action client (/reaching_goal action server) for interacting with the goal planning system. The handle_goal_commands() method operates in a loop, subscribing to the /odom topic to obtain the robot's odometry information. It prompts users to input commands ('s' for setting a new goal or 'q' for canceling the current goal) and processes these commands accordingly. Additionally, the publish_position_velocity() method extracts current position and velocity information from the /odom topic, publishing this data on the /pos_vel topic. The main() function initializes the ROS node, creates an instance of the GoalHandler class, and calls the handle_goal_commands() method to start the user interaction loop. 
In summary, this script provides a straightforward interface for users to set new goals or cancel the current goal within a robotic system, utilizing an action client to communicate with a goal planning action server while continuously updating position and velocity information on specific topics.

### **2. nodeB.py**
This Python script functions as a ROS node, operating as a service client in a robotic system. Its primary role is to manage service requests for the last desired x and y positions. The script begins by importing necessary ROS modules and custom message types, such as Vel, Input, and InputResponse. Within the LastTargetService class, the initialization step involves setting up class variables and providing an 'input' service using the Input service type. The result_callback function, acting as a callback for the service, retrieves the last desired x and y positions from ROS parameters and sends them as a response. The spin() method ensures the node remains active. In the main function, an instance of LastTargetService is created, initiating the node's operation. The execution flow involves the node initializing and providing the 'input' service. Upon receiving a service request, the result_callback function retrieves the last desired positions and sends a response. The node continues to run, ready for new service requests. 
In summary, this ROS node serves as a client, addressing requests for the last desired x and y positions within a broader robotic system.

To visualize the information, you can call the service using the following command in a new terminal:
```bash
    rosservice call /input
```
### **3. nodeC.py**
This Python script defines a versatile ROS node, serving as both a service client and a subscriber within a robotic system. The node performs essential functionalities such as calculating the distance between desired and actual positions and computing the average velocity within a specified window. Key components include the importation of necessary ROS modules and custom message types, with the inclusion of the math module for distance calculations. The InfoService class is instrumental in initializing variables for average velocity and distance, providing a 'info_service' service, and subscribing to '/pos_vel.' The callback function, get_distance_and_averagevelocity, extracts information related to desired and actual positions, velocity window size, and subsequently computes the distance and average velocity. The callback function for the service, get_values, responds to service requests with the calculated distance and average velocity. The spin() method ensures the node's continuous operation, and the main function orchestrates the instantiation of the InfoService class. The script's execution flow involves the node initializing, providing the 'info_service' service, and subscribing to '/pos_vel.' The subscriber callback calculates distance and average velocity upon receiving position and velocity information, while the service callback responds to service requests. The main function continuously calls the service, logging the responses. 
In summary, this ROS node seamlessly provides real-time information about distance and average velocity, demonstrating its versatility as both a service client and a subscriber within a robotic system.


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
