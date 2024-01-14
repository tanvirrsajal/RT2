# RT1 Assignment 2 - ROS Package for Robot Control and Data Gathering

This ROS package consists of three nodes designed to control robot movement in a specific environment and gather relevant data. Additionally, a launch file is provided to initiate the entire simulation.

## Installation

1. Clone the RT1_assignment_2 repository:

    ```bash
    git clone https://github.com/LemmaMoto/RT1_assignment_2.git
    ```

2. Install xterm:

    ```bash
    sudo apt-get -y install xterm
    ```

3. Ensure executable permissions for Python files inside the 'scripts' folder:

    ```bash
    chmod +x scripts/node_a.py
    chmod +x scripts/node_b.py
    chmod +x scripts/node_c.py
    chmod +x scripts/bug_as.py
    chmod +x scripts/go_to_point_service.py
    chmod +x scripts/wall_follow_service.py
    ```

4. Run the program using the provided launch file:

    ```bash
    roslaunch assignment_2_2023 assignment1.launch
    ```

## Nodes

### **1. node_a.py**

This Python script creates a ROS node for robot interaction within a ROS environment. It enables the user to assign new targets (by inputting 'y') or abort (by inputting 'c') the existing target for the robot, while also broadcasting the robot's current location and speed.

### **2. node_b.py**

This Python script defines a ROS node that provides a service to return the last desired position of a robot. It provides a service named 'input' that returns the target positions when called.

To visualize the information, you can call the service using the following command in a new terminal:
3. node_c.py

This Python script defines a ROS node that provides a service to return the average velocity and the distance between the current and desired positions of a robot. It subscribes to the /pos_vel topic to update these values and provides a service named 'info_service' that returns these values.
Pseudocode

python

# Pseudocode for node_a.py

# Import necessary libraries

# Define the GoalHandler class

    # Initialize the class
        # Create a publisher to the /pos_vel topic
        # Create an action client for the /reaching_goal action server
        # Wait for the action server to be available
        # Initialize a flag to indicate if the current goal has been cancelled

    # Define the handle_goal_commands method
        # Loop until ROS is shutdown
            # Subscribe to the /odom topic and call publish_position_velocity method when a message is received
            # Prompt the user to enter a command
            # Get the current target position from the parameter server
            # Create a new goal with the current target position
            # If the user command is 'y'
                # Prompt the user to enter the x and y coordinates for the new goal
                # If the input is valid, update the target position parameters and the goal
                # Send the new goal to the action server
                # Update the goal cancelled flag
            # If the user command is 'c'
                # If there is an active goal, cancel it and update the goal cancelled flag
            # Log the last received goal

    # Define the publish_position_velocity method
        # Extract the current position and velocity from the Odometry message
        # Create a new Vel message with the current position and velocity
        # Publish the Vel message

# Define the main function
    # Initialize the node
    # Create an instance of the GoalHandler class
    # Call the handle_goal_commands method of the GoalHandler instance

# If the script is the main program, call the main function

Possible Improvements

    Currently, when node_c publishes a message before a goal is defined, the distance slightly increases over time. To address this, a threshold could be introduced to ignore minor changes.

    At present, when the robot encounters a wall, it turns in a fixed direction, not necessarily towards the shortest path. An improvement could be to modify the algorithm so that the robot calculates the optimal direction to turn.

```bash
rosservice call /input
