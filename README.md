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

```bash
rosservice call /input
