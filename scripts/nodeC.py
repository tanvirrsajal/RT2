#!/usr/bin/env python

import rospy
import math
from assignment_2_2023.msg import Vel
from assignment_2_2023.srv import AvgVelDis, AvgVelDisResponse

# Define a class for the service
class InfoService:
    def __init__(self):
        # Initialize class variables for the average velocity and distance
        self.average_vel_x = 0
        self.distance = 0

        # Initialize the node with the name 'info_service'
        rospy.init_node('info_service')
        rospy.loginfo("Information service node initialized")

        # Provide a service named 'info_service', using the custom service type AvgVelDis
        rospy.Service("info_service", AvgVelDis, self.get_values)
        # Subscribe to the '/pos_vel' topic, using the custom message type Vel
        rospy.Subscriber("/pos_vel", Vel, self.update_distance_and_average_velocity)

    # Callback function for the subscriber
    def update_distance_and_average_velocity(self, msg):
        # Get the desired x and y positions from the parameter server
        des_x = rospy.get_param('/des_pos_x')
        des_y = rospy.get_param('/des_pos_y')

        # Get the window size for the velocity calculation from the parameter server
        velocity_window_size = rospy.get_param('/window_size')
        
        # Get the actual x and y positions from the message
        actual_x = msg.pos_x
        actual_y = msg.pos_y
        
        # Calculate the distance between the desired and actual positions
        des_coordinates = [des_x, des_y]
        actual_coordinates = [actual_x, actual_y]
        self.distance = math.dist(des_coordinates, actual_coordinates)

        # Calculate the average velocity
        if isinstance(msg.vel_x, list):
            vel_data = msg.vel_x[-velocity_window_size:]
        else:
            vel_data = [msg.vel_x]

        self.average_vel_x = sum(vel_data) / min(len(vel_data), velocity_window_size)

        # Log the current values
        rospy.loginfo("Distance: %f, Average Velocity: %f", self.distance, self.average_vel_x)

    # Callback function for the service
    def get_values(self, _):
        # Return a response with the distance and average velocity
        return AvgVelDisResponse(self.distance, self.average_vel_x)		      

    # Function to keep the node running
    def spin(self):
        rospy.spin()

# Main function
if __name__ == "__main__":
    # Create an instance of the service class
    service = InfoService()

    # Start the node
    service.spin()
