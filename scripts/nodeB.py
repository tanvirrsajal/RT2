#!/usr/bin/env python

import rospy
from assignment_2_2023.srv import Input, InputResponse

# Define a class for the service
class LastTargetService:
    """
    A ROS service class to provide the last desired target coordinates.
    
    Attributes
    ----------
    last_des_x : float
        The last desired x position.
    last_des_y : float
        The last desired y position.
    """

    def __init__(self):
        """
        Initializes the LastTargetService class.
        
        This method initializes the class variables for the last desired x and y positions,
        initializes the ROS node, and sets up the service to respond with the last target coordinates.
        """
        # Initialize class variables for the last desired x and y positions
        self.last_des_x = 0
        self.last_des_y = 0

        # Initialize the node with the name 'last_target_service'
        rospy.init_node('last_target_service')
        rospy.loginfo("Last target node initialized")

        # Provide a service named 'input', using the custom service type Input
        rospy.Service('input', Input, self.result_callback)

    def result_callback(self, _):
        """
        Callback function for the service.
        
        This method is called when a service request is received. It sets the response message
        with the last desired x and y positions retrieved from the ROS parameter server.
        
        Parameters
        ----------
        _ : InputRequest
            The service request (not used in this function).
        
        Returns
        -------
        InputResponse
            The response message containing the last desired x and y positions.
        """
        # Create a response message
        response = InputResponse()
        # Set the x and y inputs in the response to the last desired positions
        self.last_des_x = rospy.get_param('/des_pos_x')
        self.last_des_y = rospy.get_param('/des_pos_y')
        response.input_x = self.last_des_x
        response.input_y = self.last_des_y

        # Log information about the service request
        rospy.loginfo("Service request received. Responding with last target coordinates: x = %f, y = %f", self.last_des_x, self.last_des_y)

        # Return the response
        return response

    def spin(self):
        """
        Keeps the node running.
        
        This method keeps the ROS node active and responsive to service requests.
        """
        rospy.spin()

# Main function
if __name__ == "__main__":
    # Create an instance of the service class
    service = LastTargetService()
    # Start the node
    service.spin()

