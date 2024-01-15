#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Pose, Twist
from nav_msgs.msg import Odometry
import actionlib
from assignment_2_2023.msg import Vel
from assignment_2_2023.msg import PlanningAction, PlanningGoal
from std_srvs.srv import SetBool
from actionlib_msgs.msg import GoalStatus

class GoalHandler:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('set_target_client')

        # Initialize publisher and action client
        self.pub = rospy.Publisher("/pos_vel", Vel, queue_size=1)
        self.client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
        self.client.wait_for_server()
        self.goal_cancelled = True  # Flag to track if the current goal has been cancelled

    def handle_goal_commands(self):
        # Main loop for handling goal commands
        while not rospy.is_shutdown():
            self.subscribe_to_odometry()
            command = input("Press 's' to set a new goal or 'q' to cancel the current goal: ")
            
            if command == 's':
                self.set_new_goal()
            elif command == 'q':
                self.cancel_current_goal()
            else:
                rospy.logwarn("Invalid command. Please enter 's' or 'q'.")

    def subscribe_to_odometry(self):
        # Subscribe to /odom topic to get position and velocity updates
        rospy.Subscriber("/odom", Odometry, self.publish_position_velocity)

    def set_new_goal(self):
        # Set a new goal based on user input
        target_pos_x = rospy.get_param('/des_pos_x')
        target_pos_y = rospy.get_param('/des_pos_y')

        goal = PlanningGoal()
        goal.target_pose.pose.position.x = target_pos_x
        goal.target_pose.pose.position.y = target_pos_y
        rospy.loginfo("Current goal coordinates: x = %f, y = %f", target_pos_x, target_pos_y)

        try:
            input_x = float(input("Enter the x-coordinate for the new goal: "))
            input_y = float(input("Enter the y-coordinate for the new goal: "))
        except ValueError:
            rospy.logwarn("Invalid input. Please enter valid numerical values.")
            return

        rospy.set_param('/des_pos_x', input_x)
        rospy.set_param('/des_pos_y', input_y)
        goal.target_pose.pose.position.x = input_x
        goal.target_pose.pose.position.y = input_y

        self.client.send_goal(goal)
        self.goal_cancelled = False
        rospy.loginfo("New goal set: x = %f, y = %f", input_x, input_y)

    def cancel_current_goal(self):
        # Cancel the current goal if there is one
        if not self.goal_cancelled:
            self.goal_cancelled = True
            self.client.cancel_goal()
            rospy.loginfo("Current goal cancelled by user")
        else:
            rospy.loginfo("No active goal to cancel")

    def publish_position_velocity(self, msg):
        # Publish current position and velocity
        current_pos = msg.pose.pose.position
        current_vel_linear = msg.twist.twist.linear
        current_vel_angular = msg.twist.twist.angular

        pos_and_vel = Vel()
        pos_and_vel.pos_x = current_pos.x
        pos_and_vel.pos_y = current_pos.y
        pos_and_vel.vel_x = current_vel_linear.x
        pos_and_vel.vel_z = current_vel_angular.z

        self.pub.publish(pos_and_vel)

def main():
    # Initialize the GoalHandler and start handling goal commands
    handler = GoalHandler()
    handler.handle_goal_commands()

if __name__ == '__main__':
    main()
