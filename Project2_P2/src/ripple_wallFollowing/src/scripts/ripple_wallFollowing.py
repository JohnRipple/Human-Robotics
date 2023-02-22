#!/usr/bin/env python
# John Ripple
# Human Centered Robotics
# Project 2 Part 2
# 3/8/2023
# With reference to http://wiki.ros.org/turtlesim/Tutorials/Go%20to%20Goal

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from sensor_msgs.msg import LaserScan
import math

class Triton:

    def __init__(self):
            """Initializes node, publisher, and subscriber when object is created"""
            rospy.init_node('ripple_drawM', anonymous=False)    # Initialize the drawM node
            self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10) # Publish to the topic /turtle1/cmd_vel
            self.lidar_subscriber = rospy.Subscriber("/scan", LaserScan , self.update_scan)
            self.lidar = LaserScan()
            self.rate = rospy.Rate(50) # in hz


    def update_scan(self, data):
          """Callback function when the Pose message type is recieved by the subscriber"""
          self.lidar = data


    # def distance(self, goal_pose):
    #       """Distance between goal and current position"""
    #       return math.sqrt((goal_pose.x - self.pose.x)**2 + (goal_pose.y - self.pose.y)**2)
    

    # def linear_vel(self, goal_pose, constant=1.5):
    #       """Linear velocity in x direction based on distance to goal"""
    #       return constant * self.distance(goal_pose) + 1
    

    # def steering_angle(self, goal_pose):
    #       """Triangle angle between the two points: tan^-1(y/x)"""
    #       return math.atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)
    

    # def angle_dist(self, goal_pose):
    #       """Find the angular distance to rotate"""
    #       return self.steering_angle(goal_pose) - self.pose.theta
    
    
    # def angular_vel(self, goal_pose, constant=6):
    #       """Angular velocity to travel to get to the pose"""
    #       return constant * self.angle_dist(goal_pose)


    # def move2goal(self, x, y):
    #     """Move the turtle to the goal"""
    #     goal_pose = Pose()
    #     goal_pose.x = self.pose.x + x   # Goal is current position + offset from bound
    #     goal_pose.y = self.pose.y + y

    #     # Tolerances
    #     distance_tolerance = 0.05
    #     angle_tolerance = 0.0001

    #     vel_msg = Twist()

    #     angled = False
    #     distance_prev = self.distance(goal_pose)

    #     # Loop while not at the goal
    #     while self.distance(goal_pose) >= distance_tolerance and distance_prev >= self.distance(goal_pose):
    #         distance_prev = self.distance(goal_pose)    # Assign current distance to previous distance

    #         # Either rotate or move but not both at the same time, only turns once
    #         if abs(self.angle_dist(goal_pose)) >= angle_tolerance and angled == False:
    #                 vel_msg.angular.z = self.angular_vel(goal_pose, 20)
    #                 vel_msg.linear.x = 0
    #                 #print(self.angle_dist(goal_pose))
    #         else:
    #                 vel_msg.angular.z = 0
    #                 vel_msg.linear.x = self.linear_vel(goal_pose, 20)
    #                 angled = True
    #                 # print("Goal X: %.4f\tGoal y: %.4f" %(goal_pose.x, goal_pose.y))
    #                 # print(self.pose)
    #         self.velocity_publisher.publish(vel_msg)
    #         self.rate.sleep()

    #     # Stop the turtle from moving once reaching the goal
    #     vel_msg.angular.z = 0
    #     vel_msg.linear.x = 0
    #     self.velocity_publisher.publish(vel_msg)
    #     self.rate.sleep()


    def publishData(self, vel_msg):
        self.velocity_publisher.publish(vel_msg)
        self.rate.sleep()


    def followWall(self):
        j = 0
        vel_msg = Twist()
        while j < 300:
            j = j + 1 
            #print(self.lidar)
            vel_msg.linear.x = 0.22
            print(vel_msg)
            self.publishData(vel_msg)
        vel_msg.linear.x = 0
        self.publishData(vel_msg)
    



if __name__ == '__main__':
    try:
        triton = Triton()
        triton.followWall()

    except rospy.ROSInterruptException:
        pass

                      
