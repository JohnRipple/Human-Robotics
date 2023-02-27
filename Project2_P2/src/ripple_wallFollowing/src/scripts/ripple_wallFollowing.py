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
from itertools import chain

class Triton:

    def __init__(self):
            """Initializes node, publisher, and subscriber when object is created"""
            rospy.init_node('ripple_drawM', anonymous=False)    # Initialize the drawM node
            self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10) # Publish to the topic /turtle1/cmd_vel
            self.lidar_subscriber = rospy.Subscriber("/scan", LaserScan , self.update_scan)
            self.lidar = LaserScan()
            self.regions = {
                 'right' : 0,
                 'front' : 0,
                 'left' : 0
            }
            self.rate = rospy.Rate(50) # in hz


    def update_scan(self, data):
        """Callback function when the Pose message type is recieved by the subscriber"""
        # angle_min = 0
        # angle_max = 360
        # len(ranges) = 360   one datapoint per degree
        self.lidar = data
        offset = 20
        self.regions = {
            'right' : min(self.lidar.ranges[270-offset:270+offset]),
            'front' : min(chain(self.lidar.ranges[0:offset], self.lidar.ranges[359-offset:359])),
            'left' : min(self.lidar.ranges[90-offset:90+offset])
        }


    def publishData(self, vel_msg):
        self.velocity_publisher.publish(vel_msg)
        self.rate.sleep()

    def forward(self):
         # Go forward
         # Keep the horizontal distance correct
         a = 0




    def followWall(self):
        j = 0
        vel_msg = Twist()
        for_dist = 10
        desired_dist = 0.6
        gain = 0.5
        error = for_dist
        tolerance = 0.01
        while error > tolerance:
            j = j + 1 
            if (len(self.lidar.ranges) > 0):
                for_dist = self.regions('front')
                print(for_dist)
            vel_msg.linear.x = gain*error
            error = for_dist - desired_dist
            #print(vel_msg)
            self.publishData(vel_msg)
        vel_msg.linear.x = 0
        self.publishData(vel_msg)
    

if __name__ == '__main__':
    try:
        triton = Triton()
        triton.followWall()

    except rospy.ROSInterruptException:
        pass

                      
