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
from simple_pid import PID

class Triton:

    def __init__(self):
            """Initializes node, publisher, and subscriber when object is created"""
            rospy.init_node('ripple_drawM', anonymous=False)    # Initialize the drawM node
            self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10) # Publish to the topic /turtle1/cmd_vel
            self.lidar_subscriber = rospy.Subscriber("/scan", LaserScan , self.update_scan)
            self.lidar = LaserScan()
            self.state = 5
            self.stateDesc = ''
            self.desired_dist = 1
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
        offset = 10
        right = 270
        left = 90
        self.regions = {
            'right' : min(self.lidar.ranges[right-offset:right+offset]),
            'front' : min(chain(self.lidar.ranges[0:offset], self.lidar.ranges[359-offset:359])),
            'left' : min(self.lidar.ranges[left-offset:left+offset])
        }
        self.action()


    def publishData(self, vel_msg):
        self.velocity_publisher.publish(vel_msg)
        self.rate.sleep()


    def findWall(self):
        msg = Twist()
        msg.linear.x = 0.22
        if self.state is not 5:
            msg.angular.z = -0.3
        else:
            msg.angular.z = 0
        return msg


    def turnRight(self):
        msg = Twist()
        msg.angular.z = -0.3
        return msg
    

    def turnLeft(self):
        msg = Twist()
        msg.angular.z = 0.22
        return msg


    def followWall(self):
        msg = Twist()

        if self.lidar.ranges[300] > self.lidar.ranges[240]:
            msg.angular.z = -1 * abs(self.lidar.ranges[300] - self.lidar.ranges[240]) 
        else:
            msg.angular.z = 1 * abs(self.lidar.ranges[300] - self.lidar.ranges[240])
        # print(self.lidar.ranges[300] - self.lidar.ranges[240])

        if abs(self.regions['right'] - 0.3) > 0.02:
            msg.angular.z = 0.5 * (0.3 - self.regions['right'])
        msg.linear.x = 0.22
        return msg


    def changeState(self, state):
        if state is not self.state:
            print(self.stateDesc)
            self.state = state

    def action(self):
        desired_dist = self.desired_dist

        if self.regions['front'] > desired_dist and self.regions['left'] > desired_dist and self.regions['right'] > desired_dist:
            if self.state is not 5:
                self.stateDesc = 'Case 0: Nothing'
                self.changeState(0)
        elif self.regions['front'] < desired_dist and self.regions['left'] > desired_dist and self.regions['right'] > desired_dist:
            self.stateDesc = "Case 1: Front"
            self.changeState(1)
        elif self.regions['front'] > desired_dist and self.regions['left'] > desired_dist and self.regions['right'] < desired_dist:
            self.stateDesc = "Case 2: Right"
            self.changeState(3)
        elif self.regions['front'] > desired_dist and self.regions['left'] < desired_dist and self.regions['right'] > desired_dist:
            self.stateDesc = "Case 3: Left"
            self.changeState(0) 
        elif self.regions['front'] < desired_dist and self.regions['left'] > desired_dist and self.regions['right'] < desired_dist:
            self.stateDesc = "Case 4: Front and Right"
            self.changeState(1)
        elif self.regions['front'] < desired_dist and self.regions['left'] < desired_dist and self.regions['right'] > desired_dist:
            self.stateDesc = "Case 5: Front and Left"
            self.changeState(1)
        elif self.regions['front'] < desired_dist and self.regions['left'] < desired_dist and self.regions['right'] < desired_dist:
            self.stateDesc = "Case 6: Front, Left, and Right"
            self.changeState(1)
        elif self.regions['front'] > desired_dist and self.regions['left'] < desired_dist and self.regions['right'] < desired_dist:
            self.stateDesc = "Case 7: Left and Right"
            self.changeState(0)
        else:
            self.stateDesc = "Unknown Case"    


    def printLidar(self):
        if (len(self.lidar.ranges) > 0):
            print("Forward: %.5f\tRight: %.5f\tLeft: %.5f" % (self.regions['front'], self.regions['right'], self.regions['left']))


    def mainLoop(self):
        vel_msg = Twist()
        error = 1
        tolerance = 0.001
        right_last = self.regions['right']
        error_right = 1
        while not rospy.is_shutdown():
            if self.state == 0:
                vel_msg = self.findWall()
            elif self.state == 1:
                vel_msg = self.turnLeft()
            elif self.state == 2:
                vel_msg = self.turnRight()
            elif self.state == 3:
                vel_msg = self.followWall()
                # vel_msg.angular.z = 3 * (0.3 - self.regions['right']) + (self.regions['right'] - right_last)*30
            elif self.state == 5:
                vel_msg = self.findWall()
            else:
                print("Naughty robot found an edge case")
            right_last = self.regions['right']
            # self.printLidar()
            self.publishData(vel_msg)
            self.rate.sleep()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.publishData(vel_msg)
        self.rate.sleep()
    

if __name__ == '__main__':
    try:
        triton = Triton()
        triton.mainLoop()

    except rospy.ROSInterruptException:
        pass

                      
