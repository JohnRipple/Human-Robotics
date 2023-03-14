#!/usr/bin/env python
# John Ripple
# Human Centered Robotics
# Project 2 Part 2
# 3/8/2023
# With reference to http://wiki.ros.org/turtlesim/Tutorials/Go%20to%20Goal
# https://gitlab.com/HCRLab/stingray-robotics/Stingray-Simulation


import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from itertools import chain
import pickle
import math
import random


class Triton:

    def __init__(self):
            """Initializes node, publisher, and subscriber when object is created"""
            rospy.init_node('ripple_wallFollowing', anonymous=False)    # Initialize the drawM node
            self.velocity_publisher = rospy.Publisher('/triton_lidar/vel_cmd', Twist, queue_size=10) # Publish to the topic
            self.lidar_subscriber = rospy.Subscriber("/scan", LaserScan , self.update_scan) # Read LiDar data
            self.time = 0
            self.lidar = LaserScan()
            self.action = 0
            self.actionDesc = ''
            self.desired_dist = 0.6
            self.state = 0
            self.prevState = self.state
            self.reward = 0
            self.alpha = 0.1
            self.gamma = .8
            self.regions = {
                 'right' : 0,
                 'front' : 0,
                 'left' : 0,
                 'rear' : 0
            }
            self.rate = rospy.Rate(50) # in hz
            self.q = [[10, 0, 0],    # State 0: No wall at any sensor
                        [0, 1, 0],    # State 1: Front
                        [0, 0, 1],    # State 2: Right
                        [0, 1, 0],    # State 3: Left
                        [0, 1, 0],    # State 4: Front and Right
                        [0, 1, 0],    # State 5: Front and Left
                        [0, 1, 0],    # State 6: Front, Left, and Right
                        [0, 0, 1]]    # State 7: Left and Right
            readQTable = True
            try:
                if readQTable:
                    pkl_file = open('/home/john/Human-Robotics/Project2_P3/src/ripple_wallFollowing/src/qTable.pkl', 'rb')
                    self.q = pickle.load(pkl_file)
                    pkl_file.close()
                    print("Read in Q Table")
                else:
                    print("Using default Q Table")
            except:
                print("Could not read Q Table, using default") 
                



    def update_scan(self, data):
        """Callback function when the Pose message type is recieved by the subscriber"""
        # angle_min = 0
        # angle_max = 360
        # len(ranges) = 360   one datapoint per degree
        self.lidar = data
        offset_forward = 10
        offset = 45
        right = 270
        left = 90
        rear = 180
        self.regions = {
            'right' : min(self.lidar.ranges[right-offset:right+offset*3]),
            'front' : min(chain(self.lidar.ranges[0:offset_forward], self.lidar.ranges[359-offset_forward:359])),
            'left' : min(self.lidar.ranges[left-offset:left+offset]),
            'rear' : min(self.lidar.ranges[rear-offset:rear+offset])
        }
        self.chooseState()


    def publishData(self, vel_msg):
        """Publish the velocity data to \cmd_vel"""
        self.velocity_publisher.publish(vel_msg)
        self.rate.sleep()


    def findWall(self):
        """Find a wall by going forward only for the first state and then go forward and slightly turn"""
        msg = Twist()

        msg.linear.x = 0.3
        msg.angular.z = -0.3
        if self.regions['rear'] < self.desired_dist*1.5:
            msg.linear.x = 0.05
            msg.angular.z = -3
        return msg


    def turnRight(self):
        """Turn right with a constant angular velocity"""
        msg = Twist()
        msg.angular.z = -0.3
        return msg
    

    def turnLeft(self):
        """Turn left with a constant angular velocity"""
        msg = Twist()
        msg.linear.x = 0.01
        msg.angular.z = 0.5
        return msg


    def followWall(self):
        """Follow a wall on the right by first using a P controller to get distance correct and then a P controller to stay even"""
        msg = Twist()
        gain = 3
        offset = 20
        msg.angular.z = max(min(0.5* (self.lidar.ranges[270-offset] - self.lidar.ranges[270+offset]), 2.0), -2.0)
        #msg.angular.z = ( (self.desired_dist - 0.3 - self.regions['right']) + 0.5* (self.lidar.ranges[270-offset] - self.lidar.ranges[270+offset]))
        #if abs(self.lidar.ranges[270-offset] - self.lidar.ranges[270+offset]) < 0.2:
        if self.regions['right'] < self.desired_dist*1.5:
            msg.linear.y = max(min(gain * (self.desired_dist - 0.3 - self.regions['right']), 0.3), -0.3)
        msg.linear.x = 0.22
        return msg


    def changeAction(self, state):
        '''Update the action if it has changed'''
        self.state = state
        action = self.q[state]
        epsilon = self.exp(0.1, 10*60, rospy.get_time())
        p = random.random()
        p = 5
        if p < epsilon:
            action = random.randint(0,2)    # Choose random action
        else:
            action = action.index(max(action))
        if action is not self.action or self.prevState != self.state:
            print(self.actionDesc)
            self.action = action
            self.prevState = self.state


    def chooseState(self):
        '''Change state based on sensor readings. This is the Q table'''
        if self.regions['front'] > self.desired_dist and self.regions['left'] > self.desired_dist and self.regions['right'] > self.desired_dist:
            self.actionDesc = 'Case 0: Nothing'
            # if self.regions['rear'] < self.desired_dist:
            #     self.changeAction(3)
            # else:
            self.changeAction(0) # Default Action 0
        elif self.regions['front'] < self.desired_dist and self.regions['left'] > self.desired_dist and self.regions['right'] > self.desired_dist:
            self.actionDesc = "Case 1: Front"
            self.changeAction(1)    # Default Action 1
        elif self.regions['front'] > self.desired_dist and self.regions['left'] > self.desired_dist and self.regions['right'] < self.desired_dist:
            self.actionDesc = "Case 2: Right"
            self.changeAction(2)    # Default Action 2
        elif self.regions['front'] > self.desired_dist and self.regions['left'] < self.desired_dist and self.regions['right'] > self.desired_dist:
            self.actionDesc = "Case 3: Left"
            self.changeAction(3)    # Default Action 1 
        elif self.regions['front'] < self.desired_dist and self.regions['left'] > self.desired_dist and self.regions['right'] < self.desired_dist:
            self.actionDesc = "Case 4: Front and Right"
            self.changeAction(4)    # Default Action 1
        elif self.regions['front'] < self.desired_dist and self.regions['left'] < self.desired_dist and self.regions['right'] > self.desired_dist:
            self.actionDesc = "Case 5: Front and Left"
            self.changeAction(5)    # Default Action 1
        elif self.regions['front'] < self.desired_dist and self.regions['left'] < self.desired_dist and self.regions['right'] < self.desired_dist:
            self.actionDesc = "Case 6: Front, Left, and Right"
            self.changeAction(6)    # Default Action 1
        elif self.regions['front'] > self.desired_dist and self.regions['left'] < self.desired_dist and self.regions['right'] < self.desired_dist:
            self.actionDesc = "Case 7: Left and Right"
            self.changeAction(7)    # Default Action 2
        else:
            self.actionDesc = "Unknown Case"

    def printLidar(self):
        """Print Lidar Data for three directions front, left and right"""
        if (len(self.lidar.ranges) > 0):
            print("Forward: %.5f\tRight: %.5f\tLeft: %.5f\tRear: %.5f" % (self.regions['front'], self.regions['right'], self.regions['left'], self.regions['rear']))
            
    
    def exp(self, y_desired, x_desired, x, offset=0, sign=1):
        """Creates exponential function based from 0. Get desired y value at desired x value then pass in current x value and an offset if you want it centered around something other than 0."""
        gamma = math.log(y_desired, 10)/((x_desired-offset))
        return 10**(sign*gamma*(x-offset))


    def rewardFunc(self, y_desired, x_desired, x, offset=0):
        """Reward function based on exponential function away from desired measurement"""
        if self.regions['right'] < .13 or self.regions['front'] < .13 or self.regions['left'] < .13:
            # Penalize for being too close to the wall
            return -5
        if x > offset:
            return self.exp(y_desired,x_desired, x, offset)
        else:
            return self.exp(y_desired,x_desired, x, offset, -1)


    def mainLoop(self):
        """Main Loop for the robot to execute actions"""
        vel_msg = Twist()
        while not rospy.is_shutdown():
            if self.action == 0:
                self.reward = (self.rewardFunc(0.01, 2, self.regions['right'], self.desired_dist) + self.rewardFunc(0.01, 2, self.regions['front'], self.desired_dist) + self.rewardFunc(0.01, 2, self.regions['left'], self.desired_dist))
                vel_msg = self.findWall()
                #print(self.q[self.state][self.action])
            elif self.action == 1:
                self.reward = self.rewardFunc(0.01, 1, self.regions['right'], self.desired_dist)
                vel_msg = self.turnLeft()
            elif self.action == 2:
                # Average reward of being the correct distance from the wall and being straight
                self.reward = (self.rewardFunc(0.01, 1, self.regions['right'], self.desired_dist - 0.3) + self.rewardFunc(0.01, 1, self.lidar.ranges[270-20] - self.lidar.ranges[270+20])) / 2.0
                vel_msg = self.followWall()
            else:
                print("Naughty robot found an edge case")
            
            self.q[self.state][self.action] = self.q[self.state][self.action] + self.alpha*(self.reward+self.gamma*max(self.q[self.state]) - self.q[self.state][self.action])
            self.printLidar()
            self.publishData(vel_msg)
            self.rate.sleep()
        # Stop the robot from moving
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.publishData(vel_msg)
        self.rate.sleep()
    

if __name__ == '__main__':
    triton = Triton()
    try:
        triton.mainLoop()
        

    except rospy.ROSInterruptException:
        print("\nExiting\nQ Table:")
        print(triton.q)
        output = open('/home/john/Human-Robotics/Project2_P3/src/ripple_wallFollowing/src/qTable.pkl', 'wb')
        pickle.dump(triton.q, output)
        output.close()
        pass

                      
