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
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from sensor_msgs.msg import LaserScan
from itertools import chain
import pickle
import math
import random
import os
import sys
import numpy as np


class Triton:

    def __init__(self):
            """Initializes node, publisher, and subscriber when object is created"""
            rospy.init_node('ripple_wallFollowing', anonymous=False)    # Initialize the drawM node
            self.velocity_publisher = rospy.Publisher('/triton_lidar/vel_cmd', Twist, queue_size=10) # Publish to the topic
            self.lidar_subscriber = rospy.Subscriber("/scan", LaserScan , self.update_scan) # Read LiDar data
            self.time = 0
            self.wallTime = 0
            self.lidar = LaserScan()
            self.action = 0
            self.actionDesc = ''
            self.desired_dist = 0.6
            self.state = 0
            self.prevState = self.state
            self.reward = 0
            self.alpha = 0.1
            self.gamma = .8
            self.rightIndex = 270
            self.episode = 0
            self.regions = {
                 'right' : 0,
                 'front' : 0,
                 'left' : 0,
                 'rear' : 0
            }
            self.rate = rospy.Rate(50) # in hz
            self.q = [[2.9808565844326087, 0.7150296469232191, 0.6852970670094203],    # State 0: No wall at any sensor
                        [0, 1.0960474544165195, 0],   # State 1: Front
                        [-2.868780580472797, 1.5763846853311454, 4.0283334397623145],    # State 2: Right
                        [5.2005340807832585, 1.236819794267857, 1.1849904571027294],    # State 3: Left
                        [-14.659073051572154, 0.2164383596336809, -14.622830146897051],    # State 4: Front and Right
                        [0, 1, 0],    # State 5: Front and Left
                        [-26.682280438795782, -13.338257303375574, -22.846608172632394],    # State 6: Front, Left, and Right
                        [2.832833138913845, 1.045735425615309, 2.167094291252819]]    # State 7: Left and Right
            try:
                readQTable = rospy.get_param('/ripple_wallFollow/readQTable')
            except:
                readQTable = False
            try:
                self.train = rospy.get_param('/ripple_wallFollow/train')
            except:
                self.train = False
            try:
                if readQTable:
                    # pkl_file = open('/home/john/Human-Robotics/Project2_P3/src/ripple_wallFollowing/src/qTable.pkl', 'rb')
                    pkl_file = open(os.path.join(sys.path[0], "qTable.pkl"), 'rb')
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
            'right' : min(self.lidar.ranges[right-offset*2:right+offset]),
            'front' : min(chain(self.lidar.ranges[0:offset_forward], self.lidar.ranges[359-offset_forward:359])),
            'left' : min(self.lidar.ranges[left-offset:left+offset]),
            'rear' : min(self.lidar.ranges[rear-offset:rear+offset])
        }
        self.rightIndex = self.lidar.ranges.index(self.regions['right'])
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
        return msg


    def turnRight(self):
        """Turn right with a constant angular velocity"""
        msg = Twist()
        msg.angular.z = -0.3
        return msg
    

    def turnLeft(self):
        """Turn left with a constant angular velocity"""
        msg = Twist()
        msg.linear.x = 0.08
        msg.angular.z = 0.5
        return msg


    def followWall(self):
        """Follow a wall on the right by first using a P controller to get distance correct and then a P controller to stay even"""
        msg = Twist()
        gain = 3
        offset = 20
        if self.regions['right'] < self.desired_dist*1.5:
            msg.angular.z = max(min((self.rightIndex-270) * 0.015, 2.0), -2.0)
            msg.linear.y = max(min(gain * (self.desired_dist - 0.3 - self.regions['right']), 0.3), -0.3)
        msg.linear.x = 0.22
        return msg


    def changeAction(self, state):
        '''Update the action if it has changed'''
        self.state = state
        action = self.q[state]
        #epsilon = self.exp(0.1, 10*60, rospy.get_time())
        epsilon = 0.98*0.98**(self.episode/10.0)
        p = random.random()
        # p = 5
        if p < epsilon and self.train:
            action = random.randint(0,2)    # Choose random action
        else:
            action = action.index(max(action))
        if action is not self.action or self.prevState != self.state:
            print("%s\tAction: %d" % (self.actionDesc, action))
            self.action = action
            self.prevState = self.state


    def chooseState(self):
        '''Change state based on sensor readings. This is the Q table'''
        if self.regions['front'] > self.desired_dist and self.regions['left'] > self.desired_dist and self.regions['right'] > self.desired_dist:
            self.actionDesc = 'Case 0: Nothing'
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
            return -1
        if x > offset:
            return self.exp(y_desired,x_desired, x, offset)
        else:
            return self.exp(y_desired,x_desired, x, offset, -1)


    def resetEpisode(self):
        # Restart in a random location and orientation after so much time or touching a wall
        print("Restarting the episode")
        top = [[3.5, 3.5], [1.4, 1.3], [3.5, -0.6], [3.5, -2.5], [3.5, 3.5]]
        bottom = [[-3.5, 2.5], [-3.5, 0.5], [-3.5, -1.5], [-3.5, -3.5], [-2.45, -3.5]]
        region = random.randrange(0, len(top))
        x = random.uniform(top[region][0], bottom[region][0])
        y = random.uniform(top[region][1], bottom[region][1])
        self.time = rospy.get_time()
        self.wallTime = self.time
        state_msg = ModelState()
        state_msg.model_name = 'triton_lidar'
        state_msg.reference_frame = 'world'
        state_msg.pose.position.x = x
        state_msg.pose.position.y = y
        state_msg.pose.position.z = 0
        z = random.uniform(-3.14, 3.14)
        quaternion = self.get_quaternion_from_euler(0, 0, random.uniform(-3.14, 3.14))
        state_msg.pose.orientation.x = quaternion[0]
        state_msg.pose.orientation.y = quaternion[1]
        state_msg.pose.orientation.z = quaternion[2]
        state_msg.pose.orientation.w = quaternion[3]
        self.episode += 1
        print("Epsilon: %.2f" % (0.98*0.98**(self.episode/10.0)))
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg )

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
        
    
    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        
        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.
        
        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return [qx, qy, qz, qw]


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
            
            #self.printLidar()
            
            if self.train:
                self.q[self.state][self.action] = self.q[self.state][self.action] + self.alpha*(self.reward+self.gamma*max(self.q[self.state]) - self.q[self.state][self.action])
                if self.regions['right'] > self.desired_dist:
                    self.wallTime = rospy.get_time()
                if (rospy.get_time() - self.time > 0.5*60) or (self.regions['right'] < self.desired_dist and rospy.get_time() - self.wallTime > 180) or (self.regions['right'] < .13 or self.regions['front'] < .13 or self.regions['left'] < .13 or self.regions['rear'] < .13):
                    self.resetEpisode()
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
        # if raw_input("Save File? y/n: ") == 'y':
        output = open(os.path.join(sys.path[0], "qTable.pkl"), 'wb')
        pickle.dump(triton.q, output)
        output.close()
        pass

                      