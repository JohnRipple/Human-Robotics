#!/usr/bin/env python
# John Ripple
# Human Centered Robotics
# Project 1 Part 2
# 1/30/2023

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

pose = Pose()   # global pose variable


def callback(data):
    global pose
    pose = data # set the pose to the subscribed data of /turtlesim/Pose


def ripple_drawM(bound):
    global pose
    
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) # Publish to the topic /turtle1/cmd_vel
    rospy.init_node('ripple_drawM', anonymous=False)    # Initialize the drawM node
    rate = rospy.Rate(100)
    msg = Twist()   # Use Twist data type for published data

    msg.linear.x = 0
    msg.linear.y = 0
    pub.publish(msg)
    listener()
    rospy.loginfo(pose)

    new_pose = [pose.x + bound[0], pose.y + bound[1]]   # Use an offset distance from the last moved spot

    stop, boundx, boundy = 0, 0, 0
    epsilon = 0.02
    d = math.sqrt((new_pose[0] - pose.x)**2+(new_pose[1] - pose.y)**2)  # distance 
    dprev = d
    while (d > epsilon and dprev >= d):
        sign = 1
        listener()  # Get the current pose of the turtle
        dprev = d
        d = math.sqrt((new_pose[0] - pose.x)**2+(new_pose[1] - pose.y)**2)
        
        # Normalize the offset values to use for speed values
        if (bound[1] != 0):
            boundy = abs(bound[1] / bound[1])
        else: 
            boundx = abs(bound[0] / bound[0])
        if (bound[0] != 0 and bound[1] != 0):
            boundx = abs(bound[0]/ bound[1])

        if bound[1] < 0:
            sign = -1
        msg.linear.y = boundy * 5 * sign
        sign = 1
        if bound[0] < 0:
            sign = -1
        msg.linear.x = boundx * 5 * sign

        print("d: %.3f\tdprev: %.3f" % (d, dprev))
        # rospy.loginfo(pose)
        pub.publish(msg)    # Publish the x,y speed information
        rate.sleep()
        
    # Set the turtle speed to 0 after it finishes moving
    listener()
    msg.linear.x = 0
    msg.linear.y = 0
    pub.publish(msg)
    rate.sleep()


# create the subscriber to listen to the /turtle1/pose information
def listener():
    sub = rospy.Subscriber("/turtle1/pose", Pose , callback)

if __name__ == '__main__':
    try:
        bounds = [[3.0*5.0/8.0, 3.0], [0, -3], [-0.4, 0], [0, -1], [2, 0], [0, 1], [-0.4, 0], [0, 4], [0.4, 0], [0, 1], [-2, 0], [-3.0*5.0/8.0, -3.0]]    # x,y bounds 0 means it doesn't change
        #bounds = [[5.54+1.9, 5.54+3.0]]
        # Draw half the M
        for i in bounds:
            listener()
            ripple_drawM(i)

        # Draw the mirrored half of the M
        for i in bounds[::-1]:
            ripple_drawM([i[0], -1 * i[1]])
        ripple_drawM([0.4, -0.4*8.0/5.0])
        ripple_drawM([0.4, 0.4*8.0/5.0]) # Close a small gap where both halfs of the M meet

    except rospy.ROSInterruptException:
        pass