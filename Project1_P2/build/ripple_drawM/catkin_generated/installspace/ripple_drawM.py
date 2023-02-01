#!/usr/bin/env python2
# John Ripple
# Human Centered Robotics
# Project 1 Part 2
# 1/30/2023

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

pose = Pose()   # global pose variable


def callback(data):
    global pose
    pose = data # set the pose to the subscribed data of /turtlesim/Pose


def ripple_drawM(bound):
    global pose
    new_pose = [pose.x + bound[0], pose.y + bound[1]]   # Use an offset distance from the last moved spot
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) # Publish to the topic /turtle1/cmd_vel
    rospy.init_node('ripple_drawM', anonymous=False)    # Initialize the drawM node
    rate = rospy.Rate(39.8) # 39.8hz Made the M come out fairly well
    msg = Twist()   # Use Twist data type for published data
    stopx, stopy, j, boundx, boundy = 0, 0, 0, 0, 0
    
    while ((stopx == 0 or stopy == 0)):
        sign = 1
        listener()  # Get the current pose of the turtle

        # Normalize the offset values to use for speed values
        if (bound[1] != 0):
            boundy = abs(bound[1] / bound[1])
        else: 
            boundx = abs(bound[0] / bound[0])
        if (bound[0] != 0 and bound[1] != 0):
            boundx = abs(bound[0]/ bound[1])

        # Check a boundary around the new pose to see if the turtle has reached the location yet
        if ((pose.y > new_pose[1] + 0.1 or pose.y < new_pose[1] - 0.1) and bound[1] != 0 ):
            if bound[1] < 0:
                sign = -1
            msg.linear.y = boundy * 3 * sign
        else:
            stopy = 1
        sign = 1
        if ((pose.x > new_pose[0] + 0.1 or pose.x < new_pose[0] - 0.1) and bound[0] != 0):
            if bound[0] < 0:
                sign = -1
            msg.linear.x = boundx * 3 * sign
        else:
            stopx = 1
        #rospy.loginfo(pose)
        pub.publish(msg)    # Publish the x,y speed information
        rate.sleep()
        
    # Set the turtle speed to 0 after it finishes moving
    listener()
    msg.linear.x = 0
    msg.linear.y = 0
    rospy.loginfo(pose)
    pub.publish(msg)
    rate.sleep()


# create the subscriber to listen to the /turtle1/pose information
def listener():
    sub = rospy.Subscriber("/turtle1/pose", Pose , callback)

if __name__ == '__main__':
    try:
        bounds = [[5.0, 8.0], [0, -3], [-0.4, 0], [0, -1], [2, 0], [0, 1], [-0.4, 0], [0, 6], [0.4, 0], [0, 1], [-2, 0], [-1.875, -3.0]]    # x,y bounds 0 means it doesn't change
        
        # Draw half the M
        for i in bounds:
            ripple_drawM(i)

        # Draw the mirrored half of the M
        for i in bounds[::-1]:
            ripple_drawM([i[0], -1 * i[1]])
        ripple_drawM([0.63, 1]) # Close a small gap where both halfs of the M meet

    except rospy.ROSInterruptException:
        pass