#!/usr/bin/env python
# John Ripple
# Human Centered Robotics
# Project 3 Part 1
# 4/11/2023


import rospy
import os
import sys
import numpy as np
from matplotlib import pyplot as plt


class Skeleton:

    def __init__(self):
        self.distance = []
        self.angle = []
        self.data = []
        self.writeData = ""
        
    def showHist(self, hist, bins):
        width = 0.7 * (bins[1] - bins[0])
        center = (bins[:-1] + bins[1:]) / 2
        plt.bar(center, hist, align='center', width=width)
        plt.show()
        
        
    def calcHistogram(self):
        '''Create Histogram based on data'''
        
        dist = np.array(self.distance).T    # Transpose array to get each row as a set of distances from hip center to one extremity
        angle = np.array(self.angle).T
        
        # print(dist)
        for i in range(0, len(dist)):
            distHist, bins_dist = np.histogram(dist[i], bins=np.linspace(min(dist[i]), max(dist[i]), num = 20))
            distHist = distHist / float(len(dist[i]))
            self.writeData += ' '.join(map(str, distHist))
            # print(distHist)
        self.writeData += ' '
        for i in range(0, len(angle)):
            angleHist, bins_angle = np.histogram(angle[i], bins=np.linspace(min(angle[i]), max(angle[i]), num = 20))
            angleHist = angleHist / float(len(angle[i]))
            self.writeData += ' '.join(map(str, angleHist))
            # self.showHist(angleHist, bins_angle)
        self.writeData += '\n'
        #print(self.writeData)
        
        
    def unit_vector(self, vector):
        """ Returns the unit vector of the vector.  """
        if np.linalg.norm(vector):
            return vector / np.linalg.norm(vector)
        return [0, 0, 0]


    def angle_between(self, v1, v2):
        """ Returns the angle in radians between vectors 'v1' and 'v2'::

                >>> angle_between((1, 0, 0), (0, 1, 0))
                1.5707963267948966
                >>> angle_between((1, 0, 0), (1, 0, 0))
                0.0
                >>> angle_between((1, 0, 0), (-1, 0, 0))
                3.141592653589793
        """
        v1_u = self.unit_vector(v1)
        v2_u = self.unit_vector(v2)
        return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


    def calcAngle(self):
        '''Find the angle between the vectors from hip center to extremities for the RAD algorithim'''
        angle = []
        for i in range(1, len(self.data)):
            v1 = [self.data[i][2] - self.data[0][2], self.data[i][3] - self.data[0][3], self.data[i][4] - self.data[0][4]]
            j = i+1
            if j >= len(self.data):
                j = 1
            v2 = [self.data[j][2] - self.data[0][2], self.data[j][3] - self.data[0][3], self.data[j][4] - self.data[0][4]]
            angle.insert(i, self.angle_between(v1, v2))
        self.angle.append(angle)
        # print(self.angle)
        # self.angle = []
        
        
    def calcDistance(self):
        '''Find the 3D distance between the hip center and the extremity points for the RAD algorithim'''
        dist = []
        for i in range(1, len(self.data)):
            dist.insert(i, ((self.data[0][2] - self.data[i][2])**2 + (self.data[0][3] - self.data[i][3])**2 + (self.data[0][4] - self.data[i][4])**2)**0.5)
        self.distance.append(dist)
        # print(self.distance)
        # self.distance = []
        
        
    def findStar(self, line):
        '''Find the star joint positions for the RAD algorithim'''
        desired_joints = [1, 4, 8, 12, 16, 20]  # Desired joint positions (hands, feet, head, hip center)
        data = line.strip("\r\n").split(" ")
        for i in range(0, len(data)):
            if data[i] == 'NaN':
                data[i] = 0.0
            try:
                data[i] = float(data[i])
            except:
                data[i] = 0.0
        # print(data)
        if data[1] in desired_joints:
            self.data.append(data)
        if data[1] == 20:
            # print(self.data)
            self.calcDistance()
            self.calcAngle()
            self.data = []
            self.frame = data[0]
    
    
    def mainLoop(self):
        train = "train"
        path = os.path.join(os.getcwd(), "src/ripple_skeleton/dataset/", train)
        save_path = "src/ripple_skeleton/saved_files/rad_d1_train.txt"
        open(save_path, 'w').close()    # Clear contents of the file
        save_file = open(save_path, 'w')
        for filename in os.listdir(path):
            print(filename)
            self.distance = []  # Reset distance and angle for new file
            self.angle = []
            self.writeData = ""
            with open(os.path.join(path, filename), 'r') as f:
                self.frame = 1
                for i, line in enumerate(f):
                    self.findStar(line)
                
                self.calcHistogram()
                save_file.write(self.writeData)
                
        save_file.close()

    

if __name__ == '__main__':
    skeleton = Skeleton()
    try:
        skeleton.mainLoop()
    except rospy.ROSInterruptException:
        print("\nExiting")
        pass

                      