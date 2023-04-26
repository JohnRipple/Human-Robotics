#!/usr/bin/env python3
# John Ripple
# Human Centered Robotics
# Project 3 Part 1
# 4/11/2023


import os
import sys
import numpy as np
from matplotlib import pyplot as plt


class Skeleton:

    def __init__(self):
        self.distance = []  # RAD distances between hip joint and extremities
        self.angle = []     # RAD angle between hip to extremeties vectors
        self.data = []      # Read in line from a file
        self.writeData = "" # Data to write to the rad_d1_XX.txt file
        self.nan = False    # NaN flag for a frame
        self.hjpd_vec_x = []    # The x distances calculated for every joint in each frame
        self.hjpd_vec_y = []
        self.hjpd_vec_z = []
        self.hjpd_data = []
        self.hjpdWriteData = "" # Data to write to the cust_d1_xx.txt file

        # Read in the parameter from the roslaunch file
        if len(sys.argv) > 1:
            if sys.argv[1] == "test":
                self.train = False
        else:
            self.train = True
        
    def showHist(self, hist, bins):
        '''Display a histogram calulated with numpy.histogram'''
        width = 0.7 * (bins[1] - bins[0])
        center = (bins[:-1] + bins[1:]) / 2
        plt.bar(center, hist, align='center', width=width)
        plt.show()
        
        
    def calcHistogram(self):
        '''Create Histogram based on data for RAD algorithim'''
        dist = np.array(self.distance).T    # Transpose array to get each row as a set of distances from hip center to one extremity
        angle = np.array(self.angle).T
        
        # print(dist)
        for i in range(0, len(dist)):
            # distHist, bins_dist = np.histogram(dist[i], bins=20)
            distHist, bins_dist = np.histogram(dist[i], bins=np.linspace(min(dist[i]), max(dist[i]), num = 20))

            distHist = distHist / float(len(dist[i]))
            if i != 0:
                self.writeData += ' '
            self.writeData += ' '.join(map(str, distHist))
            # self.showHist(distHist, bins_dist)
        self.writeData += ' '

        for i in range(0, len(angle)):
            # angleHist, bins_angle = np.histogram(angle[i], bins=20)
            angleHist, bins_angle = np.histogram(angle[i], bins=np.linspace(min(angle[i]), max(angle[i]), num = 20))

            angleHist = angleHist / float(len(angle[i]))
            if i != 0:
                self.writeData += ' '
            self.writeData += ' '.join(map(str, angleHist))
            # self.showHist(angleHist, bins_angle)
        self.writeData += '\n'
        #print(self.writeData)
        
    
    def calHJPDHist(self):
        '''Calculate the histograms for the x, y, z distances for every joint for all frames in a file'''
        data_x = np.array(self.hjpd_vec_x).T
        data_y = np.array(self.hjpd_vec_y).T
        data_z = np.array(self.hjpd_vec_z).T
        for i in range(0, len(data_x)):
            hist_x, bins_x = np.histogram(data_x[i], bins=np.linspace(min(data_x[i]), max(data_x[i]), num = 20))
            hist_y, bins_y = np.histogram(data_y[i], bins=np.linspace(min(data_y[i]), max(data_y[i]), num = 20))
            hist_z, bins_z = np.histogram(data_z[i], bins=np.linspace(min(data_z[i]), max(data_z[i]), num = 20))
            hist_x = hist_x / float(len(data_x[i]))
            hist_y = hist_y / float(len(data_y[i]))
            hist_z = hist_z / float(len(data_z[i]))
            # self.showHist(hist_x, bins_x) 
            if i != 0:
                self.hjpdWriteData += ' '
            self.hjpdWriteData += ' '.join(map(str, hist_x)) + ' ' + ' '.join(map(str, hist_y)) + ' ' + ' '.join(map(str, hist_z))
        self.hjpdWriteData += '\n'


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
        
    def stripData(self, line):
        '''Converts read in line into list of floats while flagging Nan frames'''
        line = line.strip("\r\n").split(" ")
        if '' in line:
            # Removes extra whitespaces in data in case the line had two spaces next to each other
            try:
                while True:
                    line.remove('')
            except ValueError:
                pass
        for i in range(0, len(line)):
            if line[i] == 'NaN':
                self.nan = True
            else:
                line[i] = float(line[i])
        return line


    def calcJointDisplace(self):
        '''Find the distance between the hip center joint and all other joints'''
        ref_joint = 1   # Position 0 in self.data
        hjpd_vec_x = [] # All the x distances for every joint in a frame
        hjpd_vec_y = []
        hjpd_vec_z = []
        # print(self.hjpd_data)
        for i in range(1, len(self.hjpd_data)):
            hjpd_vec_x.append(self.hjpd_data[i][2] - self.hjpd_data[0][2])
            hjpd_vec_y.append(self.hjpd_data[i][3] - self.hjpd_data[0][3])
            hjpd_vec_z.append(self.hjpd_data[i][4] - self.hjpd_data[0][4])
        self.hjpd_vec_x.append(hjpd_vec_x)
        self.hjpd_vec_y.append(hjpd_vec_y)
        self.hjpd_vec_z.append(hjpd_vec_z)
        # hjpd_vec = [Frame 1: [joint1, joint2, joint3... ],
        # Frame 2: [joint1, joint2, joint3...]
        # ...]


    def hjpd(self, line):
        '''Find the distance from hip center to all joints for the HJPD algorithim'''
        data = self.stripData(line)
        self.hjpd_data.append(data) # Append all joint data
        if data[1] == 20:
            if self.nan:
                self.nan = False    # Don't add the Nan frame
                print(self.hjpd_data)
            else:
                self.calcJointDisplace()    # Calc the distnace between joints
            self.hjpd_data = []


    def findStar(self, line):
        '''Find the star joint positions for the RAD algorithim'''
        desired_joints = [1, 4, 8, 12, 16, 20]  # Desired joint positions (hands, feet, head, torso)
        data = self.stripData(line)
        # print(data)
        if data[1] in desired_joints:
            self.data.append(data)  # Add joint data only if is one of the desired joints
        if data[1] == 20:
            # Don't add this frame of data to the histograms if it has NaNs in the frame
            if not self.nan:
                self.calcDistance() # Calculate distance between joints
                self.calcAngle()    # Calculate angle between joints
            self.data = []
    
    
    def mainLoop(self, dataset_path):
        # if self.train:
        #     dataset_path = "train"
        # else:
        #     dataset_path = "test"
        save_path = os.path.join((__file__).replace("ripple_skeleton.py", ""), os.pardir, "saved_files/", "rad_d1_" + dataset_path + ".txt")
        save_path_cust = os.path.join((__file__).replace("ripple_skeleton.py", ""), os.pardir,"saved_files/","cust_d1_" + dataset_path +".txt")
        path = os.path.join((__file__).replace("ripple_skeleton.py", ""), os.pardir, "dataset/", dataset_path)
        print(save_path)
        open(save_path, 'w').close()    # Clear contents of the file
        save_file = open(save_path, 'w')
        open(save_path_cust, 'w').close()    # Clear contents of the file
        save_file_cust = open(save_path_cust, 'w')
        for filename in os.listdir(path):
            print(filename)
            self.distance = []  # Reset distance and angle for new file
            self.angle = []
            self.writeData = ""
            self.hjpdWriteData = ""
            with open(os.path.join(path, filename), 'r') as f:
                for i, line in enumerate(f):
                    self.findStar(line) # RAD algorithim
                    self.hjpd(line)     # HJPD algorithim
                self.calcHistogram()    # Calculate the RAD histogram
                self.calHJPDHist()      # Calculate the HJPD histogram
                # Format of 'action joint1Dist joint2Dist... jointNDist joint1Angle joint2Angle... jointNAngle
                save_file.write(filename[0:3] + ' ' + self.writeData)
                # Format of 'action joint1X joint1Y joint1Z... jointNX jointNY jointNZ
                save_file_cust.write(filename[0:3] + ' ' + self.hjpdWriteData)
        save_file.close()
        save_file_cust.close()

    

if __name__ == '__main__':
    skeleton = Skeleton()
    try:
        skeleton.mainLoop("train")
        skeleton.mainLoop('test')
    except KeyboardInterrupt:
        print("\nExiting")
        pass

                      