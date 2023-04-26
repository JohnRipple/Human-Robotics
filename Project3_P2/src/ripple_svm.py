#!/usr/bin/env python3
# John Ripple
# Human Centered Robotics
# Project 3 Part 2
# 4/28/2023

from sklearn import svm
from sklearn import metrics
import os
import sys
import numpy as np

class createSVM:
    def __init__(self):
        parameters =   [{'C': [0.001, 0.01, 0.1, 1, 10, 100, 1000],
            'kernel': ['rbf', 'linear', 'poly', 'sigmoid'],
            'gamma': ['auto', 1, 0.1, 0.02, 0.001]}]
        self.support_vec_rad = svm.SVC(kernel="rbf", C=0.6, gamma=1)
        self.support_vec_hjpd = svm.SVC(kernel="rbf", C=1.0, gamma=0.7)


    def radData(self, data_line, action):
        joint_dist = [] # Structured as [[joint 1 dist], [joint 2 dist]]
        joint_angle = [] # Structured as [[joint 1 angle], [joint 2 angle]]
        j = 0
        for i in range(0, int(len(data_line)/2), 19):
            joint_dist.append([])
            joint_dist[j] = list(map(float, data_line[i:i+19]))
            j += 1
        j = 0
        for i in range(int(len(data_line) / 2), len(data_line), 19):
            joint_angle.append([])
            joint_angle[j] = list(map(float, data_line[i:i+19]))
            j += 1

    def hjpdData(self, data_line):
        hjpd_x = []
        hjpd_y = []
        hjpd_z = []
        j = 0
        for i in range(0, len(data_line), 19*3):
            hjpd_x.append([])
            hjpd_y.append([])
            hjpd_z.append([])
            hjpd_x[j] = list(map(float, data_line[i:i+19]))
            i += 19
            hjpd_y[j] = list(map(float, data_line[i:i+19]))
            i += 19
            hjpd_z[j] = list(map(float, data_line[i:i+19]))
            j += 1

    def mainLoop(self):
        path = os.path.join((__file__).replace("ripple_svm.py", ""), os.pardir, "saved_files")
        file = ["rad_d1_train.txt", "rad_d1_test.txt", "cust_d1_train.txt", "cust_d1_test.txt"]
        for filename in file:
            with open(os.path.join(path, filename), 'r') as f:
                data = []
                action = []
                for i, line in enumerate(f):
                    data.append([])
                    action.append([])
                    data_line = line.strip("\r\n").split(" ")
                    action[i] = int( data_line[0][1:])
                    data[i] = list(map(float, data_line[1:]))

                if filename is file[0]:
                    # self.radSVM(data, action)
                    self.support_vec_rad.fit(data, np.array(action).T)
                elif filename is file[1]:
                    print("RAD Accuracy:", self.support_vec_rad.score(data, np.array(action).T))
                    #self.radSVMTest(data, action)
                elif filename is file[2]:
                    self.support_vec_hjpd.fit(data, np.array(action).T)
                elif filename is file[3]:
                    print("HJPD Accuracy:", self.support_vec_hjpd.score(data, np.array(action).T))



        # for filename in os.listdir(path):
        #     print(filename)
        #     with open(os.path.join(path, filename), 'r') as f:
        #         if "rad" in filename:
        #             self.radSVM(f)
                # for i, line in enumerate(f):
                #     data_line = line.strip("\r\n").split(" ")
                #     action = int( data_line[0][1:])
                #     if "rad" in filename:
                #         createSVM(data_line[1:], action)
                #         #radData(data_line[1:], action)
                #     else:
                #         hjpdData(data_line[1:])


if __name__ == '__main__':
    classifier = createSVM()
    try:
        classifier.mainLoop()
    except KeyboardInterrupt:
        print("\nExiting")
        pass
