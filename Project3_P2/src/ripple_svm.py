#!/usr/bin/env python3
# John Ripple
# Human Centered Robotics
# Project 3 Part 2
# 4/28/2023

from sklearn import svm
import os
import sys
import numpy as np



def createSVM(dist, angle):
    support_vector = sv


def radData(data_line):
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

def hjpdData(data_line):
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

def main():
    path = os.path.join((__file__).replace("ripple_svm.py", ""), os.pardir, "saved_files")
    for filename in os.listdir(path):
        print(filename)
        with open(os.path.join(path, filename), 'r') as f:
            for i, line in enumerate(f):
                data_line = line.strip("\r\n").split(" ")
                action = data_line[0]
                if "rad" in filename:
                    radData(data_line[1:])
                else:
                    hjpdData(data_line[1:])


main()