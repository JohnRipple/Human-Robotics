#!/usr/bin/env python3
# John Ripple
# Human Centered Robotics
# Project 3 Part 2
# 4/28/2023

from sklearn import svm
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler
from sklearn import metrics
import os
import sys
import numpy as np

class createSVM:
    def __init__(self):
        self.parameters =   {'C': [0.001, 0.01, 0.1, 1, 10, 100, 1000],
            'kernel': ['rbf', 'linear', 'poly', 'sigmoid'],
            'gamma': ['auto', 1, 0.1, 0.02, 0.001]}
        self.support_vec_rad = svm.SVC(kernel="rbf", C=0.6, gamma=1)
        self.support_vec_hjpd = svm.SVC(kernel="rbf", C=1.0, gamma=0.7)
        self.dataRADtrain = []
        self.actionRADtrain = []
        self.dataRADtest = []
        self.actionRADtest = []
        self.dataHJPDtrain = []
        self.actionHJPDtrain = []
        self.dataHJPDtest = []
        self.actionHJPDtest = []
        # self.clf = make_pipeline(StandardScaler(), svm.SVC(gamma='auto'))



    def evalSVM(self):
        y_test = np.array(self.actionRADtest).T
        for k, v in self.parameters.items():
            for val in v:
                self.clf = svm.SVC().set_params(**{k: val})
                self.clf.fit(self.dataRADtrain, np.array(self.actionRADtrain).T)
                predicted = self.clf.predict(self.dataRADtest)
                # print(self.clf,  self.clf.score(self.dataRADtest, np.array(self.actionRADtest).T))
                print(self.clf)
                print("Precision: ",  metrics.precision_score(y_test, predicted, average=None, zero_division=0))
                print("Accuracy: ",metrics.accuracy_score(y_test, predicted))
                print("Confusion Matrix:\n",metrics.confusion_matrix(y_test, predicted))
                
    
    def evalSVMHJPD(self):
        y_test = np.array(self.actionHJPDtest).T
        for k, v in self.parameters.items():
            for val in v:
                self.clf = svm.SVC().set_params(**{k: val})
                self.clf.fit(self.dataHJPDtrain, np.array(self.actionHJPDtrain).T)
                predicted = self.clf.predict(self.dataHJPDtest)
                # print(self.clf,  self.clf.score(self.dataHJPDtest, np.array(self.actionHJPDtest).T))
                print(self.clf)
                print("Precision: ",  metrics.precision_score(y_test, predicted, average=None, zero_division=0))
                print("Accuracy: ",metrics.accuracy_score(y_test, predicted))
                print("Confusion Matrix:\n",metrics.confusion_matrix(y_test, predicted))
                

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
                    self.dataRADtrain = data
                    self.actionRADtrain = action
                    # self.support_vec_rad.fit(data, np.array(action).T)
                    # self.clf.fit(data, np.array(action).T)
                elif filename is file[1]:
                    self.dataRADtest = data
                    self.actionRADtest = action
                    # print("RAD Accuracy:", self.support_vec_rad.score(data, np.array(action).T))
                    # print("RAD Accuracy CLF:", self.clf.score(data, np.array(action).T))
                    #self.radSVMTest(data, action)
                    print("Testing SVM for RAD")
                    self.evalSVM()
                elif filename is file[2]:
                    self.dataHJPDtrain = data
                    self.actionHJPDtrain = action
                    # self.support_vec_hjpd.fit(data, np.array(action).T)
                elif filename is file[3]:
                    self.dataHJPDtest = data
                    self.actionHJPDtest = action
                    print("\nTesting SVM for HJPD")
                    self.evalSVMHJPD()
                    # print("HJPD Accuracy:", self.support_vec_hjpd.score(data, np.array(action).T))



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
