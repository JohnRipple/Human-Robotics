#!/usr/bin/env python3
# John Ripple
# Human Centered Robotics
# Project 3 Part 2
# 4/28/2023

from sklearn import svm
from sklearn.pipeline import make_pipeline
from sklearn import metrics
import os
import numpy as np

class createSVM:
    def __init__(self):
        # Create a number of parameters for various SVM testing
        self.parameters =   {'C': [0.001, 0.01, 0.1, 1, 10, 100, 1000],
            'kernel': ['rbf', 'linear', 'poly', 'sigmoid'],
            'gamma': ['auto', 1, 0.1, 0.02, 0.001]}
        
        # Final SVM with best parameters
        self.support_vec_rad = svm.SVC(kernel="rbf", C=0.6, gamma=1)
        self.support_vec_hjpd = svm.SVC(kernel="rbf", C=9.0, gamma=0.7)
        
        # Create values to store data for SVM testing at various parameters
        self.dataRADtrain = []
        self.actionRADtrain = []
        self.dataRADtest = []
        self.actionRADtest = []
        self.dataHJPDtrain = []
        self.actionHJPDtrain = []
        self.dataHJPDtest = []
        self.actionHJPDtest = []
        self.save_file = ""


    def bmatrix(self, a):
        """Returns a LaTeX bmatrix

        :a: numpy array
        :returns: LaTeX bmatrix as a string
        """
        if len(a.shape) > 2:
            raise ValueError('bmatrix can at most display two dimensions')
        lines = str(a).replace('[', '').replace(']', '').splitlines()
        rv = [r'\begin{bmatrix}']
        rv += ['  ' + ' & '.join(l.split()) + r'\\' for l in lines]
        rv +=  [r'\end{bmatrix}']
        return '\n'.join(rv)   
    

    def evalSVM(self, data_train, action_train, data_test, action_test):
        """Test SVM with different parameters"""
        y_test = np.array(action_test).T
        for k, v in self.parameters.items():
            for val in v:
                self.clf = svm.SVC().set_params(**{k: val})         # Set the testing parameter
                self.clf.fit(data_train, np.array(action_train).T)  # Fit the SVM based on training data
                predicted = self.clf.predict(data_test)             # Predict class based on SVM
                print(self.clf)                                     # Print the clf with the different parameter
                self.save_file.write("\n\\\\\\\\" + str(self.clf))
                self.printResults(y_test, predicted)                # print results

                
                
    def printResults(self, y_test, predicted):
        """Print the scores of the SVM"""
        precision = "Precision: " +  str(metrics.precision_score(y_test, predicted, average=None, zero_division=0))
        accuracy = "Accuracy: " + str(metrics.accuracy_score(y_test, predicted))
        confusion = "Confusion Matrix:\n" + self.bmatrix(metrics.confusion_matrix(y_test, predicted))
        self.save_file.write("\n\\\\" + precision + "\n\\\\" +  accuracy + "\n\\\\" + confusion + "\\\\")
        print(precision, accuracy, confusion)
        
        
    def mainLoop(self):
        """Read in file data and output testing data"""
        path = os.path.join((__file__).replace("ripple_svm.py", ""), os.pardir, "saved_files")
        file = ["rad_d1_train.txt", "rad_d1_test.txt", "cust_d1_train.txt", "cust_d1_test.txt"]
        save_path = os.path.join((__file__).replace("ripple_svm.py", ""), os.pardir, "saved_files/", "SVM Results.txt")
        open(save_path, 'w').close()    # Clear contents of the file
        self.save_file = open(save_path, 'w')
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
                    # Get RAD train data
                    self.dataRADtrain = data
                    self.actionRADtrain = action
                    
                    # Create final SVM for RAD train data
                    self.support_vec_rad.fit(data, np.array(action).T)
                elif filename is file[1]:
                    # Get RAD test data
                    self.dataRADtest = data
                    self.actionRADtest = action

                    # Display SVM metrics for the test data
                    print("Testing SVM for RAD\\\\")
                    print(self.support_vec_rad)
                    self.save_file.write("SVM results for RAD\\\\\nBest SVM found (rbf): " + str(self.support_vec_rad))
                    self.printResults(np.array(action).T, self.support_vec_rad.predict(data))
                    self.evalSVM(self.dataRADtrain, self.actionRADtrain, self.dataRADtest, self.actionRADtest)
                elif filename is file[2]:
                    # Get Custom train data
                    self.dataHJPDtrain = data
                    self.actionHJPDtrain = action
                    
                    # Create final SVM for Custom train data
                    self.support_vec_hjpd.fit(data, np.array(action).T)
                elif filename is file[3]:
                    # Get Custom test data
                    self.dataHJPDtest = data
                    self.actionHJPDtest = action
                    
                    # Display SVM metrics for the test data
                    print("\nTesting SVM for HJPD\\\\")
                    print(self.support_vec_hjpd)
                    self.save_file.write("\n\n\n\n#####################################################################################\n\n\n\n")
                    self.save_file.write("SVM results for Custom\\\\\nBest SVM found (rbf): " + str(self.support_vec_hjpd))
                    self.printResults(np.array(action).T, self.support_vec_hjpd.predict(data))
                    self.evalSVM(self.dataHJPDtrain, self.actionHJPDtrain, self.dataHJPDtest, self.actionHJPDtest)
        self.save_file.close()
 

if __name__ == '__main__':
    classifier = createSVM()
    try:
        classifier.mainLoop()
    except KeyboardInterrupt:
        print("\nExiting")
        pass
