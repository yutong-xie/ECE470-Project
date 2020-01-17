#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
import rotations
import numpy as np

import os.path
from os import path
import csv

#class for calculating the robots forward kinematics
class ForwardKinematics:

    #initialize class instance, creating the S and M matrices
    def __init__(self):  
        
        #by vrep handles
        pos_1 = np.array([[0.00012460349535104], [8.5577368736267e-05], [0.1044727563858]])
        pos_2 = np.array([[-0.11154142022133], [5.4415315389633e-05], [0.10887312889099]])
        pos_3 = np.array([[-0.11154179275036], [0.00013033905997872], [0.35252329707146]])
        pos_4 = np.array([[-0.11154127120972], [8.5248146206141e-05], [0.56577336788177]])
        pos_5 = np.array([[-0.11222532391548], [8.5233012214303e-05], [0.64999091625214]])
        pos_6 = np.array([[-0.11154122650623], [8.518691174686e-05], [0.65112340450287]])
        
        pos_W = np.array([[0.0],[0.0],[0.0]])
        pos_E = np.array([[-0.22553959488869], [0.0067161759361625], [0.65112614631653]])

        P = pos_E - pos_W
        R = np.identity(3)
        M = np.hstack([R, P])
        self.M = np.vstack([M,np.array([0.0,0.0,0.0,1.0])])

        joint_positions = []
        joint_positions.append(pos_1)
        joint_positions.append(pos_2)
        joint_positions.append(pos_3)
        joint_positions.append(pos_4)
        joint_positions.append(pos_5)
        joint_positions.append(pos_6)

        q = []
        for i in range(0,len(joint_positions)):
            q.append(joint_positions[i])
        
        w1 = np.array([[0.0],[0.0],[1.0]])
        w2 = np.array([[-1.0],[0.0],[0.0]])
        w3 = np.array([[-1.0],[0.0],[0.0]])
        w4 = np.array([[-1.0],[0.0],[0.0]])
        w5 = np.array([[0.0],[0.0],[1.0]])
        w6 = np.array([[-1.0],[0.0],[0.0]])

        self.w = []
        self.w.append(w1)
        self.w.append(w2)
        self.w.append(w3)
        self.w.append(w4)
        self.w.append(w5)
        self.w.append(w6)

        self.v = []
        for i in range(0,len(joint_positions)):
            self.v.append(-np.cross(self.w[i],q[i],axis=0)) 

        w0 = self.w[0]
        v0 = self.v[0]
        S = np.vstack([w0,v0])
            
        for i in range(1,6):
            wi = self.w[i]
            vi = self.v[i]
            Si = np.vstack([wi,vi])
            S = np.hstack([S,Si])
        self.S = S

    #fetch the S and M matrices
    def get_S_and_M(self):
        return self.S, self.M

    #fetch the pose with respect to the base given the joint angles
    def get_T_EinW(self, theta1, theta2, theta3, theta4, theta5, theta6):
        theta = []
        theta.append(theta1)
        theta.append(theta2)
        theta.append(theta3)
        theta.append(theta4)
        theta.append(theta5)
        theta.append(theta6)
        
        e_to_S_theta = []
        for i in range(0,len(self.w)):
            I = np.identity(3)
            w_bracket = rotations.skew3(self.w[i])
            e_to_w_theta = I + np.sin(theta[i])*w_bracket + (1 - np.cos(theta[i]))*np.matmul(w_bracket,w_bracket)

            v_mod = I*theta[i] + (1-np.cos(theta[i]))*w_bracket + (theta[i]-np.sin(theta[i]))*np.matmul(w_bracket,w_bracket)
            v_mod = np.matmul(v_mod, self.v[i])

            e_to_S_theta_i = np.hstack([e_to_w_theta, v_mod])
            e_to_S_theta_i = np.vstack([e_to_S_theta_i, np.array([0.0,0.0,0.0,1.0]) ])
            e_to_S_theta.append(e_to_S_theta_i)

        T = self.M
        for i in range(0,len(self.w)):
            j = len(self.w) - i - 1 
            T = np.matmul(e_to_S_theta[j],T)

        return T

