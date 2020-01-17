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
from scipy.linalg import   expm, sinm, cosm, logm
import modern_robotics as mr

#class for calculating the robots inverse kinematics
class InverseKinematics:

    #initialize class instance
    def __init__(self):  
        self.eomg = 0.0000001
        self.ev = 0.0000001

    #get a path of joint angles that will smoothly move from one pose to another
    def get_thetas_path(self, T_1in0, M, S, theta0):
        thetas_path = theta0        

        num_joints = S.shape[1]

        found = False
        max_attempts = 10
        current_attempt = 0

        jacobian_scalar = 0.4

        while not found and current_attempt < max_attempts:
            thetas_path = theta0
                    
            max_iterations = 150
            current_iteration = 0
            while not found and current_iteration < max_iterations:
                
                theta_end = thetas_path[-1,:]
                theta = theta_end.reshape(-1,1)
                
                T = np.eye(4)
                for i in range(0,num_joints):
                    Si = S[:,i].reshape(-1,1)
                    Si_bracket = rotations.screw_to_bracket(Si)

                    thetai = theta[i]
                    T = np.matmul(T,expm(Si_bracket*thetai))

                T = np.matmul(T,M)

                Vb_bracket = logm(np.matmul(np.linalg.inv(T),T_1in0))
                Vb = rotations.bracket_to_screw(Vb_bracket)
                
                w = Vb[0:3]
                v = Vb[3:6]

                delta_T = T_1in0 - T    
                eT = 0.00001

                if np.linalg.norm(delta_T) < eT :
                    found = True
                else:
                    #increment theta            
                    S1 = S[:,0].reshape(-1,1)
                    S1_bracket = rotations.screw_to_bracket(S1)
                    B1_bracket = np.matmul(np.matmul(np.linalg.inv(M),S1_bracket),M)
                    Blist = rotations.bracket_to_screw(B1_bracket)
                    for i in range(1,num_joints):
                        Si = S[:,i].reshape(-1,1)
                        Si_bracket = rotations.screw_to_bracket(Si)
                        Bi_bracket = np.matmul(np.matmul(np.linalg.inv(M),Si_bracket),M)
                        Bi = rotations.bracket_to_screw(Bi_bracket)
                        Blist = np.hstack([Blist, Bi])

                    Jb = mr.JacobianBody(Blist, theta)
                    
                    if Jb.shape[0] == Jb.shape[1]:
                        Jb_dagger = np.linalg.inv(Jb)
                    elif Jb.shape[0] > Jb.shape[1]:
                        Jb_dagger = np.matmul(np.linalg.inv(np.matmul(Jb.T,Jb)),Jb.T)
                    else:
                        Jb_dagger = np.matmul(Jb.T,np.linalg.inv(np.matmul(Jb,Jb.T)))

                    Jb_norm = np.linalg.norm(Jb_dagger)
                    Jb_dagger = Jb_dagger/Jb_norm*jacobian_scalar*(current_attempt+1)

                    theta = theta + np.matmul(Jb_dagger,Vb)
                    
                    for i in range(0,num_joints):

                        theta[i] = theta[i]%(2*np.pi)
                        if theta[i] > np.pi:
                            theta[i] = theta[i] - 2*np.pi
                        if theta[i] < -np.pi:
                            theta[i] = theta[i] + 2*np.pi
                        
                    thetas_path = np.vstack([thetas_path,theta.reshape(1,-1)[0]])

                    current_iteration  = current_iteration + 1

                    if current_iteration == max_iterations:
                        current_attempt = current_attempt + 1


        if found:
            print('Inverse kinematics convergence success!')
        else:
            print('Inverse kinematics convergence failure!')

        return thetas_path   

    #general code for inverse kinematics, handles any number of joints and DOF given T, M, and S
    def get_thetas(self, T_1in0, M, S, theta0):

        num_joints = S.shape[1]

        found = False
        max_attempts = 20
        current_attempt = 0

        while not found and current_attempt < max_attempts:
            theta = theta0        

            max_iterations = 50
            current_iteration = 0
            while not found and current_iteration < max_iterations:

                T = np.eye(4)
                for i in range(0,num_joints):
                    Si = S[:,i].reshape(-1,1)
                    Si_bracket = rotations.screw_to_bracket(Si)
                    thetai = theta[i]
                    T = np.matmul(T,expm(Si_bracket*thetai))

                T = np.matmul(T,M)

                Vb_bracket = logm(np.matmul(np.linalg.inv(T),T_1in0))
                Vb = rotations.bracket_to_screw(Vb_bracket)
                
                w = Vb[0:3]
                v = Vb[3:6]

                if np.linalg.norm(w) < self.eomg and np.linalg.norm(v) < self.ev :
                    found = True
                else:
                    #increment theta            
                    S1 = S[:,0].reshape(-1,1)
                    S1_bracket = rotations.screw_to_bracket(S1)
                    B1_bracket = np.matmul(np.matmul(np.linalg.inv(M),S1_bracket),M)
                    Blist = rotations.bracket_to_screw(B1_bracket)
                    for i in range(1,num_joints):
                        Si = S[:,i].reshape(-1,1)
                        Si_bracket = rotations.screw_to_bracket(Si)
                        Bi_bracket = np.matmul(np.matmul(np.linalg.inv(M),Si_bracket),M)
                        Bi = rotations.bracket_to_screw(Bi_bracket)
                        Blist = np.hstack([Blist, Bi])

                    Jb = mr.JacobianBody(Blist, theta)
                    
                    if Jb.shape[0] == Jb.shape[1]:
                        Jb_dagger = np.linalg.inv(Jb)
                    elif Jb.shape[0] > Jb.shape[1]:
                        Jb_dagger = np.matmul(np.linalg.inv(np.matmul(Jb.T,Jb)),Jb.T)
                    else:
                        Jb_dagger = np.matmul(Jb.T,np.linalg.inv(np.matmul(Jb,Jb.T)))

                    theta = theta + np.matmul(Jb_dagger,Vb)
                    
                    current_iteration  = current_iteration + 1

                    if current_iteration == max_iterations:
                        current_attempt = current_attempt + 1


        if found:
            print('Inverse kinematics convergence success!')
        else:
            print('Inverse kinematics convergence failure!')

        
        #reduce thetas to [-pi,pi]
        for i in range(0,num_joints):
            theta[i] = theta[i]%(2*np.pi)
            if theta[i] > np.pi:
                theta[i] = theta[i] - 2*np.pi  
            if theta[i] < -np.pi:
                theta[i] = theta[i] + 2*np.pi  


        print('Expected end effector transformation given the inverse kinematics joint angles')
        T = np.eye(4)
        for i in range(0,num_joints):
            Si = S[:,i].reshape(-1,1)
            Si_bracket = rotations.screw_to_bracket(Si)
            thetai = theta[i]
            T = np.matmul(T,expm(Si_bracket*thetai))

        T = np.matmul(T,M)
        print(T)

        return theta