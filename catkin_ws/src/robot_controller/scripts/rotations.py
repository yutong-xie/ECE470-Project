#!/usr/bin/env python

import numpy as np

def Rz(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0.0],
                     [np.sin(theta), np.cos(theta), 0.0],
                     [0.0, 0.0, 1.0]])

def Ry(theta):
    return np.array([[np.cos(theta), 0.0,  np.sin(theta)],
                     [0.0, 1.0, 0.0],
                     [-np.sin(theta), 0.0, np.cos(theta)]])
                
def Rx(theta):
    return np.array([[1.0, 0.0, 0.0],
                     [0.0, np.cos(theta), -np.sin(theta)],
                     [0.0, np.sin(theta), np.cos(theta)]])

def skew3(vector):
    return np.array([[0.0, -vector[2], vector[1]],[vector[2], 0.0, -vector[0]],[-vector[1], vector[0], 0.0]])

def vec_from_skew3(skew_3):
    return np.array([[-skew_3[1,2]],[skew_3[0,2]],[-skew_3[0,1]]])

def screw_to_bracket(screw):
    w = screw[0:3]
    v = screw[3:6]
    S_bracket = np.hstack([skew3(w),v])
    return np.vstack([S_bracket,[0,0,0,0]])

def bracket_to_screw(bracket):
    w_skew = bracket[0:3, 0:3]
    w = vec_from_skew3(w_skew)
    v = np.array( [ [bracket[0,3]],[bracket[1,3]],[bracket[2,3]] ] )
    return np.vstack([w,v])

def print_np(mat):
    npstr = ''
    npstr += '['
    for x in range(mat.shape[0]):
        if len(mat.shape) == 1:
            npstr += '['
            npstr += str(mat[x])
            npstr += ']'
        else:
            npstr += '['
            for y in range(mat.shape[1]):
                npstr += str(mat[x][y])
                if y != mat.shape[1] - 1:
                    npstr += ', '
            npstr += ']'
        if x != mat.shape[0] - 1:
            npstr += ', '
    npstr += ']'
    print(npstr)