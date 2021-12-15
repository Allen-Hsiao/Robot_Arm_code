# forward_kinematics_poe(Button savior).py
import numpy as np
from math import pi, cos, sin, sqrt

def forward_kinematics_poe(joints):
    # input: length L and joint angles [theta1, theta2, theta3, theta4, theta5, theta6]
    # output: the final transformation matrix T (from frame {s} to frame {b})
    # complete the computation using Product of Exponentials
    joint1 = joints[0]
    joint2 = joints[1]
    joint3 = joints[2]
    joint4 = joints[3]
    L = 0.02

    
    I = np.array([[1,0,0],[0,1,0],[0,0,1]])
    U = np.array([[0,0,0,1]])
    M = np.array([[1,0,0,-1.4*L], [0,1,0,0], [0,0,1,2*L], [0,0,0,1]]) #input M matrix

    #T1   #input V1 and w1
    v1 = np.array([[0,0,0]])
    w1 = np.array([[0,0,1]])
    w_1 = np.array([[0,-1*w1[0,2],w1[0,1]], [w1[0,2],0,-1*w1[0,0]], [-1*w1[0,1],w1[0,0],0]])
    w_12 = np.dot(w_1,w_1)


    R1 = I + sin(joint1)*w_1 + (1-cos(joint1))*w_12
    G01 = (I*joint1 + (1-cos(joint1))*w_1 + (joint1 - sin(joint1))*w_12)
    G1 = np.dot(G01,v1.T)
    T1 = np.hstack([R1,G1])
    T_1 = np.vstack([T1,U])

    #T2
    v2 = np.array([[3*L,0,0]])
    w2 = np.array([[0,-1,0]])
    w_2 = np.array([[0,-1*w2[0,2],w2[0,1]], [w2[0,2],0,-1*w2[0,0]], [-1*w2[0,1],w2[0,0],0]])
    w_22 = np.dot(w_2,w_2)


    R2 = I + sin(joint2)*w_2 + (1-cos(joint2))*w_22
    G02 = (I*joint2 + (1-cos(joint2))*w_2 + (joint2 - sin(joint2))*w_22)
    G2 = np.dot(G02,v2.T)
    T2 = np.hstack([R2,G2])
    T_2 = np.vstack([T2,U])

    #T3
    v3 = np.array([[3.7*L,0,0.7*L]])
    w3 = np.array([[0,-1,0]])
    w_3 = np.array([[0,-1*w3[0,2],w3[0,1]], [w3[0,2],0,-1*w3[0,0]], [-1*w3[0,1],w3[0,0],0]])
    w_32 = np.dot(w_3,w_3)


    R3 = I + sin(joint3)*w_3 + (1-cos(joint3))*w_32
    G03 = (I*joint3 + (1-cos(joint3))*w_3 + (joint3 - sin(joint3))*w_32)
    G3 = np.dot(G03,v3.T)
    T3 = np.hstack([R3,G3])
    T_3 = np.vstack([T3,U])

    #T4
    v4 = np.array([[3*L,0,1.4*L]])
    w4 = np.array([[0,-1,0]])
    w_4 = np.array([[0,-1*w4[0,2],w4[0,1]], [w4[0,2],0,-1*w4[0,0]], [-1*w4[0,1],w4[0,0],0]])
    w_42 = np.dot(w_4,w_4)


    R4 = I + sin(joint4)*w_4 + (1-cos(joint4))*w_42
    G04 = (I*joint4 + (1-cos(joint4))*w_4 + (joint4 - sin(joint4))*w_42)
    G4 = np.dot(G04,v4.T)
    T4 = np.hstack([R4,G4])
    T_4 = np.vstack([T4,U])

    
    T12 = np.dot(T_1,T_2)
    T23 = np.dot(T12, T_3)
    T34 = np.dot(T23, T_4)
    T = np.dot(T34, M)
    return T

joint1 = input('Enter the joint1 variable: ')
joint2 = input('Enter the joint2 variable: ')
joint3 = input('Enter the joint3 variable: ')
joint4 = input('Enter the joint4 variable: ')

result = forward_kinematics_poe((joint1, joint2, joint3, joint4))
print("End-effector's postition:")
print('{} {} {} {} {} {}'.format("X:", result[0,3],"Y:",result[1,3],"Z:",result[2,3]))