import numpy as np
from math import pi, cos, sin, atan, atan2, sqrt, acos


def inverse_kinematics(position):
    # input: the position of end effector [x, y, z]
    # output: joint angles [joint1, joint2, joint3]
    # add your code here to complete the computation

    L = 0.02
    x = position[0]
    y = position[1]
    z = position[2]
    theta3 = pi/2 #Fixed joint3 variable
    
    if x < -0.048 or x > 0.048:
        return "Out of workspace"
    if y < -0.048 or y > 0.048:
        return "Out of workspace"
    if z > 0.1 or z < 0.017:
        return "Out of workspace"
    
    theta1 = atan2(y,x)
    D = pi/2 - theta3
    #O triangle
    z = z - (3*L)
    G = sqrt(pow(x,2) + pow(y,2))
    O = atan2(z,G)
    Ohyp = sqrt(pow(G,2) + pow(z,2))
    #P triangle
    esid = sqrt(pow(L,2) + pow(L,2))
    C = acos((pow(esid,2) + pow(L,2) - pow(Ohyp,2))/ (2*esid*L))
    P = acos((pow(esid,2) + pow(Ohyp,2) - pow(L,2))/ (2*esid*Ohyp))
    #D = acos((pow(L,2) + pow(L,2) - pow(esid,2))/ (2*pow(L,2)))
    #e trianlge
    e = acos((pow(L,2) + pow(esid,2) - pow(L,2))/ (2*L*esid))
    F = pi - e - D

    theta4 = pi - (F + C) 
    theta2 = pi/2 - (e + O + P)
    

    [joint1, joint2, joint3, joint4] = [theta1, theta2, theta3, theta4]


    return [joint1, joint2, joint3, joint4]

x = input('Enter the end-effector x position: ')
y = input('Enter the end-effector y position: ')
z = input('Enter the end-effector z position: ')

print('Four joints variable:')
print(inverse_kinematics([x, y ,z]))