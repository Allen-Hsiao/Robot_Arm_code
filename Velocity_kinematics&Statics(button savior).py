#velocity_kinematics(Button savior)
import numpy as np
from math import pi, cos, sin, sqrt, atan, atan2, sqrt, acos

def inverse_kinematics(position):
    # input: the position of end effector [x, y, z]
    # output: joint angles [joint1, joint2, joint3]
    # add your code here to complete the computation

    L = 0.02
    x = position[0]
    y = position[1]
    z = position[2]
    theta3 = pi/2
    
    if x < -0.048 or x > 0.048:
        return "Out of workspace"
    if y < -0.048 or y > 0.048:
        return "Out of workspace"
    if z > 0.1 or z < 0.017:
        return "Out of workspace"
    
    theta1 = atan2(y,x)
    D = theta3
    #O triangle
    z = z - (3*L)
    G = sqrt(pow(x,2) + pow(y,2))
    O = atan2(z,G)
    Ohyp = sqrt(pow(G,2) + pow(z,2))
    #P triangle
    esid = sqrt(pow(L,2) + pow(L,2))
    C = acos((pow(esid,2) + pow(L,2) - pow(Ohyp,2))/ (2*esid*L))
    P = acos((pow(esid,2) + pow(Ohyp,2) - pow(L,2))/ (2*esid*Ohyp))
    D = acos((pow(L,2) + pow(L,2) - pow(esid,2))/ (2*pow(L,2)))
    #e trianlge
    e = acos((pow(L,2) + pow(esid,2) - pow(L,2))/ (2*L*esid))
    F = pi - e - D

    theta4 = pi - (F + C) 
    theta2 = pi/2 - (e + O + P)
    

    [joint1, joint2, joint3, joint4] = [theta1, theta2, theta3, theta4]


    return [joint1, joint2, joint3, joint4]

def velocity_kinematics(joint):
    joint1 = joint[0]
    joint2 = joint[1]
    joint3 = joint[2]
    joint4 = joint[3]
    L = 0.02
    theta1_h = 1
    theta2_h = 1
    theta3_h = 1
    theta4_h = 1

    Jacobian = np.array([[0,0,0,0],[0,-1,-1,-1],[1,0,0,0],[0,3*L, L*cos(joint2), (L*cos(joint2) - L*cos(180-joint2-joint3))], [0,0,0,0],
    [0,0, -L*sin(joint2)*cos(joint1), (L*sin(180-joint2-joint3)*cos(joint1) + L*sin(joint2)*cos(joint1))]])

    Angular_V = np.array([[theta1_h],[theta2_h],[theta3_h],[theta4_h]])

    Vs = np.dot(Jacobian, Angular_V)
    return Vs
def statics(joint):
    joint1 = joint[0]
    joint2 = joint[1]
    joint3 = joint[2]
    joint4 = joint[3]
    L = 0.02

    Jacobian_T = np.array([[0,0,1,0,0,0],[0,-1,0,3*L,0,0],[0,-1,0,L*cos(joint2),0,-L*sin(joint2)*cos(joint1)],
    [0,-1,0,L*cos(joint2)-L*cos(pi-joint2-joint3),0,L*sin(pi-joint2-joint3)*cos(joint1)+L*sin(joint2)*cos(joint1)]])

    F_tip = np.array([[-L*sin(pi-joint2-joint3)*sin(joint1)-L*sin(joint2)*sin(joint1)-L*sin(joint4)*sin(joint1)],
    [L*sin(pi-joint2-joint3)*cos(joint1)+L*sin(joint2)*cos(joint1)+L*sin(joint4)*cos(joint1)],[0],[0],[0],[-1]])

    Torque = np.dot(Jacobian_T, F_tip)
    return [Torque[0,0], Torque[1,0], Torque[2,0], Torque[3,0]]


x = input('Enter the end-effector x position: ')
y = input('Enter the end-effector y position: ')
z = input('Enter the end-effector z position: ')

result_inverse = inverse_kinematics([x, y ,z])
print('The twist at end-effector: ')
print(velocity_kinematics(result_inverse))
print('The torques at each joint: ')
print(statics(result_inverse))

