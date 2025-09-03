#!/usr/bin/env python3
import math
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped




'''
給物體(T_C_O)的位置 我給相機要移到哪T_C_E
轉T_E_O

'''
def c2e(p_C0_o0,T_a_C0):
    '''
    p_C0_o0 : np.array([x,y,z])
    T_a_C0 : np.array([x,y,z,rx,ry,rz,rw])
    =>
    T_a_C1 np.array([x,y,z,rx,ry,rz,rw])
    '''
    import numpy as np
    from scipy.spatial.transform import Rotation as R
    T_G_C = np.array(
        [[ 0.99991712, -0.01191628,  0.00487421,-0.00767288],
        [ 0.01069885,  0.97968111,  0.20027597, -0.06552731],
        [-0.00716172, -0.20020722,  0.9797274,   0.03914132],
        [ 0,          0,          0,          1,        ]]
    )
    T_G_E = np.array(
        [[ 1, 0, 0, 0],
        [ 0, 1, 0, 0],
        [ 0, 0, 1, 0.164],
        [ 0, 0, 0, 1]]
    )
    T_C_E = np.linalg.inv(T_G_C) @ T_G_E
    T_E_C = np.linalg.inv(T_C_E) 

    T_C0_o0 = np.array(
        [[ 1, 0, 0, p_C0_o0[0]],
         [ 0, 1, 0, p_C0_o0[1]],
         [ 0, 0, 1, p_C0_o0[2]],
         [ 0, 0, 0, 1]]
    )
    temp = R.from_quat(T_a_C0[3:]).as_matrix()
    T_a_C0 = np.array(
        [[ temp[0,0], temp[0,1], temp[0,2], T_a_C0[0]],
         [ temp[1,0], temp[1,1], temp[1,2], T_a_C0[1]],
         [ temp[2,0], temp[2,1], temp[2,2], T_a_C0[2]],
         [ 0, 0, 0, 1]]
    )
    T_a_o0 = T_a_C0 @ T_C0_o0
    T_a_E1 = T_a_o0
    T_a_C1 = T_a_E1 @ T_E_C
    temp = R.from_matrix(T_a_C1[:3,:3]).as_quat()
    T_a_C1 = np.array([T_a_C1[0,3],T_a_C1[1,3],T_a_C1[2,3],temp[0],temp[1],temp[2],temp[3]])
    return T_a_C1

    
from scipy.spatial.transform import Rotation as R
p_C_o0 = np.array([0,0,0.2])
T_a_C0 = np.array([0,0,0.3,0,0,0,1])
T_a_C0[3:] = R.from_euler('xyz',[180,0,0], degrees=True).as_quat()
T_a_C1 = c2e(p_C_o0,T_a_C0)
print(R.from_quat(T_a_C1[3:]).as_euler('xyz', degrees=True))
print(T_a_C1)