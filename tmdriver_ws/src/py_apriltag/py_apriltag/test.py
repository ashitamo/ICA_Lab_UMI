#!/usr/bin/env python3
import math
import numpy as np

import math

from geometry_msgs.msg import TransformStamped,PoseStamped

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

from dt_apriltags import Detector
import os
import cv2
from scipy.spatial.transform import Rotation as R


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
    rot = R.from_matrix(T_a_E1[:3,:3]).as_euler('xyz', degrees=True)
    rot[0],rot[1] = 180.0,0.0
    T_a_E1[:3,:3] = R.from_euler('xyz', rot, degrees=True).as_matrix()
    T_a_C1 = T_a_E1 @ T_E_C
    temp = R.from_matrix(T_a_C1[:3,:3]).as_quat()
    T_a_C1 = np.array([T_a_C1[0,3],T_a_C1[1,3],T_a_C1[2,3],temp[0],temp[1],temp[2],temp[3]])
    return T_a_C1


class FramePublisher(Node):

    def __init__(self):
        super().__init__('pose_publisher')
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.create_timer(0.016, self.p)

    def p(self):
        p_C0_o0 = np.array([0.005784650413574422, 0.00402626632552662, 0.013999999646330252]) *10
        T_a_C0 = np.array([-0.120461, 0.011688, 0.235501, -0.700088, 0.708054, 0.091179, -0.014905])

        T_W_a = np.array(
            [[ 0.05392254, 0.99852385,-0.00651684,-0.32107211],
            [-0.99834257, 0.05404189, 0.01978821,-0.7034037 ],
            [ 0.02011118, 0.00543901, 0.99978295, 0.01247291],
            [ 0.        , 0.        , 0.        , 1.        ]]
        )

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
        rot = R.from_matrix(T_a_E1[:3,:3]).as_euler('xyz', degrees=True)
        rot[0],rot[1] = 180.0,0.0
        T_a_E1[:3,:3] = R.from_euler('xyz', rot, degrees=True).as_matrix()
        T_a_C1 = T_a_E1 @ T_E_C
        # temp = R.from_matrix(T_a_C1[:3,:3]).as_quat()
        # T_a_C1 = np.array([T_a_C1[0,3],T_a_C1[1,3],T_a_C1[2,3],temp[0],temp[1],temp[2],temp[3]])


        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'april_tag'
        t.transform.translation.x = float(T_W_a[0,3])
        t.transform.translation.y = float(T_W_a[1,3])
        t.transform.translation.z = float(T_W_a[2,3])
        q = R.from_matrix(T_W_a[:3,:3]).as_quat()
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)
        

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'april_tag'
        t.child_frame_id = 'c0'
        t.transform.translation.x = float(T_a_C0[0,3])
        t.transform.translation.y = float(T_a_C0[1,3])
        t.transform.translation.z = float(T_a_C0[2,3])
        q = R.from_matrix(T_a_C0[:3,:3]).as_quat()
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'c0'
        t.child_frame_id = 'o0'
        t.transform.translation.x = float(T_C0_o0[0,3])
        t.transform.translation.y = float(T_C0_o0[1,3])
        t.transform.translation.z = float(T_C0_o0[2,3])
        q = R.from_matrix(T_C0_o0[:3,:3]).as_quat()
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'april_tag'
        t.child_frame_id = 'c1'
        t.transform.translation.x = float(T_a_C1[0,3])
        t.transform.translation.y = float(T_a_C1[1,3])
        t.transform.translation.z = float(T_a_C1[2,3])
        q = R.from_matrix(T_a_C1[:3,:3]).as_quat()
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'c1'
        t.child_frame_id = 'e1'
        t.transform.translation.x = float(T_C_E[0,3])
        t.transform.translation.y = float(T_C_E[1,3])
        t.transform.translation.z = float(T_C_E[2,3])
        q = R.from_matrix(T_C_E[:3,:3]).as_quat()
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]        
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

    

def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()

# Current object position in camera frame: (0.005784650413574422, 0.00402626632552662, 0.013999999646330252)
# Current object position in world frame: [-0.04312665  0.00137246  0.33544388 -0.69127177  0.6992027   0.15976637
#  -0.08794112]
# 1 [-0.04312665  0.00137246  0.33544388 -0.69127177  0.6992027   0.15976637
#  -0.08794112] 0.071294
# <class 'numpy.ndarray'>
# <class 'float'>
# [INFO] [1757472994.607071353] [executor_node]: Publishing: Pose=array('d', [0.0, -0.120461, 0.011688, 0.235501, -0.700088, 0.708054, 0.091179, -0.014905, 0.071294]) with ID=0
# [INFO] [1757473001.645698520] [executor_node]: Publishing: Pose=array('d', [1.0, -0.043126652110451136, 0.00137245974920724, 0.3354438789839984, -0.691271765270915, 0.6992026987351818, 0.1597663665865552, -0.08794112080478671, 0.071294]) with ID=1.0
