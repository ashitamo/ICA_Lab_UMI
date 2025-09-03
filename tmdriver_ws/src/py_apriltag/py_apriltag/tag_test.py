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

# This function is a stripped down version of the code in
# https://github.com/matthew-brett/transforms3d/blob/f185e866ecccb66c545559bc9f2e19cb5025e0ab/transforms3d/euler.py
# Besides simplifying it, this version also inverts the order to return x,y,z,w, which is
# the way that ROS prefers it.
def quaternion_from_euler(ai, aj, ak): #內旋
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

print(R.from_euler('xyz',[180,0,90], degrees=True).as_matrix())

R_0_1 = np.array([[ 0,  1,  0],
            [1,  0,  0],
            [ 0, 0,  -1]])
#R.from_matrix(R_0_1).as_euler('xyz', degrees=True) 外旋 按照xyz顺序返回

print(R.from_matrix(R_0_1).as_euler('xyz', degrees=True))
R_1_0 = np.linalg.inv(R_0_1)

T_W_W = np.eye(4)
T_W_A = np.array([[ 0, 1, 0, 0.170],
                  [ 1, 0, 0 ,-0.415],
                  [ 0, 0, -1, 0],
                  [ 0, 0,  0 ,1]])
T_A_W = np.linalg.inv(T_W_A)

T_a_A =  np.array([[ 0, 1, 0, 0],
                  [ 1, 0, 0 ,0],
                  [ 0, 0, -1, 0],
                  [ 0, 0,  0 ,1]])
T_A_a = np.linalg.inv(T_a_A)

T_A_C = np.array([[ 1, 0, 0, 0],
                  [ 0, 1, 0 ,0],
                  [ 0, 0, 1, -0.3],
                  [ 0, 0,  0 ,1]])

T_C_A = np.linalg.inv(T_A_C)

T_C_c = np.array([[ -1, 0, 0, 0],
                  [ 0, 1, 0 ,0],
                  [ 0, 0, -1 ,0],
                  [ 0, 0,  0 ,1]])

T_c_C = np.linalg.inv(T_C_c)

R_W_G = np.array([[ -1, 0, 0,],
                  [ 0, -1, 0],
                  [ 0, 0, 1]]) 
R_G_W = np.linalg.inv(R_W_G)

T = np.array(
    [[ 0.99991712, -0.01191628,  0.00487421,-0.00767288],
    [ 0.01069885,  0.97968111,  0.20027597, -0.06552731],
    [-0.00716172, -0.20020722,  0.9797274,   0.03914132],
    [ 0,          0,          0,          1,        ]]
 )

print(R.from_matrix(T[:3,:3]).as_euler('xyz', degrees=True))

class FramePublisher(Node):

    def __init__(self):
        super().__init__('pose_publisher')
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.create_timer(0.016, self.p)

    def p(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'april_tag'
        T_W_a = T_W_A @ T_A_a
        rot = R.from_matrix(T_W_a[:3,:3]).as_euler('xyz', degrees=False) # zyx 內旋
        q = quaternion_from_euler(rot[0],rot[1],rot[2])
        t.transform.translation.x = float(T_W_a[0,3])
        t.transform.translation.y = float(T_W_a[1,3])
        t.transform.translation.z = float(T_W_a[2,3])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)
       
        # t = TransformStamped()
        # t.header.stamp = self.get_clock().now().to_msg()
        # t.header.frame_id = 'world'
        # t.child_frame_id = 'camera'
        # T_W_C = T_W_A @ T_A_C
        # rot = R.from_matrix(T_W_C[:3,:3]).as_euler('xyz', degrees=False)
        # q = quaternion_from_euler(rot[0],rot[1],rot[2])
        # t.transform.translation.x = float(T_W_C[0,3])
        # t.transform.translation.y = float(T_W_C[1,3])
        # t.transform.translation.z = float(T_W_C[2,3])
        # t.transform.rotation.x = q[0]
        # t.transform.rotation.y = q[1]
        # t.transform.rotation.z = q[2]
        # t.transform.rotation.w = q[3]
        # self.tf_broadcaster.sendTransform(t)

        # t = TransformStamped()
        # t.header.stamp = self.get_clock().now().to_msg()
        # t.header.frame_id = 'world'
        # t.child_frame_id = 'c'
        # T_W_c = T_W_C @ T_C_c
        # rot = R.from_matrix(T_W_c[:3,:3]).as_euler('xyz', degrees=False)
        # q = quaternion_from_euler(rot[0],rot[1],rot[2])
        # t.transform.translation.x = float(T_W_c[0,3])
        # t.transform.translation.y = float(T_W_c[1,3])
        # t.transform.translation.z = float(T_W_c[2,3])
        # print(t.transform.translation.x,t.transform.translation.y,t.transform.translation.z)
        # R_G_c = T_c_C[:3,:3] @ R_G_W
        # rot = R.from_matrix(R_G_c).as_euler('xyz', degrees=False)
        # print(rot)
        # t.transform.rotation.x = q[0]
        # t.transform.rotation.y = q[1]
        # t.transform.rotation.z = q[2]
        # t.transform.rotation.w = q[3]
        # self.tf_broadcaster.sendTransform(t)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'arm'
        t.child_frame_id = 'ca'
        rot = R.from_matrix(T[:3,:3]).as_euler('xyz', degrees=False)
        q = quaternion_from_euler(rot[0],rot[1],rot[2])
        t.transform.translation.x = float(T[0,3])
        t.transform.translation.y = float(T[1,3])
        t.transform.translation.z = float(T[2,3])
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
