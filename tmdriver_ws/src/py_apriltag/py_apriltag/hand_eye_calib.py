import math
from nav_msgs.msg import Path
from geometry_msgs.msg import TransformStamped,PoseStamped
from sensor_msgs.msg import Image
from py_gripper_interfaces.srv import Trajectory
from tm_msgs.srv import SetPositions,SetEvent
from tm_msgs.msg import FeedbackState
from robotiq_85_msgs.msg import GripperCmd

import cv_bridge
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from dt_apriltags import Detector
import os
import cv2
from scipy.spatial.transform import Rotation as R
import time
import queue

camera_params = [427.6786193847656,427.6786193847656,426.2860107421875,240.9849090576172]

# 170 -415 190 mm 180 0 -90
T_W_W = np.eye(4)

T_a_A =  np.array([[ 0, 1, 0, 0],
                  [ 1, 0, 0 ,0],
                  [ 0, 0, -1, 0],
                  [ 0, 0,  0 ,1]])
T_A_a = np.linalg.inv(T_a_A)

T_C_c = np.array([[ -1, 0, 0, 0],
                  [ 0, 1, 0 ,0],
                  [ 0, 0, -1 ,0],
                  [ 0, 0,  0 ,1]])

T_c_C = np.linalg.inv(T_C_c)

R_W_G = np.array([[ -1, 0, 0,],
                  [ 0, -1, 0],
                  [ 0, 0, 1]]) 
R_G_W = np.linalg.inv(R_W_G)
        
    

class HandEyeCalib(Node):
    def __init__(self):
        super().__init__('hand_eye_calib')

        self.pos_cli = self.create_client(SetPositions, 'set_positions')
        self.event_cli = self.create_client(SetEvent, 'set_event')
        self.gripper_pub = self.create_publisher(GripperCmd, '/gripper/cmd', 10)
        self.pos_sub = self.create_subscription(FeedbackState, 'feedback_states', self.pos_callback, 10)
        while not self.pos_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        while not self.event_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.set_positions_req = SetPositions.Request()
        self.set_event_req = SetEvent.Request()
        self.target_positions = [0.2, -0.4, 0.35, 3.14159, 0.0, -1.57]
        self.current_positions = [0.2, -0.4, 0.35, 3.14159, 0.0, -1.57]

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cv_bridge = cv_bridge.CvBridge()
        self.at_detector = Detector(searchpath=['apriltags'],
                       families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

        self.queue = queue.Queue(maxsize=1)
        self._sub = self.create_subscription(
            Image,
            '/camera/infra1/image_rect_raw',
            self.readCam,
            10
        )

        self.T_W_G_list = []
        self.T_C_A_list = []
    def pos_callback(self,msg):
        self.current_positions = msg.tool_pose
        # self.get_logger().info("Current Position: %s" % self.current_positions)

    def is_arrived(self,error=0.001):
        if sum((self.target_positions[i]-self.current_positions[i])**2 for i in range(6)) > error**2:
            return False
        return True

    def send_request(self,positions=[0.2, -0.4, 0.35, 3.14159, 0.0, -1.57],
                     velocity=0.1, acc_time=0.5, blend_percentage=100, fine_goal=False):
        
        self.target_positions = positions
        print(self.target_positions)
        set_positions_req = SetPositions.Request()
        set_positions_req.motion_type = SetPositions.Request.LINE_T
        set_positions_req.positions = positions
        set_positions_req.velocity = velocity
        set_positions_req.acc_time = acc_time
        set_positions_req.blend_percentage = blend_percentage
        set_positions_req.fine_goal = fine_goal
        future = self.pos_cli.call_async(set_positions_req)
        # rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def send_gripper(self,gap=0.085):
        gap = gap if gap < 0.085 else 0.085
        gap = gap if gap > 0.0 else 0.0
        print(gap)
        grip_msg = GripperCmd()
        grip_msg.emergency_release = False
        grip_msg.emergency_release_dir = 0
        grip_msg.stop = False
        grip_msg.position = gap
        grip_msg.speed = 0.1
        grip_msg.force = 1.0
        self.gripper_pub.publish(grip_msg)

    def send_event(self):
        rclpy.spin_once(self)
        self.set_event_req = SetEvent.Request()
        self.set_event_req.func = SetEvent.Request.STOP
        self.set_event_req.arg0 = 0
        self.set_event_req.arg1 = 0
        future = self.event_cli.call_async(self.set_event_req)
        # rclpy.spin_until_future_complete(self, future)
        return future.result()  
    
    def readCam(self,msg):
        frame = self.cv_bridge.imgmsg_to_cv2(msg)
        if self.queue.full():
            self.queue.get()
        self.queue.put(frame)

    def detectTag(self):
        frame = self.queue.get()
        results = self.at_detector.detect(frame,True,camera_params,0.050)
        cv2.circle(frame,(int(camera_params[2]),int(camera_params[3])),5,(0,0,255),-1)
        if len(results) != 0:
            cv2.circle(frame,(int(results[0].center[0]),int(results[0].center[1])),5,(0,255,0),-1)

            cv2.line(frame,(int(results[0].corners[0][0]),int(results[0].corners[0][1])),
                            (int(results[0].corners[1][0]),int(results[0].corners[1][1])),(255,0,0),2)
            
            cv2.line(frame,(int(results[0].corners[1][0]),int(results[0].corners[1][1])),
                            (int(results[0].corners[2][0]),int(results[0].corners[2][1])),(255,0,0),2)
            
            cv2.line(frame,(int(results[0].corners[2][0]),int(results[0].corners[2][1])),
                            (int(results[0].corners[3][0]),int(results[0].corners[3][1])),(255,0,0),2)
            
            cv2.line(frame,(int(results[0].corners[3][0]),int(results[0].corners[3][1])),
                            (int(results[0].corners[0][0]),int(results[0].corners[0][1])),(255,0,0),2)
            r = results[0]
            T_C_A = np.eye(4)
            T_C_A[:3,:3] = r.pose_R
            T_C_A[:3,3] = r.pose_t.reshape(3)
            return T_C_A,frame
        return None,frame
    def pub_tf(self,T,parent_frame,child_frame):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = float(T[0,3])
        t.transform.translation.y = float(T[1,3])
        t.transform.translation.z = float(T[2,3])
        q = R.from_matrix(T[:3,:3]).as_quat()
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

    def HEC(self):
        R_W_G = [T[:3,:3] for T in self.T_W_G_list]
        t_W_G = [T[:3,3] for T in self.T_W_G_list]
        R_C_A = [T[:3,:3] for T in self.T_C_A_list]
        t_C_A = [T[:3,3] for T in self.T_C_A_list]
        return cv2.calibrateHandEye(R_W_G, t_W_G, R_C_A, t_C_A, None, cv2.CALIB_HAND_EYE_DANIILIDIS)

pos =[
    [-0.125, -0.5, 0.3, 3.14159, 0.0, 3.14159],
    [-0.1, -0.5, 0.275, -3.0, 0.0, 3.14159],
    [-0.15, -0.45, 0.21, -2.6, 0.0, 3.14159],
    [-0.2, -0.41, 0.27, -2.7, 0.0, -2.7],
    [-0.075, -0.45, 0.3, -2.8, 0.0, 2.8],
    [-0.1, -0.35, 0.21, -2.5, 0.0, 2.7],
    [-0.205, -0.5, 0.3, -3.14, 0.0, -3.14],
    [-0.195, -0.55, 0.275, -3.14, 0.0, -2.0],
    [-0.25, -0.51, 0.23, -2.5, 0.0, -2.0],
    [-0.05, -0.45, 0.22, -2.7, 0.0, 2.5],
    [-0.02, -0.45, 0.24, -2.6, 0.0, 2.4],
    [0.01, -0.55, 0.21, -2.75, 0.0, 2.0],
    [-0.1, -0.6, 0.27, -3.14, 0.0, 1.8],
    [-0.05, -0.57, 0.31, -3.14, 0.0, 1.57],
    [-0.15, -0.51, 0.27, -3.14, 0.0, 3.14],
]
def main():
    rclpy.init()
    handEyeCalib = HandEyeCalib()
    rclpy.spin_once(handEyeCalib)
    idx = 0
    while True:
        if idx >= len(pos):
            break
        print("current positions: ",handEyeCalib.current_positions)
        positions = pos[idx]
        handEyeCalib.send_request(positions,fine_goal=True,blend_percentage=0)

        while not handEyeCalib.is_arrived():
            rclpy.spin_once(handEyeCalib)
        
        for _ in range(200):
            rclpy.spin_once(handEyeCalib)
        print('arrived')
        T_W_G = np.eye(4)
        T_W_G[:3,:3] = R.from_euler('xyz',handEyeCalib.current_positions[3:], degrees=False).as_matrix()
        T_W_G[:3,3] = handEyeCalib.current_positions[:3]

        T_C_A,frame = handEyeCalib.detectTag()
        if T_C_A is not None:
            handEyeCalib.T_C_A_list.append(T_C_A)
            handEyeCalib.T_W_G_list.append(T_W_G)
            # handEyeCalib.get_logger().info("T_C_A: %s" % T_C_A)
            # handEyeCalib.get_logger().info("T_W_G: %s" % T_W_G)
        
        for i in range(len(handEyeCalib.T_C_A_list)):
            T_C_A = handEyeCalib.T_C_A_list[i]
            T_C_a = T_C_A @ T_A_a
            handEyeCalib.pub_tf(np.linalg.inv(T_C_a),'apriltag',f'cam{i}')
            T_W_G = handEyeCalib.T_W_G_list[i]

        T_G_C = np.eye(4)
        if len(handEyeCalib.T_C_A_list) > 2:
            r,t = handEyeCalib.HEC()
            T_G_C[:3,:3] = r
            T_G_C[:3,3] = t.reshape(3)
            T_W_a = T_W_G @ T_G_C @ T_C_A @ T_A_a
            handEyeCalib.get_logger().info("T_G_C: \n%s" % np.array2string(T_G_C,separator=','))
            print(R.from_matrix(T_G_C[:3,:3]).as_euler('xyz', degrees=True))
            handEyeCalib.get_logger().info("T_W_a: \n%s" % np.array2string(T_W_a,separator=','))
            print(R.from_matrix(T_W_a[:3,:3]).as_euler('xyz', degrees=True))
            handEyeCalib.pub_tf(T_G_C,'arm','camera')
            handEyeCalib.pub_tf(T_W_a,'world','apriltag')
       

        cv2.imshow('frame',frame)   
        cv2.waitKey(100)
        idx += 1
    T_a_W = np.linalg.inv(T_W_a)
    for i in range(len(handEyeCalib.T_C_A_list)):
        T_C_A = handEyeCalib.T_C_A_list[i]
        T_W_G = handEyeCalib.T_W_G_list[i]
        T = T_W_G @ T_G_C @ T_C_A @ T_A_a @ T_a_W
        handEyeCalib.get_logger().info("repjt rot err: %s" % R.from_matrix(T[:3,:3]).as_euler('xyz', degrees=True))
        handEyeCalib.get_logger().info("repjt pos err: %s\n" % (T[:3,3].reshape(3)))


    rclpy.shutdown()

def getTagPos():
    rclpy.init()
    handEyeCalib = HandEyeCalib()
    for _ in range(200):
        rclpy.spin_once(handEyeCalib)
    print("init done")
    T_W_G = np.eye(4)
    T_W_G[:3,:3] = R.from_euler('xyz',handEyeCalib.current_positions[3:], degrees=False).as_matrix()
    T_W_G[:3,3] = handEyeCalib.current_positions[:3]
    T_G_C = np.array(
        [[ 0.99991712, -0.01191628,  0.00487421,-0.00767288],
        [ 0.01069885,  0.97968111,  0.20027597, -0.06552731],
        [-0.00716172, -0.20020722,  0.9797274,   0.03914132],
        [ 0,          0,          0,          1,        ]]
    )
    T_C_A = handEyeCalib.detectTag()[0]
    T_W_a = T_W_G @ T_G_C @ T_C_A @ T_A_a
    handEyeCalib.get_logger().info("tag pos: %s\n" % np.array2string(T_W_a, separator=','))
    rclpy.shutdown()

if __name__ == '__main__':
    # main()
    getTagPos()
