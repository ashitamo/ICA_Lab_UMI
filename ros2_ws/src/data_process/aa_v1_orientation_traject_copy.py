import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

import numpy as np
import json
from pathlib import Path
import argparse 
import cv2
import os

from py_gripper_interfaces.msg import TrajState
from py_gripper_interfaces.srv import Trajectory
from scipy.spatial.transform import Rotation as R
import math

from cv_bridge import CvBridge
from collections import deque

# remember to check this path when putting in ros2 workspace
from xyz_pointcloud_final_five_shot import get_xy_xyz_for_grasp, get_xy_xyz_for_insert, c2e, rotGripper2Horizon 

'''
This program is used to execute the task workflow with orientation and trajectory control.

Using Trajectory!!!

'''
with open('/home/lab606/ICA_Lab_UMI/ICA_Lab_UMI_Config.yaml', 'r') as file:
    config_data = yaml.safe_load(file)
    T_G_C = np.array(config_data['T_G_C'])
    T_C_G = np.array(config_data['T_C_G'])
    T_G_E = np.array(config_data['T_G_E'])
    T_E_G = np.array(config_data['T_E_G'])
    T_W_a = np.array(config_data['T_W_a'])
    T_a_W = np.linalg.inv(T_W_a)
    T_a_A = np.array(config_data['T_a_A'])
    T_A_a = np.array(config_data['T_A_a'])
    T_E_C = T_E_G @ T_G_C
    T_C_E = np.linalg.inv(T_E_C)

# setup publisher
class aa_publisher(Node):
    def __init__(self):
        super().__init__('executor_node')
        # Publisher for the target pose as a list of 9 floats [x,y,z,qx,qy,qz,qw]
        self.pose_publisher = self.create_publisher(Float64MultiArray, '/target_pose_list', 10)
        '''
        # State tracking
        self.last_known_pose = Pose() # Start with a default pose
        self.last_known_gripper_width = 0.08 # Start with gripper open
        '''
        self.action_dispatcher = {
            "approach": self._execute_approach,
            "move": self._execute_move,
            "grasp": self._execute_grasp,
            "insert": self._execute_insert,
            "retreat": self._execute_move
        }
        self.get_logger().info("Executor Node Initialized. Ready to run mission.")
        
        self.calib = {'color': {'fx': 434.5713806152344, 'fy': 433.9692077636719,
                                      'ppx': 422.2452392578125, 'ppy': 241.36700439453125, 
                                      'width': 848, 'height': 480, }
                    }

        self.current_pose = None  # To track the current pose of the robot
    ########## Add publishers, subscribers, timers, etc. here
        self.subscription = self.create_subscription(TrajState,'/traj_state',self.data_callback,10)
        self.bridge = CvBridge()
        self.data = []
        
        self.exe_folder = os.path.dirname(workflow_path)
        self.action_clinet = self.create_client(Trajectory,'/target_pose_service')
        while not self.action_clinet.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        print("Service connected!")
        self.gripLen = 0.0295
    ########## Get image function
    def data_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg.color_image, desired_encoding='bgr8')
        idx = msg.idx
        depth = self.bridge.imgmsg_to_cv2(msg.depth_image, desired_encoding='passthrough')
        self.data.append([idx,img,depth])
        print(f"Get feedback id={idx} from /traj_state topic")

        #when full take away the old data and add in the new one to queue



    # call this function to publish pose data and wait for feedback
    def action_publish(self, command_list: list):
        """Takes an 8-element list, creates a Float32MultiArray message, and publishes it."""
        if len(command_list) != 9:
            self.get_logger().error(f"Invalid command list length. Expected 9, got {len(command_list)}.")
            return
        
        msg = Float64MultiArray()
        msg.data = [float(p) for p in command_list]
        # msg.data = array('d', [-0.185891, 0.068578, 0.398634, -0.591989, 0.685533, -0.340521, 0.252266, 0.165614])
        # msg = std_msgs.msg.Float64MultiArray(layout=std_msgs.msg.MultiArrayLayout(dim=[], data_offset=0), data=[-0.185891, 0.068578, 0.398634, -0.591989, 0.685533, -0.340521, 0.252266, 0.165614])
        
        id = command_list[0]
        self.get_logger().info(f"Publishing: Pose={msg.data} with ID={id}")
        # self.pose_publisher.publish(msg)
        req = Trajectory.Request()
        req.positions = msg.data[1:8]
        req.idx = int(msg.data[0])
        req.grip = msg.data[-1]
        self.action_clinet.call_async(req)
        self.save_traj(msg.data)  # Save the trajectory after publishing

    def save_traj(self, traj_state):
        # Create a file and save the trajectory data as list in a txt file
        # If the file exists, append to it. Delete all data at the first time.
        traj_file_path = os.path.join(self.exe_folder, 'rec_traj.txt')
        if not hasattr(self, '_traj_file_initialized'):
            # Delete all data at the first time
            open(traj_file_path, 'w').close()
            self._traj_file_initialized = True
        with open(traj_file_path, 'a') as f:
            f.write(','.join(map(str, traj_state)) + '\n')


    def wait_feedback(self, wait_for_id):
        while True:
            rclpy.spin_once(self)
            if len(self.data):
                field_data = self.data[0]
                if field_data[0] == wait_for_id:
                    print("Confirm action finish!")
                    self.data.pop(0)
                    return field_data[1], field_data[2]
                else:
                    self.data.pop(0)
                    print(f"id={field_data[0]}, Call back id not as expected")
            else:
                # print("Waiting task finish and return...")
                pass


    def _execute_approach(self, trajectory_list, demo_image_list):
        self.get_logger().info(f"Approaching to target pose...")
        if self.current_pose is not None:
            self.current_pose[0] = init_id
            self.action_publish(self.current_pose)
        # if len(trajectory_list) > 40: do not skip too much
        if len(trajectory_list) > 40:
            trajectory_list = trajectory_list[:-30]  # skip first 30 to avoid jitter
        else:
            trajectory_list = trajectory_list[:int(1.5*len(trajectory_list)/3)]
        sample_num = len(trajectory_list)
        for i in range (len(trajectory_list)):
            if trajectory_list[i][2] <= 0.25:
                trajectory_list[i][2] = 0.25
            target_pose = trajectory_list[i].copy()
            target_pose.insert(0, i+1)
            self.action_publish(target_pose)
        self.field_img_color, self.field_img_depth = self.wait_feedback(sample_num)
        self.current_pose = trajectory_list[-1].copy()
        self.current_pose.insert(0, 0)
        cv2.imshow("field_img_color", self.field_img_color)
        cv2.waitKey(16)

    def _execute_move(self, trajectory_list, demo_image_list):
        self.get_logger().info(f"Move to target pose...")
        if self.current_pose is not None:
            self.current_pose[0] = init_id
            self.action_publish(self.current_pose)

        if len(trajectory_list) > 40:
            trajectory_list = trajectory_list[30:]  # skip first 30 to avoid jitter
        else:
            trajectory_list = trajectory_list[int(len(trajectory_list)/3):]
        sample_num = len(trajectory_list)
        for i in range (len(trajectory_list)):
            if trajectory_list[i][2] <= 0.33:
                trajectory_list[i][2] = 0.33
            trajectory_list[i][-1] = 0.029
            target_pose = trajectory_list[i].copy()
            target_pose.insert(0, i+1)
            self.action_publish(target_pose)
        self.field_img_color, self.field_img_depth = self.wait_feedback(sample_num)
        self.current_pose = trajectory_list[-1].copy()
        self.current_pose.insert(0, 0)
        cv2.imshow("field_img_color", self.field_img_color)
        cv2.waitKey(16)
        pass


    def _execute_grasp(self, trajectory_list, demo_image_list):
        self.get_logger().info(f"Grasp to target pose...")
        if self.current_pose is not None:
            self.current_pose[0] = init_id
            self.action_publish(self.current_pose)
        else:
            self.current_pose = trajectory_list[0].copy()
            self.current_pose.insert(0, init_id)
        # Rotate camera to horizon before use field image to find object
        # Get the pose with gripper rotated to horizon
        print("Rotate camera to horizon")
        pose_7element = np.array(self.current_pose[1:-1])  #np.array([x,y,z,rx,ry,rz,rw])
        initial_pose_7element_horizon = rotGripper2Horizon(pose_7element)
        rotated_pose = [target_id] + initial_pose_7element_horizon.tolist() + [self.current_pose[-1]]
        rotated_pose_array = np.array(rotated_pose)

        # rotated_pose_array = self.rotCam2Horizon(self.current_pose)

        # Publish initial pose and rotated pose
        # self.action_publish(self.current_pose)
        self.action_publish(rotated_pose_array)
        # rotated_pose_array[0] = -1
        # self.action_publish(rotated_pose_array)
        self.field_img_color, self.field_img_depth = self.wait_feedback(target_id)
        # Update current pose
        self.current_pose = rotated_pose_array.copy()
        print("Current pose after rotating gripper to horizon:", self.current_pose)
        # Show feedback image
        cv2.imshow("field_img_color", self.field_img_color)
        cv2.waitKey(16)
        
        # Get the object's current location with SAM (Camera frame)
        xyz_c = get_xy_xyz_for_grasp(demo_image_list, self.field_img_color, self.field_img_depth, self.calib)
        #xyz_c = np.array(xyz_c) * 10  # convert to cm
        print("Current object position in camera frame:", xyz_c)
        # Convert to world frame
        grasp_pose_7element = np.array(self.current_pose[1:-1])  #np.array([x,y,z,rx,ry,rz,rw])
        print(grasp_pose_7element)
        xyz_w = c2e(xyz_c, grasp_pose_7element)   # np.array([x,y,z,rx,ry,rz,rw])
        print("Current object position in world frame:", xyz_w)

        # from 1 to 2, which is at grasp pose with gripper opening
        grasp_pose_temp = [target_id] + xyz_w.tolist() + [0.02]
        grasp_pose_temp[3] -= 0.02  # avoid hitting the table
        grasp_pose = np.array(grasp_pose_temp)
        self.current_pose[0] = init_id

        # Publish grasp pose
        self.action_publish(self.current_pose)
        self.action_publish(grasp_pose)
        self.field_img_color, self.field_img_depth = self.wait_feedback(target_id)
        # Update current pose
        self.current_pose = grasp_pose.copy()
        # Show feedback image
        cv2.imshow("field_img_color", self.field_img_color)
        cv2.waitKey(100)
        pass

        
        
    def _execute_insert(self, trajectory_list, demo_image_list):
        self.get_logger().info(f"Insert to target pose...")
        if self.current_pose is None:
            self.current_pose[0] = init_id
            self.action_publish(self.current_pose)

        # Rotate camera to horizon before use field image to find object
        # Get the pose with gripper rotated to horizon
        print("Rotate camera to horizon")

        rotated_pose_array = self.rotCam2Horizon(self.current_pose)
        rotated_pose_array[3] += 0.01  # lift up a bit to avoid hitting the table
        rotated_pose_array[1] += 0.05
        
        # Publish initial pose and rotated pose
        # self.action_publish(self.current_pose)
        rotated_pose_array[0] = target_id
        self.action_publish(rotated_pose_array)
        self.field_img_color, self.field_img_depth = self.wait_feedback(target_id)
        # Update current pose
        self.current_pose = rotated_pose_array.copy()
        # Show feedback image
        cv2.imshow("field_img_color", self.field_img_color)
        cv2.waitKey(16)
        
        # Get the object's current location with SAM (Camera frame)
        result = get_xy_xyz_for_insert(demo_image_list, self.field_img_color, self.field_img_depth, self.calib)
        
        # 檢查結果是否有效
        if result is None:
            self.get_logger().error("Failed to get result from get_xy_xyz_for_insert")
            return
        
        print(f"[Debug] result type: {type(result)}, length: {len(result) if hasattr(result, '__len__') else 'N/A'}")
        
        # get_xy_xyz_for_insert 可能返回：
        # 1. ((x,y,z), normal_vector) - tuple of 2 elements
        # 2. (x,y,z) - tuple of 3 elements (simple position)
        # 3. ((x,y,z), normal_vector, d_plane, stats) - tuple of 4 elements
        
        if isinstance(result, tuple):
            if len(result) == 2:
                # 可能是 ((x,y,z), normal_vector)
                first_elem = result[0]
                if isinstance(first_elem, tuple) and len(first_elem) == 3:
                    # 第一個元素是 (x,y,z)
                    xyz_c = first_elem
                    normal_vector = result[1]
                    print(f"[Debug] Extracted xyz_c: {xyz_c}, normal: {normal_vector}")
                else:
                    # 可能是錯誤格式
                    self.get_logger().error(f"Unexpected result format: {result}")
                    return
            elif len(result) == 3:
                # 直接是 (x,y,z)
                xyz_c = result
                normal_vector = None
                print(f"[Debug] xyz_c (3 elements): {xyz_c}")
            elif len(result) == 4:
                # ((x,y,z), normal_vector, d_plane, stats)
                xyz_c = result[0]
                normal_vector = result[1]
                print(f"[Debug] Extracted from 4-tuple xyz_c: {xyz_c}, normal: {normal_vector}")
            else:
                self.get_logger().error(f"Unexpected result length: {len(result)}")
                return
        else:
            xyz_c = result
            normal_vector = None

        self.cylinderLen = 0.085
        xyz_w_to_cam_pose = self.xyz_w_to_cam_pose(xyz_c, rotated_pose_array, self.cylinderLen)

        # Publish pre-insert pose
        pre_insert_pose = np.array(xyz_w_to_cam_pose)
        pre_insert_pose[0] = target_id
        self.insert(pre_insert_pose,rotated_pose_array)

    def xyz_w_to_cam_pose(self,xyz_c,current_pose,cylinderLen=0.09):
        # 確保 xyz_c 是有效的
        if xyz_c is None:
            raise ValueError("xyz_c is None")
        
        # 處理不同類型的輸入
        if isinstance(xyz_c, (tuple, list)):
            if len(xyz_c) != 3:
                raise ValueError(f"xyz_c should have 3 elements, got {len(xyz_c)}")
            p_c = np.array([xyz_c[0], xyz_c[1], xyz_c[2], 1.0])
        elif isinstance(xyz_c, np.ndarray):
            if xyz_c.shape[0] < 3:
                raise ValueError(f"xyz_c array should have at least 3 elements, got {xyz_c.shape[0]}")
            p_c = np.array([xyz_c[0], xyz_c[1], xyz_c[2], 1.0])
        else:
            raise TypeError(f"xyz_c has unsupported type: {type(xyz_c)}")

        T_a_c = np.eye(4)
        T_a_c[:3,:3] = R.from_quat(current_pose[4:8]).as_matrix()
        T_a_c[:3,3] = current_pose[1:4]
        p_a = T_a_c @ p_c
        # p_a[2] -= 0.0045
        # p_a[0] +=0.0004
        # p_a[1] -=0.002
        p_a[2] -= 0.0045
        p_a[0] +=0.00025
        p_a[1] -=0.0015
        print("Object position in arm frame:", p_a)

        T_a_p = np.eye(4)
        rot = R.from_quat(current_pose[4:8]).as_euler('xyz', degrees=True)
        rot = R.from_euler('xyz', [180.0, 0.0, rot[2]], degrees=True).as_quat()
        T_a_p = np.array([0.0,p_a[0],p_a[1],p_a[2],rot[0],rot[1],rot[2],rot[3],current_pose[-1]])
        print("Object position in arm frame with gripper rotated:", T_a_p)
        T_a_g = self.p2g(T_a_p,cylinderLen)
        T_a_c = self.g2c(T_a_g)
        return T_a_c
    
    def insert(self,holePos_A_C,lastPos_A_C):
        # temp = holePos_A_C.copy()
        # temp[3] += 0.05
        # self.action_publish(temp.tolist())
        # holePos_A_C[0] = 1.0

        # temp = holePos_A_C.copy()
        # temp[3] += 0.002
        # self.action_publish(temp.tolist())
        # # self.action_publish(holePos_A_C.tolist()) # pretouch
        # input("Press Enter to continue to insert...")
        # temp = holePos_A_C.copy()
        # temp[3] += 0.05
        # self.action_publish(temp.tolist())

        holePos_A_C[0] = 1.0
        holePos_W_C = self.a2w(holePos_A_C.copy())
        holePos_W_G = self.c2g(holePos_W_C.copy())

        lastPos_W_C = self.a2w(lastPos_A_C.copy())
        lastPos_W_G = self.c2g(lastPos_W_C.copy())

        holePosApproach, holePosTouch ,holePosFinal = self.plan_insert_traj(holePos_W_G,lastPos_W_G,r=0.09,cylinderLen=0.09)
        holePosApproach = self.w2a(holePosApproach.copy())
        holePosApproach = self.g2c(holePosApproach.copy())

        holePosTouch = self.w2a(holePosTouch.copy())
        holePosTouch = self.g2c(holePosTouch.copy())

        holePosFinal = self.w2a(holePosFinal.copy())
        holePosFinal = self.g2c(holePosFinal.copy())

        holePosApproach[0] = 0.0
        self.action_publish(holePosApproach.tolist())
        holePosTouch[0] = 0.0
        holePosTouch[-1] = self.gripLen
        self.action_publish(holePosTouch.tolist())
        holePosFinal[0] = 0.0
        holePosFinal[-1] = self.gripLen
        self.action_publish(holePosFinal.tolist())
        holePosFinal[0] = 0.0
        holePosFinal[3]-= 0.04
        holePosFinal[-1] = 0.085
        self.action_publish(holePosFinal.tolist())
        holePosFinal[0] = 0.0
        holePosFinal[3] = 0.35
        holePosFinal[-1] = 0.085
        self.action_publish(holePosFinal.tolist())
        holePosFinal[0] = -1.0
        holePosFinal[3] = 0.35
        holePosFinal[-1] = 0.085
        self.action_publish(holePosFinal.tolist())

        print(holePos_W_G)
        print(holePosApproach)
        print(holePosTouch)
        print(holePosFinal)
        print()

    def plan_insert_traj(self,holePos_W_G,seeholePos_W_G,r=0.1,th1=15.0,cylinderLen=0.09, c = -0.000,b=0.004):
        
        holePos_W_P = self.g2p(holePos_W_G,cylinderLen)

        x0,y0,z0 = seeholePos_W_G[1],seeholePos_W_G[2],seeholePos_W_G[3]
        x2,y2,z2 = holePos_W_P[1],holePos_W_P[2],holePos_W_P[3]
        
        th2 = math.atan2(x2 - x0, y2 - y0)
        th1 = math.radians(th1)

        print(th1,th2)

        rz = -math.degrees(th2)
        rx = math.degrees(th1)
        print(rx,-rz)
        l = r*math.sin(th1)
        
        x2,y2 = x2 + b*math.sin(th2), y2 + b*math.cos(th2)
        x1,y1,z1 = -l*math.sin(th2)+x2,  -l*math.cos(th2)+y2,  r*math.cos(th1)+z2
        holePos_W_P_approch = np.array([0.0, x1, y1, z1, 0.0, 0.0, 0.0, 1.0, self.gripLen])
        holePos_W_P_approch[4:8] = R.from_euler('xyz', [180.0+rx, 0.0, rz], degrees=True).as_quat()
        holePos_W_P[4:8] = R.from_euler('xyz', [180.0+rx, 0.0, rz], degrees=True).as_quat()
        holePos_W_P[1],holePos_W_P[2],holePos_W_P[3] = x2,y2,z2
        holePos_W_G_approch = self.p2g(holePos_W_P_approch,cylinderLen)
        holePos_W_G_touch = self.p2g(holePos_W_P,cylinderLen)

        print(l)
        print()
        holePos_W_G_final = holePos_W_G_touch.copy()
        holePos_W_G_final[1] = (l+c)*math.sin(th2)+holePos_W_G_touch[1]
        holePos_W_G_final[2] = (l+c)*math.cos(th2)+holePos_W_G_touch[2]
        holePos_W_G_final[3] = holePos_W_G_touch[3] - 0.005
        holePos_W_E_final = self.g2e(holePos_W_G_final)
        holePos_W_E_final[4:8] = R.from_euler('xyz', [180.0, 0.0, rz], degrees=True).as_quat()
        holePos_W_G_final = self.e2g(holePos_W_E_final)

        return holePos_W_G_approch, holePos_W_G_touch ,holePos_W_G_final
    
    def rotCam2Horizon(self,T_a_ci):
        '''
        T_a_ci : np.array([idx,x,y,z,rx,ry,rz,rw,grip]) camera frame
        =>
        T_a_ci : np.array([idx,x,y,z,rx,ry,rz,rw,grip]) gripper horizon frame
        '''
        '''
        if let cam horizontal
            => cal to p rot
        '''
        T_a_g = self.c2g(T_a_ci.copy())
        T_a_p = self.g2p(T_a_g.copy(),0.09)
        px,py,pz = T_a_p[1],T_a_p[2],T_a_p[3]
        rot = R.from_quat(T_a_ci[4:8]).as_euler('xyz', degrees=True)
        T_a_ci[4:8] = R.from_euler('xyz', [180.0, 0.0, 90], degrees=True).as_quat()
        T_a_g = self.c2g(T_a_ci.copy())
        T_a_p = self.g2p(T_a_g.copy(),0.09)
        T_a_p = np.array([T_a_ci[0],px,py,pz,T_a_p[4],T_a_p[5],T_a_p[6],T_a_p[7],T_a_ci[8]])
        T_a_g = self.p2g(T_a_p.copy(),0.09)
        T_a_ci = self.g2c(T_a_g.copy())
        return T_a_ci
    

    def g2c(self,p):
        T_x_G = np.eye(4)
        T_x_G[:3,:3] = R.from_quat(p[4:8]).as_matrix()
        T_x_G[:3,3] = np.array(p[1:4])
        T_x_C = T_x_G @ T_G_C
        p[1] = T_x_C[0,3]
        p[2] = T_x_C[1,3]
        p[3] = T_x_C[2,3]
        p[4:8] = R.from_matrix(T_x_C[:3,:3]).as_quat()
        return p
    
    def c2g(self,p):
        T_x_C = np.eye(4)
        T_x_C[:3,:3] = R.from_quat(p[4:8]).as_matrix()
        T_x_C[:3,3] = np.array(p[1:4])
        T_x_G = T_x_C @ T_C_G
        p[1] = T_x_G[0,3]
        p[2] = T_x_G[1,3]
        p[3] = T_x_G[2,3]
        p[4:8] = R.from_matrix(T_x_G[:3,:3]).as_quat()
        return p
    
    def a2w(self,p):
        T_a_x = np.eye(4)
        T_a_x[:3,:3] = R.from_quat(p[4:8]).as_matrix()
        T_a_x[:3,3] = np.array(p[1:4])
        T_W_x = T_W_a @ T_a_x
        p[1] = T_W_x[0,3]
        p[2] = T_W_x[1,3]
        p[3] = T_W_x[2,3]
        p[4:8] = R.from_matrix(T_W_x[:3,:3]).as_quat()
        return p
    
    def w2a(self,p):
        T_W_x = np.eye(4)
        T_W_x[:3,:3] = R.from_quat(p[4:8]).as_matrix()
        T_W_x[:3,3] = np.array(p[1:4])
        T_a_x = T_a_W @ T_W_x
        p[1] = T_a_x[0,3]
        p[2] = T_a_x[1,3]
        p[3] = T_a_x[2,3]
        p[4:8] = R.from_matrix(T_a_x[:3,:3]).as_quat()
        return p
    
    def p2g(self,p,len):
        T_W_P = np.eye(4)
        T_W_P[:3,:3] = R.from_quat(p[4:8]).as_matrix()
        T_W_P[:3,3] = np.array(p[1:4])
        T_P_E = np.array(
            [[ 1, 0, 0, 0],
            [ 0, 1, 0, 0],
            [ 0, 0, 1, -len],
            [ 0, 0, 0, 1]]
        )
        T_W_G = T_W_P @ T_P_E @ T_E_G
        p[1] = T_W_G[0,3]
        p[2] = T_W_G[1,3]
        p[3] = T_W_G[2,3]
        p[4:8] = R.from_matrix(T_W_G[:3,:3]).as_quat()
        return p

    def g2p(self,p,len):
        T_W_G = np.eye(4)
        T_W_G[:3,:3] = R.from_quat(p[4:8]).as_matrix()
        T_W_G[:3,3] = np.array(p[1:4])
        T_E_P = np.array(
            [[ 1, 0, 0, 0],
            [ 0, 1, 0, 0],
            [ 0, 0, 1, len],
            [ 0, 0, 0, 1]]
        )
        T_W_E = T_W_G @ T_G_E @ T_E_P
        p[1] = T_W_E[0,3]
        p[2] = T_W_E[1,3]
        p[3] = T_W_E[2,3]
        p[4:8] = R.from_matrix(T_W_E[:3,:3]).as_quat()
        return p
    
    def g2e(self,p):
        T_x_G = np.eye(4)
        T_x_G[:3,:3] = R.from_quat(p[4:8]).as_matrix()
        T_x_G[:3,3] = np.array(p[1:4])
        T_x_E = T_x_G @ T_G_E
        p[1] = T_x_E[0,3]
        p[2] = T_x_E[1,3]
        p[3] = T_x_E[2,3]
        p[4:8] = R.from_matrix(T_x_E[:3,:3]).as_quat()
        return p
    
    def e2g(self,p):
        T_x_E = np.eye(4)
        T_x_E[:3,:3] = R.from_quat(p[4:8]).as_matrix()
        T_x_E[:3,3] = np.array(p[1:4])
        T_x_G = T_x_E @ T_E_G
        p[1] = T_x_G[0,3]
        p[2] = T_x_G[1,3]
        p[3] = T_x_G[2,3]
        p[4:8] = R.from_matrix(T_x_G[:3,:3]).as_quat()
        return p

    def get_reference_images(self, demo_image_list, sample_num):
        reference_images_list = []
        if len(demo_image_list) <= sample_num:
            images_to_load = demo_image_list
        else:
            indices = np.linspace(0, len(demo_image_list) - 1, sample_num, dtype=int)
            images_to_load = [demo_image_list[i] for i in indices]
        print("Reference images to load:", images_to_load)
        # 讀取影像
        for path in images_to_load:
            demo_image = cv2.imread(path)
            if demo_image is not None:
                reference_images_list.append(demo_image)
            else:
                self.get_logger().warn(f"  _sample_images_evenly: 找不到影像: {path}")
        return reference_images_list



    # --- Main Mission Runner ---
    def run_workflow(self, opened_workflow_json_file):
        """Main logic to read the plan and execute actions using the dispatcher."""
        
        data = opened_workflow_json_file

        # set start point and end point ids
        global init_id, target_id
        init_id = 0
        target_id = -1

        for pt in data.get("primitive_tasks", []):
            pt_id = pt.get("task_id", [])
            atomic_actions_list = pt.get("atomic_actions", [])
            for i in range(len(atomic_actions_list)):
                act = atomic_actions_list[i]
                next_act = atomic_actions_list[i+1] if i + 1 < len(atomic_actions_list) else None
                
                action_name = str(act.get("action", "")).lower()
                self.get_logger().info(f"--- Executing Action: {action_name.upper()} ---")
                params = act.get("parameters", {})
                trajectory_list = params.get("trajectory", [])
                current_demo_image_path_list = params.get("image_path")

                if next_act is not None:
                    next_params = next_act.get("parameters", {})
                    next_demo_image_path_list = next_params.get("image_path", [])
                    # Combine current and next action image paths

                demo_image_list = []
                current_referencce_num = 3
                next_referencce_num = 2

                demo_image_list = self.get_reference_images(current_demo_image_path_list, current_referencce_num)
                demo_image_list += self.get_reference_images(next_demo_image_path_list, next_referencce_num)

                if len(demo_image_list) == 0:
                    self.get_logger().warn(f"缺少影像資料，跳過此動作。")
                    continue

                # check action is on dispatcjer
                action_function = self.action_dispatcher.get(action_name)
                
                if action_function:
                    action_function(trajectory_list, demo_image_list)
                else:
                    self.get_logger().warn(f"Unknown action '{action_name}'. Skipping.")

            print(f"Primitive task id= {pt_id} completed.")



def main():
    # Set up command-line argument parsing
    # python aa_v1.py exe_data/task_workflow.json 
    global workflow_path, workflow_dir
    workflow_path = "/home/lab606/ICA_Lab_UMI/UMI_exe_12/task_workflow.json"
    #workflow_path = "exe_data3/task_workflow.json
    #workflow_path = "/home/lab606/ICA_Lab_UMI/ros2_ws/src/data_process/exe_data3/task_workflow_2.json"
    # Initialize ROS 2
    rclpy.init()

    # Open and read the workflow JSON file
    json_path = Path(workflow_path)
    workflow_dir = os.path.dirname(workflow_path)   #exe_data3
    print(f"Workflow directory: {workflow_dir}")

    if not json_path.exists():
        print(f"JSON file not found at {json_path}")
        return
    with json_path.open("r", encoding="utf-8") as f:
        workflow = json.load(f)

    # Create an instance of your executor node
    executor_node = aa_publisher()

    try:
        executor_node.run_workflow(workflow)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanly destroy the node and shut down ROS 2
        executor_node.destroy_node()
        rclpy.shutdown()
    
       

if __name__ == "__main__":
    

    # Call the main function with the parsed argument
    main()

