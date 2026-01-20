import math
from apriltag_grid_pose import AprilGridPoseEstimator
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
import yaml

T_a_A =  np.array([[ 0, 1, 0, 0],
                  [ 1, 0, 0 ,0],
                  [ 0, 0, -1, 0],
                  [ 0, 0,  0 ,1]])
T_A_a = np.linalg.inv(T_a_A)

T_G_E = np.array(
    [[ 1, 0, 0, 0],
    [ 0, 1, 0, 0],
    [ 0, 0, 1, 0.164],
    [ 0, 0, 0, 1]]
)
T_E_G = np.linalg.inv(T_G_E)

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
        # AprilGrid parameters
        self.tag_size = 0.022
        self.spacing_ratio = 0.3


        self.april_tag_size = 0.14142
        self.april_tag_size = 0.05012
        # self.april_tag_size = 0.1325

        self.new_mtx, self.roi = cv2.getOptimalNewCameraMatrix(CAM_MTX, DIST_COEFFS, (848,480), 1, (848,480))

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

        self.queue = queue.Queue(maxsize=2)
        self._sub = self.create_subscription(
            Image,
            '/camera/color/image_rect_raw',
            self.readCam,
            10
        )

        self.T_W_G_list = []
        self.T_C_A_list = []
    def pos_callback(self,msg):
        self.current_positions = msg.tool_pose
        # self.get_logger().info("Current Position: %s" % self.current_positions)

    def is_arrived(self,error=0.001):
        if sum((self.target_positions[i]-self.current_positions[i])**2 for i in range(3)) > error**2:
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

    def detectGrid(self):
        frame = self.queue.queue[-1]
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        
        estimator = AprilGridPoseEstimator(self.tag_size,self.spacing_ratio,CAM_MTX,DIST_COEFFS)
        results,frame = estimator.estimate_pose(frame,visualize=True)
        return results,frame


    def detectTag(self):
        frame = self.queue.queue[-1]
        frame = cv2.undistort(frame, CAM_MTX, DIST_COEFFS, None, self.new_mtx)
        x, y, w, h = self.roi
        frame = frame[y:y+h, x:x+w]
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        newcamera_params = [self.new_mtx[0,0], self.new_mtx[1,1], self.new_mtx[0,2], self.new_mtx[1,2]]
        print(newcamera_params)
        results = self.at_detector.detect(frame,True,newcamera_params,self.april_tag_size)

        cv2.circle(frame,(int(newcamera_params[2]),int(newcamera_params[3])),5,(0,0,255),-1)
        if len(results) != 0:
            # print(results[0])
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
    
    
    def detectBoard(self):
        """
        回傳：
            - T_C_B: 4x4 齊次矩陣（棋盤 B 在相機 C 座標系下的位姿）
            - frame: 標註後影像（便於除錯）
        若偵測失敗，回傳 (None, frame)
        """

        def make_chessboard_object_points(cols: int, rows: int, square_size: float, center_origin: bool = True):
            """
            產生棋盤 3D 角點（Z=0）：
            - 若 center_origin=True，則以棋盤幾何中心為原點（手眼時較直觀）
            - 若 False，則以 (0,0) 角點為原點（OpenCV 標準慣例）
            """
            objp = np.zeros((rows * cols, 3), dtype=np.float32)
            # 先以左上角為原點排點（OpenCV 慣例）
            grid = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
            objp[:, :2] = grid * square_size

            if center_origin:
                # 平移到幾何中心為 (0,0)
                center = np.array([(cols - 1) * square_size / 2.0,
                                (rows - 1) * square_size / 2.0], dtype=np.float32)
                objp[:, 0:2] -= center
            return objp

        frame = self.queue.get()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame

        pattern_size = (CHESSBOARD_COLS, CHESSBOARD_ROWS)  # (cols, rows)
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FAST_CHECK | cv2.CALIB_CB_NORMALIZE_IMAGE
        found, corners = cv2.findChessboardCorners(gray, pattern_size, flags=flags)

        if not found:
            return None, frame

        # 角點亞像素化
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        # 建立棋盤在自身座標的 3D 點
        object_points = make_chessboard_object_points(CHESSBOARD_COLS, CHESSBOARD_ROWS, SQUARE_SIZE, center_origin=True)

        # PnP 解姿態（棋盤相對相機）
        ok, rvec, tvec = cv2.solvePnP(object_points, corners_refined, CAM_MTX, DIST_COEFFS, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            return None, frame

        # 轉齊次矩陣
        R_cb, _ = cv2.Rodrigues(rvec)  # 3x3
        T_C_B = np.eye(4, dtype=np.float64)
        T_C_B[:3, :3] = R_cb
        T_C_B[:3,  3] = tvec.reshape(3)

        # 視覺化（畫出角點與座標軸）
        cv2.drawChessboardCorners(frame, pattern_size, corners_refined, found)

        # 畫一個簡單的 3D 軸（長度 = 3*square_size）
        axis_len = 3 * SQUARE_SIZE
        axis = np.float32([[0, 0, 0],
                        [axis_len, 0, 0],
                        [0, axis_len, 0],
                        [0, 0, axis_len]]).reshape(-1, 3)
        imgpts, _ = cv2.projectPoints(axis, rvec, tvec, CAM_MTX, DIST_COEFFS)
        imgpts = imgpts.astype(int)

        # 畫線：原點->X(藍)、原點->Y(綠)、原點->Z(紅)
        o = tuple(imgpts[0].ravel())
        cv2.line(frame, o, tuple(imgpts[1].ravel()), (255, 0, 0), 2)
        cv2.line(frame, o, tuple(imgpts[2].ravel()), (0, 255, 0), 2)
        cv2.line(frame, o, tuple(imgpts[3].ravel()), (0, 0, 255), 2)

        return T_C_B, frame



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
        return cv2.calibrateHandEye(R_W_G, t_W_G, R_C_A, t_C_A, None, cv2.CALIB_HAND_EYE_TSAI)

import math
import numpy as np
from typing import List, Tuple
from scipy.spatial.transform import Rotation as R

def generate_square_poses_faceZ_fixed_yaw(
    center_xy: Tuple[float, float] = (-0.15, -0.50),  # 正方形中心 (cx, cy)
    side: float = 0.24,                                # 邊長 (m)
    z_list: List[float] = [0.30],                      # 一次輸出多個高度層
    yaw_fixed: float = math.pi,                        # 你想要的固定 yaw（弧度）
    center_z: float = None,                            # 中心點 z；若為 None 則使用各點自己的 z（僅朝向 XY 中心）
    order: str = "snake"                               # "raster" 或 "snake" 走訪
) -> List[List[float]]:
    """
    產生 3×3 網格 (共9點) × len(z_list) 的姿態：
      - 位置落在以 (cx, cy) 為中心、邊長 side 的 3×3 格點
      - 工具 +Z 軸指向正方形中心點 (cx, cy, center_z)
      - 最後再沿著工具 +Z 軸旋轉 yaw_fixed（不改變指向）
      - 回傳 [x, y, z, roll, pitch, yaw]（XYZ intrinsic，弧度）
    """
    cx, cy = center_xy
    # 3×3：{-0.5*side, 0, +0.5*side}
    offs = [-0.5 * side, 0.0, 0.5 * side]

    # 先建立一層的 (x,y) 走訪序列
    xy_seq = []
    for j, dy in enumerate(offs):
        row = [(cx + dx, cy + dy) for dx in offs]
        if order == "snake" and (j % 2 == 1):
            row.reverse()
        xy_seq.extend(row)

    poses = []
    for z in z_list:
        cz = z if center_z is None else center_z  # 若沒指定 center_z，就只在 XY 上指向中心
        center_world = np.array([cx, cy, cz], dtype=float)

        for (x, y) in xy_seq:
            p = np.array([x, y, z], dtype=float)

            # 指向向量：由當前點指向中心
            v = center_world - p
            norm = np.linalg.norm(v)
            if norm < 1e-9:
                # 萬一剛好在中心上，給個預設 z+ 指向
                z_axis = np.array([0.0, 0.0, 1.0])
            else:
                z_axis = v / norm  # 工具 +Z 在世界座標的方向

            # 建立一組正交基底，使第三列(軸)是 z_axis
            # 先選一個不共線的參考向量
            ref = np.array([1.0, 0.0, 0.0])
            if abs(np.dot(ref, z_axis)) > 0.95:
                ref = np.array([0.0, 1.0, 0.0])

            x_axis = ref - np.dot(ref, z_axis) * z_axis
            x_n = np.linalg.norm(x_axis)
            if x_n < 1e-9:
                # 退化保護
                ref = np.array([0.0, 1.0, 0.0])
                x_axis = ref - np.dot(ref, z_axis) * z_axis
                x_n = np.linalg.norm(x_axis)
            x_axis /= x_n
            y_axis = np.cross(z_axis, x_axis)  # 右手座標

            # 這是「+Z 指向中心」的基礎旋轉 R0（列向量為世界到工具？我們要世界R^tool）
            # 我們希望 R 的列向量是工具軸在世界座標的表示 => R = [x_axis | y_axis | z_axis]
            R0 = np.column_stack([x_axis, y_axis, z_axis])  # 3x3

            # 沿工具 +Z 軸再轉 yaw_fixed（不改變指向）
            Rz = np.array([[ math.cos(yaw_fixed), -math.sin(yaw_fixed), 0.0],
                           [ math.sin(yaw_fixed),  math.cos(yaw_fixed), 0.0],
                           [ 0.0,                  0.0,                 1.0]])
            Rw = R0 @ Rz

            # 轉成 XYZ intrinsic Euler（和你既有程式一致）
            roll, pitch, yaw = R.from_matrix(Rw).as_euler('xyz', degrees=False)

            poses.append([float(x), float(y), float(z),
                          float(roll), float(pitch), float(yaw)])
    return poses



# === Chessboard config ===
# 棋盤內角點數（內點、交點數）：cols x rows，例如 9x6 代表每列9點、每行6點
CHESSBOARD_COLS = 10
CHESSBOARD_ROWS = 6
SQUARE_SIZE = 0.0254  # 每個小方格邊長（公尺）

camera_params = [434.5713806152344, 433.9692077636719, 422.2452392578125, 241.36700439453125]
# # OpenCV 預設 k1,k2,p1,p2,k3(,k4,k5,k6)；若你有Realsense標定檔可以填進來
DIST_COEFFS = np.array(
    [-0.054195329546928406, 0.06144030764698982, 8.257640001829714e-05, 0.0007601580582559109,-0.02074556238949299],
    dtype=np.float64
)

# 相機內參（fx, fy, cx, cy）與畸變係數（若未知可先全0）
fx, fy, cx, cy = camera_params
CAM_MTX = np.array([[fx, 0, cx],
                    [0, fy, cy],
                    [0,  0,  1]], dtype=np.float64)

def main():
    ## hand-eye calibration positions
    pos = [[-0.00, -0.38, 0.3, 3.14159, 0.0, 3.14159]]
    z_list = [0.25+i*0.01 for i in range(11)]
    first_angle = (-90)*math.pi/180.0
    last_angle = (180)*math.pi/180.0
    import random
    for i,z in enumerate(z_list):
        yaw = first_angle + i*(last_angle-first_angle)/len(z_list)
        temp = generate_square_poses_faceZ_fixed_yaw(
            center_xy = (-0.0, -0.38),  # 正方形中心 (cx, cy)
            side = 0.25,
            z_list = [z],   # 一次產兩層高度
            yaw_fixed = yaw,     # 你的指定 yaw（不會被自動更動）
            center_z = 0.0,         # 只在 XY 指向中心；若想抬頭/低頭指中心某高度，填一個固定數值
            order = "snake"
        )
        pos.extend(temp)
    print(pos)
    rclpy.init()
    handEyeCalib = HandEyeCalib()
    handEyeCalib.april_tag_size = 0.14142
    rclpy.spin_once(handEyeCalib)
    idx = 0
    while True:
        print("move to position idx:",idx,"/",len(pos))
        if idx >= len(pos):
            break
        print("current positions: ",handEyeCalib.current_positions)
        positions = pos[idx]
        handEyeCalib.send_request(positions,fine_goal=True,blend_percentage=0,velocity=0.2)

        while not handEyeCalib.is_arrived(0.01):
            # print(handEyeCalib.current_positions)
            rclpy.spin_once(handEyeCalib, timeout_sec=0.1)
        
        for _ in range(200):
            rclpy.spin_once(handEyeCalib)
        print('arrived')
        T_W_G = np.eye(4)
        T_W_G[:3,:3] = R.from_euler('xyz',handEyeCalib.current_positions[3:], degrees=False).as_matrix()
        T_W_G[:3,3] = handEyeCalib.current_positions[:3]
        
        T_C_A,frame = handEyeCalib.detectTag()
        # T_C_A,frame = handEyeCalib.detectGrid()
        # T_C_A,frame = handEyeCalib.detectBoard()
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

    roterr = []
    poserr = []
    for i in range(len(handEyeCalib.T_C_A_list)):
        T_C_A = handEyeCalib.T_C_A_list[i]
        T_W_G = handEyeCalib.T_W_G_list[i]
        T = T_W_G @ T_G_C @ T_C_A @ T_A_a @ T_a_W
        rot = R.from_matrix(T[:3,:3]).as_euler('xyz', degrees=True)
        handEyeCalib.get_logger().info("repjt rot err: {}".format(np.array2string(rot,separator=',',precision=6)))
        handEyeCalib.get_logger().info("repjt pos err: {}\n".format(np.array2string(T[:3,3],separator=',',precision=6)))
        roterr.append(rot)
        poserr.append(T[:3,3])
    roterr = np.array(roterr)
    poserr = np.array(poserr)
    handEyeCalib.get_logger().info("mean rot err: {}".format(np.array2string(np.mean(np.abs(roterr),axis=0),separator=',',precision=6)))
    handEyeCalib.get_logger().info("std rot err: {}".format(np.array2string(np.std(np.abs(roterr),axis=0),separator=',',precision=6)))
    handEyeCalib.get_logger().info("mean pos err: {}".format(np.array2string(np.mean(np.abs(poserr),axis=0),separator=',',precision=6)))
    handEyeCalib.get_logger().info("std pos err: {}".format(np.array2string(np.std(np.abs(poserr),axis=0),separator=',',precision=6)))
    
    import pickle
    filename = 'calib_results{}.pkl'.format(counter:=len([name for name in os.listdir('.') if os.path.isfile(name) and name.startswith('calib_results') and name.endswith('.pkl')]))
    with open(filename,'wb') as f:
        pickle.dump([handEyeCalib.T_W_G_list,handEyeCalib.T_C_A_list,T_G_C,T_a_W],f)
    
     # Write to YAML file
    with open('ICA_Lab_UMI_Config.yaml', 'r') as file:
        config_data = yaml.safe_load(file)
    config_data['T_A_a'] = T_A_a.tolist()
    config_data['T_a_A'] = T_a_A.tolist()
    config_data['T_G_C'] = T_G_C.tolist()
    config_data['T_C_G'] = np.linalg.inv(T_G_C).tolist()
    with open('ICA_Lab_UMI_Config.yaml', 'w') as file:
        yaml.dump(config_data, file, default_flow_style=False)

    rclpy.shutdown()


def getTagPos():
    # mean april tag
    pos = [[0.28, -0.33, 0.20, 3.14159, 0.0, -1.5708]]
    z_list = [0.20+i*0.04 for i in range(3)]
    first_angle = (180-40)*math.pi/180.0
    last_angle = (180+40)*math.pi/180.0
    import random
    for i,z in enumerate(z_list):
        yaw = first_angle + i*(last_angle-first_angle)/len(z_list)
        temp = generate_square_poses_faceZ_fixed_yaw(
            center_xy = (pos[0][0], pos[0][1]),  # 正方形中心 (cx, cy)
            side = 0.1,
            z_list = [z],   # 一次產兩層高度
            yaw_fixed = yaw,     # 你的指定 yaw（不會被自動更動）
            center_z = 0.0,         # 只在 XY 指向中心；若想抬頭/低頭指中心某高度，填一個固定數值
            order = "snake"
        )
        pos.extend(temp[3:])
    print(pos)
    # Load from YAML file
    with open('ICA_Lab_UMI_Config.yaml', 'r') as file:
        config_data = yaml.safe_load(file)
        T_G_C = np.array(config_data['T_G_C'])
        T_C_G = np.array(config_data['T_C_G'])

    rclpy.init()
    handEyeCalib = HandEyeCalib()
    handEyeCalib.april_tag_size = 0.05012
    print("init done")
    idx = 0
    T_W_a_list = []
    while idx<len(pos):
        print("move to position idx:",idx,"/",len(pos))
        print("current positions: ",handEyeCalib.current_positions)
        positions = pos[idx]
        handEyeCalib.send_request(positions,fine_goal=True,blend_percentage=0,velocity=0.1)
        while not handEyeCalib.is_arrived(0.01):
            # print(handEyeCalib.current_positions)
            rclpy.spin_once(handEyeCalib, timeout_sec=0.1)
        idx += 1
        for _ in range(200):
            rclpy.spin_once(handEyeCalib)
        print('arrived')
        
        T_W_G = np.eye(4)
        T_W_G[:3,:3] = R.from_euler('xyz',handEyeCalib.current_positions[3:], degrees=False).as_matrix()
        T_W_G[:3,3] = handEyeCalib.current_positions[:3]

        T_C_A,frame = handEyeCalib.detectTag()
        
        cv2.imshow('frame',frame)
        cv2.waitKey(100)
        if T_C_A is None:
            print("no tag detected, please adjust the pose")
            continue
        T_W_a = T_W_G @ T_G_C @ T_C_A @ T_A_a
        T_W_a_list.append(T_W_a)
        handEyeCalib.get_logger().info("tag pos: \n%s" % np.array2string(T_W_a, separator=','))


    rclpy.shutdown()
    T_W_a = np.mean(np.array(T_W_a_list),axis=0)
    T_a_W = np.linalg.inv(T_W_a)
    print("T_W_a: \n",T_W_a)
    print("T_a_W: \n",T_a_W)

    # Write to YAML file
    with open('ICA_Lab_UMI_Config.yaml', 'r') as file:
        config_data = yaml.safe_load(file)
    config_data['T_A_a'] = T_A_a.tolist()
    config_data['T_a_A'] = T_a_A.tolist()
    config_data['T_W_a'] = T_W_a.tolist()
    config_data['T_a_W'] = T_a_W.tolist()
    config_data['T_G_E'] = T_G_E.tolist()
    config_data['T_E_G'] = T_E_G.tolist()
    with open('ICA_Lab_UMI_Config.yaml', 'w') as file:
        yaml.dump(config_data, file, default_flow_style=False)


if __name__ == '__main__':
    # main()
    getTagPos()

'''
[[-0.01433173, 0.99963671, 0.02282667,-0.34401851],
 [-0.99989453,-0.01438161, 0.00202218,-0.65156833],
 [ 0.00234972,-0.02279528, 0.99973739, 0.01457564],
 [ 0.        , 0.        , 0.        , 1.        ]]

 [[-0.01156194, 0.99988935, 0.00936036,-0.34356403],
 [-0.99990892,-0.01162633, 0.00685366,-0.65177863],
 [ 0.00696173,-0.00928026, 0.9999327 , 0.01568898],
 [ 0.        , 0.        , 0.        , 1.        ]]


 [[-0.01553806, 0.99919739, 0.03692071,-0.34480372],
 [-0.99986662,-0.01571304, 0.0044538 ,-0.65188919],
 [ 0.00503036,-0.03684658, 0.99930827, 0.01596489],
 [ 0.        , 0.        , 0.        , 1.        ]]

 [[-0.0110116 , 0.99992776,-0.0048186 ,-0.34385463],
 [-0.999871  ,-0.01095437, 0.01174702,-0.65254015],
 [ 0.01169339, 0.00494733, 0.99991939, 0.01354095],
 [ 0.        , 0.        , 0.        , 1.        ]]

 

'''
