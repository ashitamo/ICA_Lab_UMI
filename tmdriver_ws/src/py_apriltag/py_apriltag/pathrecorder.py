import math
from nav_msgs.msg import Path
from geometry_msgs.msg import TransformStamped,PoseStamped
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

class RealtimeLowPassFilterQuat:
    def __init__(self, alpha: float):
        """
        一階低通 (quaternion, [x,y,z,w])
        用 SLERP 濾波，含「同半球對齊」避免±q跳號問題。
        alpha ∈ [0,1]，越小越平滑（反應越慢）；越大越跟新值
        """
        if not (0.0 <= alpha <= 1.0):
            raise ValueError("Alpha must be between 0 and 1.")
        self.alpha = alpha
        self.initialized = False
        self.filtered_q = None  # np.ndarray shape (4,)

    @staticmethod
    def _normalize(q: np.ndarray) -> np.ndarray:
        q = np.asarray(q, dtype=float).reshape(4)
        n = np.linalg.norm(q)
        if n == 0.0:
            # 避免 0 四元數
            return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        return q / n

    @staticmethod
    def _slerp(q1: np.ndarray, q2: np.ndarray, t: float) -> np.ndarray:
        # q = [x, y, z, w]
        q1 = RealtimeLowPassFilterQuat._normalize(q1)
        q2 = RealtimeLowPassFilterQuat._normalize(q2)

        dot = float(np.dot(q1, q2))
        # 同半球對齊，避免走遠路與號誌翻轉
        if dot < 0.0:
            q2 = -q2
            dot = -dot

        DOT_THRESH = 0.9995
        if dot > DOT_THRESH:
            # 幾乎平行：線性插值 + normalize
            out = q1 + t * (q2 - q1)
            return RealtimeLowPassFilterQuat._normalize(out)

        theta0 = np.arccos(np.clip(dot, -1.0, 1.0))   # 兩四元數夾角
        sin_theta0 = np.sin(theta0)
        theta = theta0 * t

        s1 = np.sin(theta0 - theta) / sin_theta0
        s2 = np.sin(theta) / sin_theta0
        return s1 * q1 + s2 * q2

    def update(self, new_sample) -> np.ndarray:
        """
        new_sample: iterable/list/np.ndarray 長度4，順序 [x, y, z, w]
        回傳: 濾後四元數 np.ndarray(4,)
        """
        q_new = self._normalize(np.asarray(new_sample, dtype=float))
        if not self.initialized:
            self.filtered_q = q_new
            self.initialized = True
            return self.filtered_q

        # 與原本 scalar LPF 相同介面：用 alpha 當「往新樣本靠近的比例」
        self.filtered_q = self._slerp(self.filtered_q, q_new, self.alpha)
        return self.filtered_q

class RealtimeLowPassFilter:
    def __init__(self, alpha):
        """
        Initializes a first-order low-pass filter.
        :param alpha: Filter coefficient, between 0 and 1.
                      Higher alpha means more smoothing (lower cutoff frequency).
        """
        if not (0 <= alpha <= 1):
            raise ValueError("Alpha must be between 0 and 1.")
        self.alpha = alpha
        self.filtered_value = 0.0

    def update(self, new_sample):
        """
        Processes a new sample and updates the filtered value.
        :param new_sample: The new data point to filter.
        :return: The current filtered value.
        """
        self.filtered_value = self.alpha * new_sample + (1 - self.alpha) * self.filtered_value
        return self.filtered_value
    
def quaternion_from_euler(ai, aj, ak):
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

camera_params = [839.72913173,835.51579792,311.87321929,255.38762288]
# camera_params = [839.72913173*2,835.51579792*2,311.87321929*2,255.38762288*2]

# 170 -415 190 mm 180 0 -90
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

T_C_c = np.array([[ -1, 0, 0, 0],
                  [ 0, 1, 0 ,0],
                  [ 0, 0, -1 ,0],
                  [ 0, 0,  0 ,1]])

T_c_C = np.linalg.inv(T_C_c)

R_W_G = np.array([[ -1, 0, 0,],
                  [ 0, -1, 0],
                  [ 0, 0, 1]]) 
R_G_W = np.linalg.inv(R_W_G)
class FramePublisher(Node):

    def __init__(self):
        super().__init__('pose_publisher')
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.at_detector = Detector(searchpath=['apriltags'],
                       families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        # self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, 120)

        aplha = 0.2
        self.lpf_q = RealtimeLowPassFilterQuat(aplha)

        self.lpf_px = RealtimeLowPassFilter(aplha)
        self.lpf_py = RealtimeLowPassFilter(aplha)
        self.lpf_pz = RealtimeLowPassFilter(aplha)


        self.path = Path()
        self.path.header.frame_id = 'world'
        self.publisher_ = self.create_publisher(Path, '/cam_path', 10)
        self.last_time = time.time()
        self.queue = queue.Queue(maxsize=10)
        self.create_timer(0.016, self.readCam)
        self.create_timer(0.016, self.trans)
        
    def trans(self):
        if self.queue.empty():
            return
        frame = self.queue.get()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        results = self.at_detector.detect(gray,True,camera_params,0.050)
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

            r = results[0]
            # print(r)
            T_C_A = np.eye(4)
            T_C_A[:3,:3] = r.pose_R
            T_C_A[:3,3] = r.pose_t.reshape(3)
            T_A_C = np.linalg.inv(T_C_A)

            T_W_C = T_W_A @ T_A_C

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'world'
            t.child_frame_id = 'camera'
            print('x',T_W_C[0,3])
            print('y',T_W_C[1,3])
            print('z',T_W_C[2,3])
            
            q = R.from_matrix(T_W_C[:3,:3]).as_quat()
            q = self.lpf_q.update(q)
            t.transform.translation.x = self.lpf_px.update(T_W_C[0,3])
            t.transform.translation.y = self.lpf_py.update(T_W_C[1,3])
            t.transform.translation.z = self.lpf_pz.update(T_W_C[2,3])
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            rot = R.from_quat(q).as_euler('xyz', degrees=True)
            print(rot)
            
            self.tf_broadcaster.sendTransform(t)

            # if abs(rot[2])<70:
            #     cv2.waitKey(0)
            # if abs(rot[0])<70:
            #     cv2.waitKey(0)
            # if abs(rot[1])>20:
            #     cv2.waitKey(0)  

            self.publish_path(t)
        cv2.imshow('frame', frame)
        
        if cv2.waitKey(1) == ord('b'):
            self.recordpath()

    def readCam(self):
        # print(1/(time.time()-self.last_time))
        self.last_time = time.time()
        ret, frame = self.cap.read()
        if not ret:
            return
        self.queue.put(frame)
        
    def recordpath(self):
        with open('path.txt', 'w') as f:
            for pose in self.path.poses:
                f.write(f'{pose.header.stamp.sec}.{pose.header.stamp.nanosec:09d},{pose.pose.position.x},{pose.pose.position.y},{pose.pose.position.z},{pose.pose.orientation.x},{pose.pose.orientation.y},{pose.pose.orientation.z},{pose.pose.orientation.w}\n')
    
    def publish_path(self,t: TransformStamped):
        pose = PoseStamped()
        pose.header.frame_id = t.child_frame_id
        pose.header.stamp = t.header.stamp
        pose.pose.position.x = t.transform.translation.x
        pose.pose.position.y = t.transform.translation.y
        pose.pose.position.z = t.transform.translation.z
        pose.pose.orientation.x = t.transform.rotation.x
        pose.pose.orientation.y = t.transform.rotation.y
        pose.pose.orientation.z = t.transform.rotation.z
        pose.pose.orientation.w = t.transform.rotation.w
        self.path.poses.append(pose)
        if len(self.path.poses) > 300:
            self.path.poses.pop(0)


        self.publisher_.publish(self.path)
        # self.get_logger().info(f'Publishing path with {len(self.path.poses)} poses')

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
