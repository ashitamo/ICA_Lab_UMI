from sensor_msgs.msg import Image,PointCloud2,CameraInfo
from sensor_msgs_py import point_cloud2
import rclpy
from rclpy.node import Node
import time
import numpy as np
import queue
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
from scipy.spatial.transform import Rotation as R
import math
import threading
import open3d as o3d
from util import *
from icp import *
import cv2
class ICPNode(Node):
    def __init__(self):
        super().__init__('icp_node')
        self.cv_bridge = CvBridge()
        self.cam_queue = queue.Queue(2)
        self.cam_info_queue = queue.Queue(1)
        self.infra_info_queue = queue.Queue(1)
        self.cam_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.cam_info_callback,
            10
        )
        self.infra_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/infra1/camera_info',
            self.infra_info_callback,
            10
        )

        self.color_img_sub = Subscriber(self, Image, '/camera/color/image_rect_raw')
        self.infra_img_sub = Subscriber(self, Image, '/camera/infra1/image_rect_raw')
        
        self.ts = ApproximateTimeSynchronizer(
            [self.color_img_sub, self.infra_img_sub],
            queue_size=10,
            slop=0.051  # 1/60+1/30 + 0.001 (30 hz, 60 hz)
        )
        self.ts.registerCallback(self.sync_cam_callback)
        self.get_logger().info('ICP Node has been started.')
        self.is_logged = True

    def sync_cam_callback(self, color_img, infra_img):
        if self.is_logged:
            self.get_logger().info('color_img timestamp: %.6f' % color_img.header.stamp.sec + '%.6f' % (color_img.header.stamp.nanosec*1e-9))
            self.get_logger().info('infra_img timestamp: %.6f' % infra_img.header.stamp.sec + '%.6f' % (infra_img.header.stamp.nanosec*1e-9))
            
        if self.cam_queue.full():
            self.cam_queue.get()
        color_img_cv = self.cv_bridge.imgmsg_to_cv2(color_img)
        depth_img_cv = self.cv_bridge.imgmsg_to_cv2(infra_img)
        self.cam_queue.put((color_img_cv, depth_img_cv))
        
    def get_cam_data(self):
        if self.cam_queue.empty():
            return None
        return self.cam_queue.get()

    def cam_info_callback(self, msg):
        if self.is_logged:
            self.get_logger().info('cam_info timestamp: %.6f' % msg.header.stamp.sec + '%.6f' % (msg.header.stamp.nanosec*1e-9))

        if self.cam_info_queue.full():
            self.cam_info_queue.get()
        self.cam_info_queue.put(msg)

    def infra_info_callback(self, msg):
        if self.is_logged:
            self.get_logger().info('cam_info timestamp: %.6f' % msg.header.stamp.sec + '%.6f' % (msg.header.stamp.nanosec*1e-9))

        if self.infra_info_queue.full():
            self.infra_info_queue.get()
        self.infra_info_queue.put(msg)
        
    def get_cam_info(self):
        if self.cam_info_queue.empty():
            return None
        info = {}
        msg = self.cam_info_queue.get()
        info['width'] = msg.width
        info['height'] = msg.height
        info['fx'] = msg.k[0]
        info['fy'] = msg.k[4]
        info['ppx'] = msg.k[2]
        info['ppy'] = msg.k[5]
        info['distortion_model'] = msg.distortion_model
        info['coeffs'] = msg.d
        return info
    
    def get_infra_info(self):
        if self.infra_info_queue.empty():
            return None
        info = {}
        msg = self.infra_info_queue.get()
        info['width'] = msg.width
        info['height'] = msg.height
        info['fx'] = msg.k[0]
        info['fy'] = msg.k[4]
        info['ppx'] = msg.k[2]
        info['ppy'] = msg.k[5]
        info['distortion_model'] = msg.distortion_model
        info['coeffs'] = msg.d
        return info

    def is_ready(self):
        return not self.cam_queue.empty() and not self.cam_info_queue.empty()

def startNode():
    iCPNode = ICPNode()
    executor = MultiThreadedExecutor()
    executor.add_node(iCPNode)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    return iCPNode
def to_gray(img):
    if img is None:
        raise ValueError("img is None")
    if img.ndim == 2:
        return img
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

def find_chessboard_corners(gray, pattern_size, refine=True):
    """
    pattern_size: (cols, rows) 例如 (9,6) 表示內角點 9x6
    回傳 corners: (N,1,2) float32
    """
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv2.findChessboardCorners(gray, pattern_size, flags)
    if not ok:
        return None

    if refine:
        # cornerSubPix 需要 float32
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-4)
        corners = cv2.cornerSubPix(
            gray,
            corners,
            winSize=(11, 11),
            zeroZone=(-1, -1),
            criteria=criteria
        )
    return corners

def estimate_color_to_infra_homography(color_img, infra_img, pattern_size=(11, 8), debug_vis=False):
    """
    回傳 H (3x3): infra ~= H * color
    """
    g_color = to_gray(color_img)
    g_infra = to_gray(infra_img)

    c_corners = find_chessboard_corners(g_color, pattern_size, refine=True)
    i_corners = find_chessboard_corners(g_infra, pattern_size, refine=True)

    if c_corners is None:
        raise RuntimeError("Chessboard not found in color image")
    if i_corners is None:
        raise RuntimeError("Chessboard not found in infra image")

    # (N,1,2) -> (N,2)
    pts_color = c_corners.reshape(-1, 2)
    pts_infra = i_corners.reshape(-1, 2)

    # 用 RANSAC 更抗雜訊/錯角點
    H, inliers = cv2.findHomography(pts_color, pts_infra, method=cv2.RANSAC, ransacReprojThreshold=2.0)
    if H is None:
        raise RuntimeError("findHomography failed")

    if debug_vis:
        # 畫角點 + inlier
        vis_c = color_img.copy()
        vis_i = infra_img.copy()
        cv2.drawChessboardCorners(vis_c, pattern_size, c_corners, True)
        cv2.drawChessboardCorners(vis_i, pattern_size, i_corners, True)

        inl = inliers.ravel().astype(bool)
        # 把 color 角點投影到 infra，看誤差
        proj = cv2.perspectiveTransform(pts_color.reshape(-1,1,2).astype(np.float32), H).reshape(-1,2)

        # 畫投影點（綠）與真實 infra 角點（紅）
        for p_est, p_gt, ok in zip(proj, pts_infra, inl):
            if not ok:
                continue
            cv2.circle(vis_i, (int(p_gt[0]), int(p_gt[1])), 4, (0,0,255), -1)   # GT red
            cv2.circle(vis_i, (int(p_est[0]), int(p_est[1])), 3, (0,255,0), -1) # EST green

        cv2.imshow("color corners", vis_c)
        cv2.imshow("infra corners + projected", vis_i)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return H
def main():
    rclpy.init()
    iCPNode = startNode()
    while True:
        if iCPNode.is_ready():
            break
        time.sleep(0.1)
    iCPNode.is_logged = False
    data = iCPNode.get_cam_data()
    intr = iCPNode.get_cam_info()

    color_img, infra_img = data
    cv2.imshow("color", color_img)
    cv2.imshow("infra", infra_img)
    cv2.waitKey(0)
    # 1) find chessboard + 2) find homography
    # 設定你的棋盤格「內角點」數量（請改成你的）
    pattern_size = (10, 7)   # 例如 9x6 內角點

    H_color_to_infra = estimate_color_to_infra_homography(
        color_img, infra_img,
        pattern_size=pattern_size,
        debug_vis=True
    )

    print("H (color -> infra)=\n", np.array2string(H_color_to_infra, separator=', '))

    # 如果你要把 color warp 到 infra 尺寸看效果：
    h_i, w_i = infra_img.shape[:2]
    color_warp = cv2.warpPerspective(color_img, H_color_to_infra, (w_i, h_i))
    cv2.imshow("color warped to infra", color_warp)
    cv2.imshow("infra", infra_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    


if __name__ == '__main__':
    main()
