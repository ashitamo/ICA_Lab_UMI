#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np
import cv2
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import threading


class ClickToPoint(Node):
    def __init__(self):
        super().__init__('click_to_point')
        self.bridge = CvBridge()
        self.color = None
        self.depth = None
        self.depth_encoding = None
        self.K = True  # intrinsics: [fx 0 cx; 0 fy cy; 0 0 1]
        self.lock = threading.Lock()

        qos = QoSProfile(depth=4)
        self.create_subscription(Image, '/camera/color/image_raw', self.color_cb, qos)
        self.create_subscription(Image, '/camera/depth/image_rect_raw', self.depth_cb, qos)
        # self.create_subscription(CameraInfo, '/camera/color/camera_info', self.info_cb, qos)
        self.create_subscription(Float64MultiArray, '/arm_pose', self.arm_pose_cb, qos)

        # publisher for clicked targets (x,y,z) as a flat array
        self.target_pub = self.create_publisher(Float64MultiArray, '/target_pose_list', qos)

        cv2.namedWindow('color', cv2.WINDOW_NORMAL)
        cv2.setMouseCallback('color', self.on_mouse)
        self.get_logger().info('Click on the image window to get X,Y,Z in camera frame (meters)')

    def color_cb(self, msg: Image):
        with self.lock:
            try:
                self.color = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            except Exception:
                self.color = None

    def arm_pose_cb(self, msg):
        with self.lock:
            try:
                self.arm_pose = msg.data
            except Exception:
                self.arm_pose = None

    def depth_cb(self, msg: Image):
        with self.lock:
            try:
                self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                # encoding field name is the same in ROS2
                self.depth_encoding = msg.encoding if hasattr(msg, 'encoding') else None
            except Exception:
                self.depth = None
                self.depth_encoding = None

    # def info_cb(self, msg: CameraInfo):
    #     # camera_info.k is row-major 3x3
    #     with self.lock:
    #         K = np.array(msg.k).reshape(3, 3)
    #         self.K = K

    def on_mouse(self, event, u, v, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        with self.lock:
            # if self.color is None or self.depth is None or self.K is None:
            #     print('No data yet (color/depth/camera_info).')
            #     return
            h, w = self.depth.shape[:2]
            if not (0 <= v < h and 0 <= u < w):
                print('Click out of depth image bounds')
                return

            z = None
            if self.depth_encoding in ('16UC1', 'mono16'):
                # depth in millimeters
                z_raw = float(self.depth[v, u])
                if z_raw == 0:
                    print('Depth value is 0 (invalid)')
                    return
                z = z_raw / 1000.0
            else:
                # assume float meters (32FC1)
                z_raw = float(self.depth[v, u])
                if np.isnan(z_raw) or z_raw <= 0.0:
                    print('Invalid depth (NaN or <= 0)')
                    return
                z = z_raw

            fx = 439.09984732
            fy = 440.97311686
            cx = 428.10459275
            cy = 236.80079264

            x = (u - cx) * z / fx
            y = (v - cy) * z / fy
            print(f'Clicked pixel (u={u}, v={v}) -> 3D (camera frame) X={x:.4f} Y={y:.4f} Z={z:.4f} [m]')
            print(f'Arm pose (base frame) : {self.arm_pose}')
            temp = self.c2e([x,y,z],self.arm_pose)
            # publish clicked 3D point as Float64MultiArray [x, y, z]
            try:
                msg = Float64MultiArray()
                msg.data = [0.0, temp[0], temp[1], 0.4, temp[3], temp[4], temp[5], temp[6], 0.01]
                self.target_pub.publish(msg)
                msg.data = [0.0, temp[0], temp[1], temp[2], temp[3], temp[4], temp[5], temp[6], 0.01]
                self.target_pub.publish(msg)
                print(f'Published target to /target_pose_list: {msg.data}')
            except Exception as e:
                self.get_logger().error(f'Failed to publish target: {e}')

    def c2e(self, p_C0_o0, T_W_G):
        '''
        p_C0_o0 : np.array([x,y,z])
        T_a_C0 : np.array([x,y,z,rx,ry,rz,rw])
        =>
        T_a_C1 np.array([x,y,z,rx,ry,rz,rw])
        '''
        import numpy as np
        from scipy.spatial.transform import Rotation as R
        temp = T_W_G
        T_W_G = np.eye(4)
        T_W_G[:3,:3] = R.from_euler('xyz',temp[3:], degrees=False).as_matrix()
        T_W_G[:3,3] = temp[:3]

        T_W_a = np.array(
        [[0.0010496426298024, -0.9998620271063147, -0.016033199637844893,-0.3159137375616857],
            [0.999989949780332,0.0010719732591989246, -0.0018331928875377549,-0.21343326295610782],
            [ 0.0018531577824868462,-0.016033198136207613, 0.9998548573736284, -0.0032541557022790885],
            [ 0.        , 0.        , 0.        , 1.        ]]
        )
        T_a_W = np.linalg.inv(T_W_a)

        T_G_C = np.array(
            [[ 0.9999290185258485, -0.0011008873649813458,  0.011863640121940749,-0.008890647492573154],
            [ -0.0011860146991396002,  0.9815797405642499,  0.1910492247640736, -0.06862693073147604],
            [-0.011855432470674476, -0.1910497342600343,  0.9815087609186703,   0.035262395085736725],
            [ 0,          0,          0,          1,        ]]
        )
        T_G_E = np.array(
            [[ 1, 0, 0, 0],
            [ 0, 1, 0, 0],
            [ 0, 0, 1, 0.164],
            [ 0, 0, 0, 1]]
        )
        T_E_G = np.linalg.inv(T_W_G)
        T_C_E = np.linalg.inv(T_G_C) @ T_G_E
        T_E_C = np.linalg.inv(T_C_E) 

        T_a_C0 = T_a_W @ T_W_G @ T_G_C
        T_C0_o0 = np.array(
            [[ 1, 0, 0, p_C0_o0[0]],
            [ 0, 1, 0, p_C0_o0[1]],
            [ 0, 0, 1, p_C0_o0[2]],
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

    def spin(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01)
                with self.lock:
                    if self.color is not None:
                        cv2.imshow('color', self.color)
                key = cv2.waitKey(1) & 0xFF
                if key == 27:
                    break
        finally:
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = ClickToPoint()
    try:
        node.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()