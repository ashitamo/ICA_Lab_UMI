#!/usr/bin/env python3
"""Detect AprilTag and publish arm target pose to /target_pose_list.

This node watches the same marker ID as `distance_ros2.py` (ID 0). When the
marker is detected it computes the 3D position using depth-based unprojection
(preferred) or solvePnP fallback, converts the camera-frame point into the arm
target pose using the `c2e` conversion from `touch.py`, and publishes a
Float64MultiArray on `/target_pose_list` similar to the behavior in `touch.py`.

"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray as Float64ArrayMsg
from cv_bridge import CvBridge
import numpy as np
import cv2
from dt_apriltags import Detector
import threading
import yaml


TARGET_MARKER_ID = 0
MARKER_LENGTH = 0.103

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


class ArmAprilTagMover(Node):
    def __init__(self):
        super().__init__('arm_apriltag_mover')
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.arm_pose = None
        # flag to publish only once
        self.published_once = False

        qos = QoSProfile(depth=10)

        # publisher to match touch.py usage
        self.target_pub = self.create_publisher(Float64MultiArray, '/target_pose_list', qos)

        # arm pose subscription (expects Float64MultiArray like touch.py)
        self.create_subscription(Float64ArrayMsg, '/arm_pose', self.arm_pose_cb, qos)

        # camera intrinsics (copied from distance_ros2.py)
        self.camera_matrix = np.array([[439.09984732, 0.0, 440.97311686],
                                      [0.0, 428.10459275, 236.80079264],
                                      [0.0, 0.0, 1.0]], dtype=np.float64)
        self.dist_coeffs = np.array([-0.05268922, 0.04680826, -0.00317253, 0.00438352, 0], dtype=np.float64)
        self.fx = self.camera_matrix[0, 0]
        self.fy = self.camera_matrix[1, 1]
        self.cx = self.camera_matrix[0, 2]
        self.cy = self.camera_matrix[1, 2]
        

        # AprilTag detector setup (tag36h11 family)
        self.at_detector = Detector(
            families='tag36h11',
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        # subscribe to the color (RGB) image for detection
        self.create_subscription(Image, '/camera/color/image_rect_raw', self.sync_callback, qos)

        self.get_logger().info('ArmAprilTagMover started: will publish targets to /target_pose_list when marker ID %d is seen' % TARGET_MARKER_ID)

    def arm_pose_cb(self, msg: Float64ArrayMsg):
        with self.lock:
            try:
                self.arm_pose = list(msg.data)
            except Exception:
                self.arm_pose = None

    def sync_callback(self, color_msg):
        # detection and publish flow using RGB image only
        # If we've already published a target once, skip further processing.
        if self.published_once:
            return
        try:
            color_img = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn('CvBridge error: %s' % str(e))
            return

        gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)

        # Detection using AprilTag detector with pose estimation
        # camera_params = [fx, fy, cx, cy]
        tags = self.at_detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=[self.fx, self.fy, self.cx, self.cy],
            tag_size=MARKER_LENGTH
        )

        if len(tags) == 0:
            return

        # find target marker
        target_tag = None
        for tag in tags:
            if tag.tag_id == TARGET_MARKER_ID:
                target_tag = tag
                break

        if target_tag is None:
            return

        # Get 3D position from AprilTag's built-in pose estimation
        if target_tag.pose_t is None:
            self.get_logger().warn('AprilTag pose estimation failed')
            return
        
        apriltag_pos = tuple(target_tag.pose_t.flatten())

        # Print detected AprilTag position
        print(f'Detected AprilTag marker ID {TARGET_MARKER_ID} at position (camera frame): x={apriltag_pos[0]:.4f}, y={apriltag_pos[1]:.4f}, z={apriltag_pos[2]:.4f}')

        # have camera-frame 3D point -> convert to arm frame and publish
        with self.lock:
            arm_pose_local = list(self.arm_pose) if self.arm_pose is not None else None

        if arm_pose_local is None:
            self.get_logger().warn('No arm pose available to convert target')
            return

        try:
            # apriltag_pos is (X,Y,Z) in camera frame
            # target = self.c2e(list(apriltag_pos), arm_pose_local)
            target = self.c2e([-0.038,-0.008,0.215], arm_pose_local)
        except Exception as e:
            self.get_logger().error('c2e conversion failed: %s' % str(e))
            return

        # publish targets similarly to touch.py
        try:
            # msg = Float64MultiArray()
            # # first message: placeholder values similar to touch.py
            # msg.data = [0.0, target[0], target[1], 0.3, target[3], target[4], target[5], target[6], 0.05]
            # self.target_pub.publish(msg)
            # msg2 = Float64MultiArray()
            # msg2.data = [0.0, target[0], target[1], 0.20, target[3], target[4], target[5], target[6], 0.01]
            # self.target_pub.publish(msg2)
            # self.get_logger().info('Published target to /target_pose_list: %s' % (msg2.data,))
            # # mark published so we only send once
            #             # first message: placeholder values similar to touch.py
            msg = Float64MultiArray()
            msg.data = [0.0, target[0], target[1], 0.3, target[3], target[4], target[5], target[6], 0.08]
            self.target_pub.publish(msg)
            msg2 = Float64MultiArray()
            msg2.data = [0.0, target[0], target[1], target[2]-0.05, target[3], target[4], target[5], target[6], 0.01]
            self.target_pub.publish(msg2)
            msg3 = Float64MultiArray()
            msg3.data = [0.0, target[0], target[1], target[2]-0.05, target[3], target[4], target[5], target[6], 0.01]
            self.target_pub.publish(msg3)
            self.get_logger().info('Published target to /target_pose_list: %s' % (msg2.data,))
            # mark published so we only send once
            self.published_once = True
        except Exception as e:
            self.get_logger().error('Failed to publish target: %s' % str(e))

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


def main(args=None):
    rclpy.init(args=args)
    node = ArmAprilTagMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
