import rclpy.time_source
from tm_msgs.srv import SetPositions,SetEvent
from tm_msgs.msg import FeedbackState
from robotiq_85_msgs.msg import GripperCmd, GripperStat
import rclpy
from rclpy.node import Node
import time
from icp_node import *
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R
from line import detect_horizontal_curves_in_roi,compute_top2_curve_distance
from client.client import sender_receiver

with open('/home/lab606/ICA_Lab_UMI/ICA_Lab_UMI_Config.yaml', 'r') as file:
    config_data = yaml.safe_load(file)
    T_G_C = np.array(config_data['T_G_C'])
    T_C_G = np.array(config_data['T_C_G'])
    T_W_a = np.array(config_data['T_W_a'])
    T_a_W = np.linalg.inv(T_W_a)
    T_a_A = np.array(config_data['T_a_A'])
    T_A_a = np.array(config_data['T_A_a'])


import math

def robotiq_2f85_tcp_distance_from_opening(
    opening_m: float,
    *,
    clamp: bool = True
) -> float:
    """
    Convert Robotiq 2F-85 opening width to flange->TCP distance.

    Definition
    ----------
    - opening_m:
        Distance between the two INNER contact faces of the default finger pads.
    - return value:
        Distance from URDF base flange frame (`robotiq_arg2f_base_link`)
        to the midpoint between the two inner contact faces (meters).

    Notes
    -----
    This is derived from the official/standard ROS URDF geometry of the
    Robotiq 2F-85 gripper, not from a linear approximation.

    Valid range
    -----------
    Approximately 0.0015 m to 0.0860 m for the default pads in the URDF.
    """

    # Derived from URDF geometry
    A = 0.0752
    B = 0.0860
    C = 0.0108115106614128

    # d(q) = D*sin(q) + E*cos(q) + F
    D = 0.0376
    E = 0.0430
    F = 0.087324 + -0.004

    # w(q) = A*cos(q) - B*sin(q) + C
    R = math.hypot(A, B)
    phi = math.atan2(B, A)

    w_min = A * math.cos(0.8) - B * math.sin(0.8) + C
    w_max = A * math.cos(0.0) - B * math.sin(0.0) + C

    if clamp:
        opening_m = max(w_min, min(w_max, opening_m))
    else:
        if not (w_min <= opening_m <= w_max):
            raise ValueError(
                f"opening_m out of range: got {opening_m:.6f}, "
                f"expected [{w_min:.6f}, {w_max:.6f}] m"
            )

    # Inverse kinematics for q in [0, 0.8]
    cos_term = (opening_m - C) / R
    cos_term = max(-1.0, min(1.0, cos_term))
    q = math.acos(cos_term) - phi

    # Forward compute TCP distance
    distance_m = D * math.sin(q) + E * math.cos(q) + F
    T_G_E = np.eye(4)
    T_G_E[:3, 3] = np.array([0, 0, distance_m])
    return T_G_E
# for w_mm in [85, 60, 40, 20, 5]:
#     d = robotiq_2f85_tcp_distance_from_opening(w_mm / 1000.0)
#     print(f"opening = {w_mm:>2} mm -> flange_to_tcp = {d*1000:.2f} mm")

class ArmCmd(Node):
    def __init__(self):
        super().__init__('arm_cmd')
        self.cv_bridge = CvBridge()
        self.pos_cli = self.create_client(SetPositions, 'set_positions')
        self.event_cli = self.create_client(SetEvent, 'set_event')
        self.gripper_pub = self.create_publisher(GripperCmd, '/gripper/cmd', 10)
        self.pos_sub = self.create_subscription(FeedbackState, 'feedback_states', self.pos_callback, 10)
        self.cam1_sub = self.create_subscription(Image, '/cam1/color/image_rect_raw', self.cam1_callback, 10)
        self.pc_sub = self.create_subscription(PointCloud2, '/cam1/depth/color/points', self.pc_callback, 10)

        self.umi_sub = self.create_subscription(Image, '/umi_camera/color/image_rect_raw', self.umi_img_callback, 10)
        
        self.grip_sub = self.create_subscription(GripperStat, '/gripper/stat', self.grip_callback, 10)

        self.cam_info_queue = queue.Queue(1)
        self.cam1_info_sub = self.create_subscription(
            CameraInfo,
            '/cam1/color/camera_info',
            self.cam1_info_callback,
            10
        )

        while not self.pos_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        while not self.event_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.set_positions_req = SetPositions.Request()
        self.set_event_req = SetEvent.Request()
        self.target_positions = [0.5, -0.1, 0.44, 3.14, 0.0, -3.14]
        self.current_positions = self.target_positions.copy()
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
    def cam1_info_callback(self, msg):
        if self.cam_info_queue.full():
            self.cam_info_queue.get()
        self.cam_info_queue.put(msg)

    def pc_callback(self,msg):
        self.current_pc = pc2_to_open3d(msg)
    def grip_callback(self,msg):
        self.current_grip = msg.position
        self.current_grip_is_moving = msg.is_moving
        # self.get_logger().info("Current Gripper: %s" % self.current_grip)
    def cam1_callback(self,msg):
        self.current_cam1_img = self.cv_bridge.imgmsg_to_cv2(msg)

    def umi_img_callback(self,msg):
        self.current_umi_img = self.cv_bridge.imgmsg_to_cv2(msg)
    
    def pos_callback(self,msg):
        self.current_positions = msg.tool_pose
        # self.get_logger().info("Current Position: %s" % self.current_positions)

    def is_arrived(self,error=0.005):
        if sum((self.target_positions[i]-self.current_positions[i])**2 for i in range(3)) > error**2:
            return False
        return True

    def set_positions(self,positions=[0.2, -0.4, 0.35, 3.14159, 0.0, -3.14],
                     velocity=0.2, acc_time=0.25, blend_percentage=100, fine_goal=False):
        
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
    
    def set_positions_block(self,positions=[0.2, -0.4, 0.35, 3.14159, 0.0, -3.14],
                     velocity=0.2, acc_time=0.25, blend_percentage=100, fine_goal=False):
        response = self.set_positions(positions,velocity,acc_time,blend_percentage,fine_goal)
        while not self.is_arrived():
            rclpy.spin_once(self)
        print("move",self.target_positions)

    def set_grip_positions_block(self,positions=[0.2, -0.4, 0.35, 3.14159, 0.0, -3.14],gap=0.085,
                     velocity=0.2, acc_time=0.25, blend_percentage=100, fine_goal=False):
        T_G_E = robotiq_2f85_tcp_distance_from_opening(gap)
        T_E_G = np.linalg.inv(T_G_E)
        T_W_E = np.eye(4)
        T_W_E[:3,:3] = R.from_euler('xyz', positions[3:], degrees=False).as_matrix()
        T_W_E[:3,3] = np.array(positions[:3])
        T_W_G = T_W_E @ T_E_G
        positions = list(T_W_G[:3,3]) + list(R.from_matrix(T_W_G[:3,:3]).as_euler('xyz', degrees=False))
        response = self.set_positions(positions,velocity,acc_time,blend_percentage,fine_goal)
        while not self.is_arrived():
            rclpy.spin_once(self)
        self.send_gripper_block(gap)
        print("move",self.target_positions)
    def show_grip_positions(self,gap=0.085):
        for _ in range(10):
            rclpy.spin_once(self)
        T_W_G = np.eye(4)
        T_W_G[:3,:3] = R.from_euler('xyz', self.current_positions[3:], degrees=False).as_matrix()
        T_W_G[:3,3] = np.array(self.current_positions[:3])

        T_G_E = robotiq_2f85_tcp_distance_from_opening(gap)
        T_E_G = np.linalg.inv(T_G_E)

        T_W_E = T_W_G @ T_G_E
        x,y,z = T_W_E[:3,3]
        rx,ry,rz= R.from_matrix(T_W_E[:3,:3]).as_euler('xyz', degrees=False)
        print([x,y,z,rx,ry,rz])

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
        grip_msg.force = 0.5
        self.gripper_pub.publish(grip_msg)
    def send_gripper_block(self,gap=0.085):
        gap = gap if gap < 0.085 else 0.085
        gap = gap if gap > 0.0 else 0.0
        print(gap)
        grip_msg = GripperCmd()
        grip_msg.emergency_release = False
        grip_msg.emergency_release_dir = 0
        grip_msg.stop = False
        grip_msg.position = gap
        grip_msg.speed = 0.1
        grip_msg.force = 0.5
        self.gripper_pub.publish(grip_msg)
        count = 1
        while not self.is_gripper_arrived(gap):
            if count >= 3 and not self.current_grip_is_moving:
                break
            rclpy.spin_once(self)
            count+=1

    def is_gripper_arrived(self,target=0.085):
        return abs(self.current_grip - target) < 0.004

    def send_event(self):
        rclpy.spin_once(self)
        self.set_event_req = SetEvent.Request()
        self.set_event_req.func = SetEvent.Request.STOP
        self.set_event_req.arg0 = 0
        self.set_event_req.arg1 = 0
        future = self.event_cli.call_async(self.set_event_req)
        # rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def wait(self,t=10):
        for _ in range(t):
            rclpy.spin_once(self)


def main(args=None):
    rclpy.init(args=args)
    armCmd = ArmCmd()
    rclpy.spin_once(armCmd)
    armCmd.send_gripper(0.085)
    time.sleep(1.0)
    armCmd.set_positions_block([0.511, -0.0, 0.340, 3.14159, 0.0, -3.14])
    armCmd.send_gripper(0.085)
    time.sleep(1.0)
    armCmd.set_positions_block([0.511, -0.0, 0.340, 3.14159, 0.0, -3.14])


    armCmd.set_positions_block([0.511, -0.25, 0.55, 3.14159, 0.4, -1.57])
    armCmd.wait()
    img=armCmd.current_cam1_img
    cv2.imshow("img",img)
    cv2.waitKey(100)
    cv2.imwrite(r"/home/lab606/ICA_Lab_UMI/ros2_ws/src/ipc_test/ipc_test/client/temp.jpg",img)
    keypoints = sender_receiver(r"/home/lab606/ICA_Lab_UMI/ros2_ws/src/ipc_test/ipc_test/client/temp.jpg")
    #offset keypoints
    keypoints['top_hem_left'][0] = keypoints['top_hem_left'][0] + 40
    keypoints['top_hem_left'][1] = keypoints['top_hem_left'][1] - 10
    # generate a square mask region by keypoints
    hem_left_mask = np.zeros((img.shape[0], img.shape[1]), dtype=np.uint8)
    hem_left_point = keypoints['top_hem_left']
    hem_left_mask[hem_left_point[1]-10:hem_left_point[1]+10, hem_left_point[0]-10:hem_left_point[0]+10] = 255

    # visualize the square mask region from rgb image
    rgb_region = cv2.bitwise_and(img, img, mask=hem_left_mask)
    cv2.imshow("rgb_region", rgb_region)
    cv2.waitKey(100)

    intr = armCmd.get_cam_info()
    all_pc = armCmd.current_pc
    pc = masked_pointcloud_from_o3d(all_pc, hem_left_mask, intr)
    #visualize the pointcloud with bbox
    bbox = pc.get_axis_aligned_bounding_box()
    bbox.color = (0, 0, 255)
    o3d.visualization.draw_geometries([all_pc, bbox])
    # avg pointcloud coordinates
    hem_left_pos = pc.get_center()
    print("hem_left_pos cam1 frame",hem_left_pos)
    # transfrom to world frame
    T_W_G = np.eye(4)
    T_W_G[:3,:3] = R.from_euler('xyz', armCmd.current_positions[3:], degrees=False).as_matrix()
    T_W_G[:3,3] = np.array(armCmd.current_positions[:3])
    T_W_C = T_W_G @ T_G_C
    pc.transform(T_W_C)
    # hem_left_pos = pc.get_center()
    points = np.asarray(pc.points)
    # find topest z point
    hem_left_pos = points[np.argmax(points[:, 2])]
    print("hem_left_pos world frame",hem_left_pos)

    initx = hem_left_pos[0]
    inity = hem_left_pos[1]
    initz = hem_left_pos[2] + 0.0185
    for z in range(20):
        zmm = z * 0.00075
        armCmd.send_gripper(0.085)
        time.sleep(1.0)
        print(initz-zmm,z)
        armCmd.set_grip_positions_block([initx, inity, initz, 3.14159, 0.0, -3.14],0.085)
        armCmd.set_grip_positions_block([initx, inity, initz-zmm, 3.14159, 0.0, -3.14],0.000,velocity=0.2)
        armCmd.set_grip_positions_block([initx, inity, 0.05, 3.14159, 0.0, -3.14],0.000,velocity=0.2)
        rclpy.spin_once(armCmd)
        result, curves = detect_horizontal_curves_in_roi(
            img=armCmd.current_umi_img,
            roi=(340, 370, 230, 150
                 ),
            canny1=60,
            canny2=150,
            min_points=75,
            y_gap=3,
            match_tol=6,
            max_gap_x=15,          # 允許短暫遮擋
            merge_tracks=True,
            max_join_gap=60,       # 左右兩段相隔多少仍可合併
            max_join_y_diff=15,    # 遮擋前後高度差容忍
            smooth_window=7,
            fit_poly=True,         # 用擬合補中間缺口
            poly_deg=2,
            debug=True
        )
        distance_stat = compute_top2_curve_distance(curves)
        if distance_stat:
            print(
                f"Distance: max={distance_stat['max']:.2f}, "
                f"median={distance_stat['median']:.2f}, "
                f"mean={distance_stat['mean']:.2f}, "
                f"num_samples={distance_stat['num_samples']}"
            )
            if distance_stat["mean"] > 20.0:
                break

    
    # armCmd.set_grip_positions_block([0.5109968641797566, -0.1809885315022186, 0.012674790636978961, 3.141555635650641, -6.897627047175448e-05, -3.1399811748141864],0.03)
    # armCmd.show_grip_positions(0.03)
    
    # armCmd.send_gripper(0.085)

    armCmd.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()