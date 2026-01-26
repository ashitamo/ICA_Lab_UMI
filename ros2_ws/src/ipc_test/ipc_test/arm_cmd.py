import rclpy.time_source
from tm_msgs.srv import SetPositions,SetEvent
from tm_msgs.msg import FeedbackState
from robotiq_85_msgs.msg import GripperCmd
import rclpy
from rclpy.node import Node
import time
from icp_node import *
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

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

class ArmCmd(Node):
    def __init__(self):
        super().__init__('arm_cmd')
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
        self.target_positions = [0.5, -0.1, 0.44, 3.14, 0.0, -1.57]
        self.current_positions = self.target_positions.copy()
        
    def pos_callback(self,msg):
        self.current_positions = msg.tool_pose
        # self.get_logger().info("Current Position: %s" % self.current_positions)

    def is_arrived(self,error=0.005):
        if sum((self.target_positions[i]-self.current_positions[i])**2 for i in range(3)) > error**2:
            return False
        return True

    def set_positions(self,positions=[0.2, -0.4, 0.35, 3.14159, 0.0, -1.57],
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
    
    def set_positions_block(self,positions=[0.2, -0.4, 0.35, 3.14159, 0.0, -1.57],
                     velocity=0.2, acc_time=0.25, blend_percentage=100, fine_goal=False):
        response = self.set_positions([0.5, -0.1, 0.44, 3.14, 0.0, -1.57])
        while not self.is_arrived():
            rclpy.spin_once(self)
        print("move",self.target_positions)

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


def make_frame(T: np.ndarray, size: float = 0.05) -> o3d.geometry.TriangleMesh:
    """Create an Open3D coordinate frame mesh transformed by T (4x4)."""
    fr = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size, origin=[0, 0, 0])
    fr.transform(T)
    return fr

def make_sphere_at(p: np.ndarray, radius: float = 0.01) -> o3d.geometry.TriangleMesh:
    """Create a small sphere at point p (3,)."""
    s = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
    s.translate(p.reshape(3))
    return s

def make_frame(T: np.ndarray, size: float = 0.05) -> o3d.geometry.TriangleMesh:
    fr = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size, origin=[0,0,0])
    fr.transform(T)
    return fr

def make_sphere_at(p: np.ndarray, radius: float = 0.008) -> o3d.geometry.TriangleMesh:
    s = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
    s.translate(np.asarray(p).reshape(3))
    return s

def visualize_chain_in_camera_origin(
    p_C0_o0: np.ndarray,      # 點雲座標下的點（相機座標）
    T_W_G0: np.ndarray,
    T_G_C: np.ndarray,        # G -> C
    T_E_G: np.ndarray,        # E -> G（或你用的那個）
    extra_geoms=None,
    frame_size: float = 0.06,
):
    # ---- Build W->* transforms (你的原本鏈) ----
    T_C0_o0 = np.eye(4)
    T_C0_o0[:3, 3] = np.asarray(p_C0_o0).reshape(3)

    T_W_C0 = T_W_G0 @ T_G_C
    T_W_o0 = T_W_C0 @ T_C0_o0

    T_W_E1 = T_W_o0.copy()
    rot = R.from_matrix(T_W_E1[:3,:3]).as_euler("xyz", degrees=True)
    rot[0], rot[1] = 180.0, 0.0
    T_W_E1[:3,:3] = R.from_euler("xyz", rot, degrees=True).as_matrix()

    T_W_G1 = T_W_E1 @ T_E_G

    # ---- Convert everything to camera-origin: C0->* ----
    T_C0_W = np.linalg.inv(T_W_C0)  # (W->C0)^-1 = C0->W

    def to_C(T_W_X):
        return T_C0_W @ T_W_X

    T_C0_G0 = to_C(T_W_G0)
    T_C0_C0 = np.eye(4)
    T_C0_o0 = to_C(T_W_o0)
    T_C0_E1 = to_C(T_W_E1)
    T_C0_G1 = to_C(T_W_G1)

    # ---- Draw: now origin is C0 ----
    geoms = [] if extra_geoms is None else list(extra_geoms)

    # 讓點雲也在 C0 座標下（你說點雲原點就是 C0，所以直接加就好）
    # 若 extra_geoms 裡已經是相機座標，就不用再 transform。

    # 畫 C0 世界座標軸（在原點）
    geoms.append(o3d.geometry.TriangleMesh.create_coordinate_frame(size=frame_size * 1.2))

    frames = {
        "G0": T_C0_G0,
        "o0": T_C0_o0,
        # "E1": T_C0_E1,
        # "G1": T_C0_G1,
        "W": T_C0_W
    }

    for name, T in frames.items():
        geoms.append(make_frame(T, size=frame_size))
        geoms.append(make_sphere_at(T[:3, 3], radius=frame_size * 0.12))

    o3d.visualization.draw_geometries(geoms)

    return T_W_G1  # 你若想要最後結果也用相機座標回傳

def _frame(T, size=0.05):
    f = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size, origin=[0,0,0])
    f.transform(T)
    return f

def _sphere(p, r=0.008):
    s = o3d.geometry.TriangleMesh.create_sphere(radius=r)
    s.translate(np.asarray(p).reshape(3))
    return s

def _line(p0, p1):
    ls = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector([p0.tolist(), p1.tolist()]),
        lines=o3d.utility.Vector2iVector([[0, 1]])
    )
    ls.colors = o3d.utility.Vector3dVector([[0, 0, 0]])
    return ls

def visualize_process_in_camera_origin(
    T_W_G0: np.ndarray,
    T_G_C: np.ndarray,
    T_C_P: np.ndarray,
    T_P_C: np.ndarray,
    T_C_G: np.ndarray,
    selected_hole_c: np.ndarray,   # (3,) in camera frame
    point_cloud_c=None,            # Open3D pcd already in camera frame
    frame_size=0.06,
):
    # --- compute W->* (跟你原本一樣) ---
    hole_c_h = np.array([*selected_hole_c, 1.0], dtype=np.float64)
    hole_W_h = T_W_G0 @ T_G_C @ hole_c_h
    hole_W = hole_W_h[:3] / (hole_W_h[3] if abs(hole_W_h[3]) > 1e-12 else 1.0)

    T_W_C0 = T_W_G0 @ T_G_C
    T_W_P0 = T_W_C0 @ T_C_P

    T_W_P1 = T_W_P0.copy()
    T_W_P1[:3, 3] = hole_W

    T_W_G1 = T_W_P1 @ T_P_C @ T_C_G

    # --- convert to camera-origin transforms: C0->* ---
    T_C0_W = np.linalg.inv(T_W_C0)

    def to_C(T_W_X):
        return T_C0_W @ T_W_X

    T_C0_G0 = to_C(T_W_G0)
    T_C0_P0 = to_C(T_W_P0)
    T_C0_P1 = to_C(T_W_P1)
    T_C0_G1 = to_C(T_W_G1)

    # hole position in camera frame（用兩種方式核對）
    # 方式A：直接就是你輸入的 selected_hole_c（如果它真的是 C 座標）
    hole_C_A = np.asarray(selected_hole_c, dtype=np.float64)

    # 方式B：由 hole_W 轉回 C（用 W->C0 反推）
    hole_C_B_h = T_C0_W @ np.array([*hole_W, 1.0], dtype=np.float64)
    hole_C_B = hole_C_B_h[:3] / (hole_C_B_h[3] if abs(hole_C_B_h[3]) > 1e-12 else 1.0)

    print("[Debug] hole_C_A (input):", hole_C_A)
    print("[Debug] hole_C_B (from W):", hole_C_B)
    print("[Debug] diff (A-B):", np.linalg.norm(hole_C_A - hole_C_B))

    # --- draw in camera coordinates ---
    geoms = []
    if point_cloud_c is not None:
        geoms.append(point_cloud_c)

    # Camera/world frame at origin
    geoms.append(o3d.geometry.TriangleMesh.create_coordinate_frame(size=frame_size * 1.2))

    # frames (in C0)
    geoms += [
        _frame(T_C0_G0, size=frame_size),
        _frame(T_C0_P0, size=frame_size),
        _frame(T_C0_P1, size=frame_size),
        _frame(T_C0_G1, size=frame_size),
    ]

    # origins
    geoms += [
        _sphere(T_C0_G0[:3,3], r=frame_size*0.12),
        _sphere(T_C0_P0[:3,3], r=frame_size*0.12),
        _sphere(T_C0_P1[:3,3], r=frame_size*0.14),
        _sphere(T_C0_G1[:3,3], r=frame_size*0.12),
    ]

    # hole in camera frame
    geoms.append(_sphere(hole_C_A, r=frame_size*0.18))

    # chain lines
    geoms += [
        _line(np.zeros(3), T_C0_G0[:3,3]),
        _line(np.zeros(3), T_C0_P0[:3,3]),
        _line(T_C0_P0[:3,3], hole_C_A),
        _line(hole_C_A, T_C0_G1[:3,3]),
    ]

    o3d.visualization.draw_geometries(geoms)

    return {
        "T_W_C0": T_W_C0,
        "T_W_P0": T_W_P0,
        "T_W_P1": T_W_P1,
        "T_W_G1": T_W_G1,
        "T_C0_G0": T_C0_G0,
        "T_C0_P0": T_C0_P0,
        "T_C0_P1": T_C0_P1,
        "T_C0_G1": T_C0_G1,
        "hole_C": hole_C_A,
        "hole_W": hole_W,
    }

def rotCam(T_W_G,rot = [180,0,-90]):
    '''
    T_W_G : np.array([x,y,z,rx,ry,rz])
    desired camera orientation
    =>
    T_W_G : np.array([x,y,z,rx,ry,rz]) rotate to horizon
    '''
    temp = np.eye(4)
    temp[:3,:3] = R.from_euler('xyz',T_W_G[3:], degrees=False).as_matrix()
    temp[:3,3] = np.array(T_W_G[:3])
    T_W_G = temp.copy()
    T_W_C = T_W_G @ T_G_C
    T_W_C[:3,:3] = R.from_euler('xyz', rot, degrees=True).as_matrix()
    T_W_G = T_W_C @ T_C_G
    x,y,z = T_W_G[:3,3]
    rx,ry,rz= R.from_matrix(T_W_G[:3,:3]).as_euler('xyz', degrees=False)
    temp = np.array([x,y,z,rx,ry,rz],dtype=np.float64)
    return temp

def rotEnd(T_W_G,rot = [180,0,-90]):
    '''
    T_W_G : np.array([x,y,z,rx,ry,rz])
    desired camera orientation
    =>
    T_W_G : np.array([x,y,z,rx,ry,rz]) rotate to horizon
    '''
    temp = np.eye(4)
    temp[:3,:3] = R.from_euler('xyz',T_W_G[3:], degrees=False).as_matrix()
    temp[:3,3] = np.array(T_W_G[:3])
    T_W_G = temp.copy()
    T_W_E = T_W_G @ T_G_E
    T_W_E[:3,:3] = R.from_euler('xyz', rot, degrees=True).as_matrix()
    T_W_G = T_W_E @ T_E_G
    x,y,z = T_W_G[:3,3]
    rx,ry,rz= R.from_matrix(T_W_G[:3,:3]).as_euler('xyz', degrees=False)
    temp = np.array([x,y,z,rx,ry,rz],dtype=np.float64)
    return temp

def main(args=None):
    bias = 0.0025
    # bias = 0.0
    rclpy.init(args=args)
    iCPNode = startNode()
    armCmd = ArmCmd()
    while not iCPNode.is_ready():
        print("[Debug] iCPNode is not ready")
        time.sleep(1.0)
    print("[Debug] iCPNode is ready")
    iCPNode.is_logged = False
    rclpy.spin_once(armCmd)
    
    # init position
    armCmd.send_gripper(0.085)
    time.sleep(0.5)
    armCmd.send_gripper(0.085)
    time.sleep(0.5)
    armCmd.set_positions_block([0.5, -0.1, 0.44, 3.14, 0.0, -1.57])

    # approach 
    #   move to top position
    armCmd.set_positions_block([0.55, -0.3, 0.44, 3.14, 0.0, -1.57])

    # grasp
    #   ICP
    #   move to pick position
    last_pos = [0.6, -0.27, 0.35, 3.14, 0.0, -1.07]
    armCmd.set_positions_block(last_pos)
    time.sleep(2.0)
    data = iCPNode.get_data()
    intr = iCPNode.get_cam_info()
    color_img, depth_img, point_cloud, tcp_pose = data
    point_cloud = pc2_to_open3d(point_cloud)
    # point_cloud = create_point_cloud(depth_img, color_img, intr['width'], intr['height'], intr['fx'], intr['fy'], intr['ppx'], intr['ppy'])
    point_cloud = filter_pcd_by_depth_percentile(point_cloud,keep_percentile=95)
    o3d.visualization.draw_geometries_with_editing(
        [point_cloud],
        window_name="Shift + Left Click to pick points"
    )
    
    field_img_mask_center, field_img_colored_mask, field_masks = generate_masks(color_img)
    mask,_ = get_cylinder_mask(
            [],
            field_img_mask_center,
            field_masks
        )
    src = masked_pointcloud_from_o3d(point_cloud,mask,intr)
    src_np = np.asarray(src.points)
    tgt = mesh_to_pcd("/home/lab606/ICA_Lab_UMI/ros2_ws/src/ipc_test/peg_30.5mm - Part 1.stl")
    T, bbox, result_icp = get_bbox_by_icp(src, tgt, viz=True, all_pcd=point_cloud)
    #   cal grasp position
    top_pos, top_R, bottom_pos, bottom_R = top_and_bottom_face_centers(bbox)
    print(np.asarray(top_pos.points),np.asarray(bottom_pos.points))
    def c2e(p_C0_o0,T_W_G0):
        '''
        p_C0_o0 : np.array([x,y,z])
        T_W_G0 : transformation mat
        =>
        T_W_G1 : transformation mat
        '''

        T_C0_o0 = np.eye(4)
        T_C0_o0[:3, 3] = np.asarray(p_C0_o0).reshape(3)

        T_W_C0 = T_W_G0 @ T_G_C
        T_W_o0 = T_W_C0 @ T_C0_o0

        T_W_E1 = T_W_o0.copy()
        rot = R.from_matrix(T_W_E1[:3,:3]).as_euler("xyz", degrees=True)
        rot[0], rot[1] = 180.0, 0.0
        T_W_E1[:3,:3] = R.from_euler("xyz", rot, degrees=True).as_matrix()

        T_W_G1 = T_W_E1 @ T_E_G

        top_pos, top_R, bottom_pos, bottom_R = top_and_bottom_face_centers(bbox)
        p_C0_o0 = np.asarray(top_pos.points)[0]   # 或 bottom
        # visualize_chain_in_camera_origin(
        #     p_C0_o0,
        #     T_W_G0=T_W_G0,
        #     T_G_C=T_G_C,
        #     T_E_G=T_E_G,
        #     extra_geoms=[point_cloud],  # 你的 open3d 點雲（相機座標）
        # )
        return T_W_G1
    temp = np.eye(4)
    temp[:3,:3] = R.from_euler('xyz',last_pos[3:],degrees=False).as_matrix()
    temp[:3,3] = np.array(last_pos[:3])
  
    top_pos.points[0][1] += bias ######## add bias
    temp = c2e(np.asarray(top_pos.points[0]), temp)

    rot = R.from_matrix(temp[:3,:3]).as_euler('xyz',degrees=False)
    grasp_pos = [temp[0,3], temp[1,3], temp[2,3],rot[0],rot[1],rot[2]]
    grasp_pos[2] = grasp_pos[2] - 0.02
    print(grasp_pos)
    #   move to grasp position
    response = armCmd.set_positions(grasp_pos)
    while not armCmd.is_arrived():
        rclpy.spin_once(armCmd)
    print("move",armCmd.target_positions)
    armCmd.send_gripper(0.029)
    time.sleep(1.5)

    # move
    #   move to top position
    response = armCmd.set_positions([0.55, -0.3, 0.4, 3.14, 0.0, -1.57])
    while not armCmd.is_arrived():
        rclpy.spin_once(armCmd)
    print("move",armCmd.target_positions)

    # insert
    #   move to insert position
    horizon_pos = rotCam([0.44, -0.04, 0.42, 3.14, 0.0, -1.57],[180,0,-90])
    # horizon_pos = rotCam([0.4, 0.2, 0.35, 3.14, 0.0, -1.57],[180,-45,-90])
    response = armCmd.set_positions(horizon_pos.tolist())
    while not armCmd.is_arrived():
        rclpy.spin_once(armCmd)
    print("move",armCmd.target_positions)
    time.sleep(1.0)

    #   ICP
    for i in range(3):
        print("[Debug] try count =", i)
        data = iCPNode.get_cam_data()
        intr = iCPNode.get_cam_info()
        color_img, depth_img, point_cloud = data
        # point_cloud = pc2_to_open3d(point_cloud)
        point_cloud = create_point_cloud(depth_img, color_img, intr['width'], intr['height'], intr['fx'], intr['fy'], intr['ppx'], intr['ppy'])
        point_cloud = filter_pcd_by_depth_percentile(point_cloud,keep_percentile=95)
        field_img_mask_center, field_img_colored_mask, field_masks = generate_masks(color_img)
        mask,_ = get_cylinder_mask(
                [],
                field_img_mask_center,
                field_masks
            )
        src = masked_pointcloud_from_o3d(point_cloud,mask,intr)
        src_np = np.asarray(src.points)
        tgt = mesh_to_pcd("/home/lab606/ICA_Lab_UMI/ros2_ws/src/ipc_test/peg_30.5mm - Part 1.stl")
        T, bbox, result_icp = get_bbox_by_icp(src, tgt, viz=True, all_pcd=point_cloud)
        if result_icp.fitness >= 0.75 and result_icp.inlier_rmse <= 0.001:
            break
        if result_icp.inlier_rmse <= 0.001:
            break

    top_pos, top_R, bottom_pos, bottom_R = top_and_bottom_face_centers(bbox)

    T_C_P = np.eye(4)
    bottom_pos.points[0][1] += bias ######## add bias
    T_C_P[:3,3] = np.asarray(bottom_pos.points[0])
    T_C_P[:3,:3] = bottom_R
    T_P_C = np.linalg.inv(T_C_P)

    print("btn_peg_tcp",T_C_P)

    #   hole pos estimation
    for i in range(3):
        print("[Debug] try count =", i)
        data = iCPNode.get_cam_data()
        intr = iCPNode.get_cam_info()
        color_img, depth_img, point_cloud = data
        # point_cloud = pc2_to_open3d(point_cloud)
        point_cloud = create_point_cloud(depth_img, color_img, intr['width'], intr['height'], intr['fx'], intr['fy'], intr['ppx'], intr['ppy'])
        point_cloud = filter_pcd_by_depth_percentile(point_cloud,keep_percentile=95)
        mask,_ = get_platform_mask(field_img_mask_center, field_masks)
        src = masked_pointcloud_from_o3d(point_cloud,mask,intr)
        tgt = mesh_to_pcd("/home/lab606/ICA_Lab_UMI/ros2_ws/src/ipc_test/hole_assembly.stl")
        T, bbox, result_icp = get_bbox_by_icp(src, tgt, viz=True, all_pcd=point_cloud)
        if result_icp.inlier_rmse < 0.0015 and result_icp.fitness >= 0.7:
            data = iCPNode.get_cam_data()
            intr = iCPNode.get_cam_info()
            color_img, depth_img, point_cloud = data
            point_cloud = pc2_to_open3d(point_cloud)
            point_cloud = filter_pcd_by_depth_percentile(point_cloud,keep_percentile=95)
            break
    four_hole_points = top_face_inset_corners_as_pcd(bbox)
    four_hole_points = np.asarray(four_hole_points.points)
    print("four_hole_points",four_hole_points)
    selected_hole = four_hole_points[int(input("select hole 0~3:"))]
    print("selected_hole",selected_hole)
    selected_hole[1] += bias ######## add bias
    #   touch
    T_W_G = np.eye(4)
    T_W_G[:3,:3] = R.from_euler('xyz',horizon_pos[3:], degrees=False).as_matrix()
    T_W_G[:3,3] = horizon_pos[:3]
    selected_hole_c_h = np.array([selected_hole[0],selected_hole[1],selected_hole[2],1.0]).T
    selected_hole_W_h = T_W_G @ T_G_C @ selected_hole_c_h
    print("selected_hole_W_h",selected_hole_W_h)

    T_W_G0 = np.eye(4)
    T_W_G0[:3,:3] = R.from_euler('xyz',[180.0,0.0,-90.0], degrees=True).as_matrix()
    T_W_G0[:3,3] = np.array([0.46, -0.04, 0.38])
    T_W_P0 = T_W_G0 @ T_G_C @ T_C_P
    print("T_W_P0",T_W_P0)
    T_W_P1 = T_W_P0.copy()
    T_W_P1[:3,3] = selected_hole_W_h[:3]
    T_W_G1 = T_W_P1 @ T_P_C @ T_C_G
    
    selected_hole_c = np.array(selected_hole[:3], dtype=np.float64)  # 你說點雲原點是相機，所以這是 C 座標
    out = visualize_process_in_camera_origin(
        T_W_G0=T_W_G0,
        T_G_C=T_G_C,
        T_C_P=T_C_P,
        T_P_C=T_P_C,
        T_C_G=T_C_G,
        selected_hole_c=selected_hole_c,
        point_cloud_c=point_cloud,   # 這個 pcd 必須是相機座標
        frame_size=0.06,
    )
    touch_pos = np.array([T_W_G1[0,3],T_W_G1[1,3],T_W_G1[2,3],0.0,0.0,0.0])
    touch_pos[3:] = R.from_matrix(T_W_G1[:3,:3]).as_euler('xyz', degrees=False)
    print("touch_pos",touch_pos)
    input("touch?")
    response = armCmd.set_positions(touch_pos.tolist())
    while not armCmd.is_arrived():
        rclpy.spin_once(armCmd)
    print("move",armCmd.target_positions)
    time.sleep(2.0)

    # def plan_insert_traj(holePos_W_G,seeholePos_W_G,r=0.1,th1=15.0,cylinderLen=0.09, c = -0.000,b=0.004):
        
    #     holePos_W_P = self.g2p(holePos_W_G,cylinderLen)

    #     x0,y0,z0 = seeholePos_W_G[1],seeholePos_W_G[2],seeholePos_W_G[3]
    #     x2,y2,z2 = holePos_W_P[1],holePos_W_P[2],holePos_W_P[3]
        
    #     th2 = math.atan2(x2 - x0, y2 - y0)
    #     th1 = math.radians(th1)

    #     print(th1,th2)

    #     rz = -math.degrees(th2)
    #     rx = math.degrees(th1)
    #     print(rx,-rz)
    #     l = r*math.sin(th1)
        
    #     x2,y2 = x2 + b*math.sin(th2), y2 + b*math.cos(th2)
    #     x1,y1,z1 = -l*math.sin(th2)+x2,  -l*math.cos(th2)+y2,  r*math.cos(th1)+z2
    #     holePos_W_P_approch = np.array([0.0, x1, y1, z1, 0.0, 0.0, 0.0, 1.0, self.gripLen])
    #     holePos_W_P_approch[4:8] = R.from_euler('xyz', [180.0+rx, 0.0, rz], degrees=True).as_quat()
    #     holePos_W_P[4:8] = R.from_euler('xyz', [180.0+rx, 0.0, rz], degrees=True).as_quat()
    #     holePos_W_P[1],holePos_W_P[2],holePos_W_P[3] = x2,y2,z2
    #     holePos_W_G_approch = self.p2g(holePos_W_P_approch,cylinderLen)
    #     holePos_W_G_touch = self.p2g(holePos_W_P,cylinderLen)

    #     print(l)
    #     print()
    #     holePos_W_G_final = holePos_W_G_touch.copy()
    #     holePos_W_G_final[1] = (l+c)*math.sin(th2)+holePos_W_G_touch[1]
    #     holePos_W_G_final[2] = (l+c)*math.cos(th2)+holePos_W_G_touch[2]
    #     holePos_W_G_final[3] = holePos_W_G_touch[3] - 0.005
    #     holePos_W_E_final = self.g2e(holePos_W_G_final)
    #     holePos_W_E_final[4:8] = R.from_euler('xyz', [180.0, 0.0, rz], degrees=True).as_quat()
    #     holePos_W_G_final = self.e2g(holePos_W_E_final)

    #     return holePos_W_G_approch, holePos_W_G_touch ,holePos_W_G_final




    armCmd.destroy_node()
    iCPNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()