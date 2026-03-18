from sensor_msgs.msg import Image,PointCloud2,CameraInfo
from sensor_msgs_py import point_cloud2
from tm_msgs.msg import FeedbackState
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
def pc2_to_open3d(
    msg,
    *,
    remove_nans: bool = True,
    keep_rgb: bool = True,
) -> o3d.geometry.PointCloud:
    """
    使用 ROS2 官方 read_points 將 PointCloud2 轉為 Open3D PointCloud
    - 正確性優先（對齊 RViz）
    """

    field_names = ("x", "y", "z")
    if keep_rgb:
        field_names = ("x", "y", "z", "rgb")

    points_iter = point_cloud2.read_points(
        msg,
        field_names=field_names,
        skip_nans=False,   # 我們自己控制
    )

    xyz_list = []
    rgb_list = [] if keep_rgb else None

    for p in points_iter:
        x, y, z = p[0], p[1], p[2]

        if remove_nans:
            if not np.isfinite(x) or not np.isfinite(y) or not np.isfinite(z):
                continue

        xyz_list.append([x, y, z])

        if keep_rgb:
            rgb_f = p[3]
            rgb_u32 = np.asarray(rgb_f, dtype=np.float32).view(np.uint32)
            r = (rgb_u32 >> 16) & 255
            g = (rgb_u32 >> 8) & 255
            b = rgb_u32 & 255
            rgb_list.append([r / 255.0, g / 255.0, b / 255.0])

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(
        np.asarray(xyz_list, dtype=np.float64)
    )

    if keep_rgb and rgb_list:
        pcd.colors = o3d.utility.Vector3dVector(
            np.asarray(rgb_list, dtype=np.float64)
        )

    return pcd

def create_point_cloud(depth_image, color_image, width, height, fx, fy, cx, cy):
    """根据深度图、RGB图和相机位姿生成彩色点云"""
    depth_o3d = o3d.geometry.Image(depth_image.astype(np.float32))
    color_o3d = o3d.geometry.Image(color_image)
    
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_o3d, depth_o3d, depth_scale=1000.0, depth_trunc=3.0, convert_rgb_to_intensity=False)
    
    intrinsic = o3d.camera.PinholeCameraIntrinsic(
        o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
    
    intrinsic.set_intrinsics(width=width, height=height, fx=fx, fy=fy, cx=cx, cy=cy)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd_image, intrinsic)
    
    return pcd



class ICPNode(Node):
    def __init__(self):
        super().__init__('icp_node')
        self.cv_bridge = CvBridge()
        self.cam_queue = queue.Queue(2)
        self.cam_info_queue = queue.Queue(1)

        self.cam_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.cam_info_callback,
            10
        )

        self.color_img_sub = Subscriber(self, Image, '/camera/color/image_rect_raw')
        self.depth_img_sub = Subscriber(self, Image, '/camera/depth/image_rect_raw')
        self.points_sub = Subscriber(self, PointCloud2, '/camera/depth/color/points')
        self.feedback_sub = Subscriber(self, FeedbackState, '/feedback_states')
        
        self.ts = ApproximateTimeSynchronizer(
            [self.color_img_sub, self.depth_img_sub, self.points_sub, self.feedback_sub],
            queue_size=10,
            slop=0.051  # 1/60+1/30 + 0.001 (30 hz, 60 hz)
        )
        self.ts.registerCallback(self.sync_callback)
        self.get_logger().info('ICP Node has been started.')
        self.is_logged = True

    def sync_callback(self, color_img, depth_img, point_cloud, feedback_state):
        if self.is_logged:
            self.get_logger().info('color_img timestamp: %.6f' % color_img.header.stamp.sec + '%.6f' % (color_img.header.stamp.nanosec*1e-9))
            self.get_logger().info('depth_img timestamp: %.6f' % depth_img.header.stamp.sec + '%.6f' % (depth_img.header.stamp.nanosec*1e-9))
            self.get_logger().info('point_cloud timestamp: %.6f' % point_cloud.header.stamp.sec + '%.6f' % (point_cloud.header.stamp.nanosec*1e-9))
            self.get_logger().info('feedback_state timestamp: %.6f' % feedback_state.header.stamp.sec + '%.6f' % (feedback_state.header.stamp.nanosec*1e-9))
        
        if self.cam_queue.full():
            self.cam_queue.get()

        color_img_cv = self.cv_bridge.imgmsg_to_cv2(color_img)
        depth_img_cv = self.cv_bridge.imgmsg_to_cv2(depth_img)
        tcp_pose = feedback_state.tool_pose
        self.cam_queue.put((color_img_cv, depth_img_cv, point_cloud, tcp_pose))
        
    def get_data(self):
        if self.cam_queue.empty():
            return None
        return self.cam_queue.get()

    def cam_info_callback(self, msg):
        if self.is_logged:
            self.get_logger().info('cam_info timestamp: %.6f' % msg.header.stamp.sec + '%.6f' % (msg.header.stamp.nanosec*1e-9))

        if self.cam_info_queue.full():
            self.cam_info_queue.get()
        self.cam_info_queue.put(msg)
        
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

    def is_ready(self):
        return not self.cam_queue.empty() and not self.cam_info_queue.empty()
def filter_pcd_by_depth_percentile(
    pcd: o3d.geometry.PointCloud,
    z_axis: int = 2,
    keep_percentile: float = 99.5,   # 保留到 99.5% 的深度
    z_min: float = 0.05,
) -> o3d.geometry.PointCloud:
    pts = np.asarray(pcd.points)
    if pts.size == 0:
        return pcd

    valid = np.isfinite(pts).all(axis=1)
    pts_v = pts[valid]

    z = pts_v[:, z_axis]
    z = z[z > z_min]
    if z.size == 0:
        return pcd

    z_max = np.percentile(z, keep_percentile)

    keep = (pts_v[:, z_axis] >= z_min) & (pts_v[:, z_axis] <= z_max)
    idx = np.where(valid)[0][keep]
    return pcd.select_by_index(idx)

def startNode():
    iCPNode = ICPNode()
    executor = MultiThreadedExecutor()
    executor.add_node(iCPNode)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    return iCPNode

def get_bbox_by_icp(src, tgt ,viz=False , all_pcd=None) -> Tuple[np.ndarray, o3d.geometry.OrientedBoundingBox, o3d.pipelines.registration.RegistrationResult]:
    start = time.time()
    extent = np.linalg.norm(np.asarray(tgt.get_max_bound()) - np.asarray(tgt.get_min_bound()))
    voxel_size = max(extent / 50.0, 1e-5)  # 視尺度可改 /30, /100
    print("[Debug] voxel_size =", voxel_size)
    print("[Debug] src points =", len(src.points))
    T = np.eye(4)
    src_down, src_fpfh = preprocess_point_cloud(src, voxel_size)
    tgt_down, tgt_fpfh = preprocess_point_cloud(tgt, voxel_size)
    print("[Debug] src_down points =", len(src_down.points))
    print("[Debug] tgt_down points =", len(tgt_down.points))
    if len(src_down.points) <= 100 or len(tgt_down.points) == 100:
        return np.eye(4), o3d.geometry.OrientedBoundingBox(), o3d.pipelines.registration.RegistrationResult()
    # 1) 粗配準：FPFH + RANSAC
    for i in range(3):
        print("[Debug] try RANSAC FGR count =", i)
        result_fgr = fgr(src_down, tgt_down, src_fpfh, tgt_fpfh, voxel_size)
        print("[FGR] fitness:", result_fgr.fitness, "rmse:", result_fgr.inlier_rmse)
        print("[FGR] T=\n", result_fgr.transformation)

        result_ransac = global_registration_ransac(src_down, tgt_down, src_fpfh, tgt_fpfh, voxel_size)
        print("[RANSAC] fitness:", result_ransac.fitness, "rmse:", result_ransac.inlier_rmse)
        print("[RANSAC] T=\n", result_ransac.transformation)
        rot_fgr = Rotation.from_matrix(result_fgr.transformation[:3, :3].copy()).as_euler('xyz', degrees=True)
        rot_ransac = Rotation.from_matrix(result_ransac.transformation[:3, :3].copy()).as_euler('xyz', degrees=True)
        
        if abs(rot_fgr[0])<90.0 and abs(rot_ransac[0])<90.0:
            if result_ransac.fitness > result_fgr.fitness:
                result = result_ransac
                print("[ICP] use RANSAC")
            else:
                result = result_fgr
                print("[ICP] use FGR")
        elif abs(rot_fgr[0])<90.0:
            result = result_fgr
            print("[ICP] use FGR")
        elif abs(rot_ransac[0])<90.0:
            result = result_ransac
            print("[ICP] use RANSAC")
        else:
            if result_ransac.fitness > result_fgr.fitness:
                result = result_ransac
                print("[ICP] use RANSAC")
            else:
                result = result_fgr
                print("[ICP] use FGR")
            continue
        if result.fitness > 0.8:
            break
    if result.fitness < 0.2:
        return np.eye(4), o3d.geometry.OrientedBoundingBox(), o3d.pipelines.registration.RegistrationResult()
    T = result.transformation
    # 2) 精配準：ICP（point-to-plane）
    
    result_icp = multiscale_refine_icp(src, tgt, T, voxel_size)
    T = result_icp.transformation
    T = np.linalg.inv(T)
    R = T[:3, :3]
    t = T[:3, 3]

    tgt_aligned = copy.deepcopy(tgt)
    tgt_aligned.transform(T)

    # 你可以選 AABB 或 OBB
    bbox = tgt_aligned.get_minimal_oriented_bounding_box()  # OBB 比較貼物體
    hole_points = top_face_inset_corners_as_pcd(bbox)
    peg_top_point, peg_top_rot, peg_btn_point, peg_btn_rot = top_and_bottom_face_centers(bbox)
    print("[ICP] time   :", time.time() - start)
    print("[ICP] fitness:", result_icp.fitness)
    print("[ICP] rmse   :", result_icp.inlier_rmse)
    print("[ICP] R=\n", R)
    print("[ICP] deg=\n", Rotation.from_matrix(R).as_euler("xyz", degrees=True))
    print("[ICP] t=\n", t)
    print("[ICP] T=\n", T)
    print("[ICP] bbox=\n", np.asarray(bbox.get_box_points()))
    
    
    all_pcd = filter_pcd_by_depth_percentile(all_pcd)
    if viz:
        s = copy.deepcopy(src)
        t = copy.deepcopy(tgt)

        # s.paint_uniform_color([1, 0.706, 0])
        t.paint_uniform_color([0, 0.651, 0.929])
        # 注意：你這裡是把 target 變到 source/camera frame
        t.transform(T)

        coor = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=[0, 0, 0])
        p = np.asarray(peg_btn_point.points)[0]   # 底面中心點 (3,)
        T = np.eye(4)
        T[:3, :3] = peg_top_rot
        T[:3,  3] = p
        frame_peg_btn = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
        frame_peg_btn.transform(T)  # 套用完整位姿

        p= np.asarray(peg_top_point.points)[0]   # 上面中心點 (3,)
        T = np.eye(4)
        T[:3, :3] = peg_top_rot
        T[:3,  3] = p
        frame_peg_top = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
        frame_peg_top.transform(T)


        geoms = [s,t,coor,hole_points,peg_btn_point,frame_peg_btn,frame_peg_top]
        if bbox is not None:
            bbox_ls = o3d.geometry.LineSet.create_from_oriented_bounding_box(bbox)
            bbox_ls.paint_uniform_color([1, 0, 0])  # 線框紅色
            geoms.append(bbox_ls)
        if all_pcd is not None:
            geoms.append(all_pcd)

        o3d.visualization.draw_geometries(geoms)
    return T, bbox ,result_icp

def main():
    rclpy.init()
    iCPNode = startNode()
    while True:
        if iCPNode.is_ready():
            break
        time.sleep(0.1)
    data = iCPNode.get_cam_data()
    intr = iCPNode.get_cam_info()

    if data is not None:
        color_img, depth_img, point_cloud = data
        field_img_mask_center, field_img_colored_mask, field_masks = generate_masks(color_img)

        mask,_ = get_cylinder_mask(
            [],
            field_img_mask_center,
            field_masks
        )
        src = masked_pointcloud_from_o3d(point_cloud, mask, intr)
        tgt = mesh_to_pcd("/home/lab606/ICA_Lab_UMI/ros2_ws/src/ipc_test/peg_30.5mm - Part 1.stl")
        T, bbox, result_icp = get_bbox_by_icp(src, tgt, viz=True, all_pcd=point_cloud)

        
        target_mask, target_id = get_platform_mask(field_img_mask_center, field_masks)
        print("[Debug] target_id =", target_id)
        cv2.imshow("field_img_mask_center", field_img_mask_center)
        cv2.imshow("field_img_colored_mask", field_img_colored_mask)
        visualize_mask(color_img, target_mask, "target_mask")
        cv2.waitKey(1000)

        src = masked_pointcloud_from_o3d(point_cloud, target_mask, intr)
        tgt = mesh_to_pcd("hole_assembly.stl")
        get_bbox_by_icp(src, tgt, viz=True, all_pcd=point_cloud)

        
    else:
        print("No data received.")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
