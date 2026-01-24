from sensor_msgs.msg import Image,PointCloud2,CameraInfo
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

class ICPNode(Node):
    def __init__(self):
        super().__init__('icp_node')

        self.cam_queue = queue.Queue(2)
        self.cam_info_queue = queue.Queue(1)

        self.color_img_sub = Subscriber(self, Image, '/camera/color/image_rect_raw')
        self.depth_img_sub = Subscriber(self, Image, '/camera/depth/image_rect_raw')
        self.points_sub = Subscriber(self, PointCloud2, '/camera/depth/color/points')

        self.cam_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.cam_info_callback,
            10
        )

        self.cv_bridge = CvBridge()
        self.ts = ApproximateTimeSynchronizer(
            [self.color_img_sub, self.depth_img_sub, self.points_sub],
            queue_size=10,
            slop=0.051  # 1/60+1/30 + 0.001 (30 hz, 60 hz)
        )
        self.ts.registerCallback(self.sync_cam_callback)
        self.get_logger().info('ICP Node has been started.')

    def sync_cam_callback(self, color_img, depth_img, point_cloud):
        if self.cam_queue.full():
            self.cam_queue.get()
        point_cloud_o3d = pc2_to_open3d(point_cloud)
        color_img_cv = self.cv_bridge.imgmsg_to_cv2(color_img)
        depth_img_cv = self.cv_bridge.imgmsg_to_cv2(depth_img)
        self.cam_queue.put((color_img_cv, depth_img_cv, point_cloud_o3d))
        # self.get_logger().info('color_img timestamp: %.6f' % color_img.header.stamp.sec + '%.6f' % (color_img.header.stamp.nanosec*1e-9))
        # self.get_logger().info('depth_img timestamp: %.6f' % depth_img.header.stamp.sec + '%.6f' % (depth_img.header.stamp.nanosec*1e-9))
        # self.get_logger().info('point_cloud timestamp: %.6f' % point_cloud.header.stamp.sec + '%.6f' % (point_cloud.header.stamp.nanosec*1e-9))
    
    def get_cam_data(self):
        if self.cam_queue.empty():
            return None
        return self.cam_queue.get()

    def cam_info_callback(self, msg):
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

def startNode():
    rclpy.init()
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
    # 1) 粗配準：FPFH + FGR
    # result_ransac = global_registration_ransac(src_down, tgt_down, src_fpfh, tgt_fpfh, voxel_size)
    result_fgr = fgr(src_down, tgt_down, src_fpfh, tgt_fpfh, voxel_size)
    print("[FGR] fitness:", result_fgr.fitness, "rmse:", result_fgr.inlier_rmse)
    print("[FGR] T=\n", result_fgr.transformation)
    T = result_fgr.transformation
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
    _,_, peg_btn_point, rot = top_and_bottom_face_centers(bbox)
    print("[ICP] time   :", time.time() - start)
    print("[ICP] fitness:", result_icp.fitness)
    print("[ICP] rmse   :", result_icp.inlier_rmse)
    print("[ICP] R=\n", R)
    print("[ICP] deg=\n", Rotation.from_matrix(R).as_euler("xyz", degrees=True))
    print("[ICP] t=\n", t)
    print("[ICP] T=\n", T)
    print("[ICP] bbox=\n", np.asarray(bbox.get_box_points()))

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
        T[:3, :3] = rot
        T[:3,  3] = p
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
        frame.transform(T)  # 套用完整位姿


        geoms = [t,s, coor,hole_points,peg_btn_point,frame]
        if bbox is not None:
            bbox_ls = o3d.geometry.LineSet.create_from_oriented_bounding_box(bbox)
            bbox_ls.paint_uniform_color([1, 0, 0])  # 線框紅色
            geoms.append(bbox_ls)
        if all_pcd is not None:
            geoms.append(all_pcd)

        o3d.visualization.draw_geometries(geoms)
    return T, bbox ,result_icp, hole_points , peg_btn_point

peg_tgt = mesh_to_pcd("peg_30.5mm - Part 1.stl")
hole_tgt = mesh_to_pcd("hole_assembly.stl")

def main():
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
        src = peg_tgt
        tgt = mesh_to_pcd("peg_30.5mm - Part 1.stl")
        T, bbox, result_icp,_,_ = get_bbox_by_icp(src, tgt, viz=True, all_pcd=point_cloud)

        
        target_mask, target_id = get_platform_mask(field_img_mask_center, field_masks)
        print("[Debug] target_id =", target_id)
        cv2.imshow("field_img_mask_center", field_img_mask_center)
        cv2.imshow("field_img_colored_mask", field_img_colored_mask)
        visualize_mask(color_img, target_mask, "target_mask")
        cv2.waitKey(1000)

        src = masked_pointcloud_from_o3d(point_cloud, target_mask, intr)
        tgt = hole_tgt
        get_bbox_by_icp(src, tgt, viz=True, all_pcd=point_cloud)

        
    else:
        print("No data received.")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
