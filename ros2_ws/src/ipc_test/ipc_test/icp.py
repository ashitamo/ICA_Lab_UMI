import copy
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation

def mesh_to_pcd(mesh: o3d.geometry.TriangleMesh | str) -> o3d.geometry.PointCloud:
    if isinstance(mesh, str):
        mesh = o3d.io.read_triangle_mesh(mesh)
    pointcloud = mesh.sample_points_poisson_disk(100000)
    return pointcloud

def load_pts_xyz(path: str) -> o3d.geometry.PointCloud:
    pts = []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        lines = [ln.strip() for ln in f if ln.strip()]

    start = 0
    if len(lines[0].split()) == 1:
        try:
            int(float(lines[0]))
            start = 1
        except:
            pass

    for ln in lines[start:]:
        sp = ln.split()
        if len(sp) >= 3:
            pts.append([float(sp[0]), float(sp[1]), float(sp[2])])

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.asarray(pts, dtype=np.float64))
    return pcd

def pc2_to_open3d(msg, remove_nans=True) -> o3d.geometry.PointCloud:
    if msg.point_step != 20:
        raise ValueError(f"Expected point_step=20, got {msg.point_step}. This fast path assumes RealSense XYZ+RGB layout.")

    dtype = np.dtype([
        ("x",   np.float32),
        ("y",   np.float32),
        ("z",   np.float32),
        ("pad", np.uint32),   # 12~15 padding
        ("rgb", np.float32),  # 16~19 packed RGB
    ])

    arr = np.frombuffer(msg.data, dtype=dtype)  # (N,)
    xyz = np.stack([arr["x"], arr["y"], arr["z"]], axis=1).astype(np.float64, copy=False)

    if remove_nans:
        mask = np.isfinite(xyz).all(axis=1)
        xyz = xyz[mask]
        rgb_f = arr["rgb"][mask]
    else:
        rgb_f = arr["rgb"]

    rgb_u32 = rgb_f.view(np.uint32)
    r = ((rgb_u32 >> 16) & 255).astype(np.float64)
    g = ((rgb_u32 >> 8) & 255).astype(np.float64)
    b = (rgb_u32 & 255).astype(np.float64)
    colors = np.stack([r, g, b], axis=1) / 255.0

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd

def preprocess_point_cloud(pcd: o3d.geometry.PointCloud, voxel_size: float):
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2.0
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30)
    )

    radius_feature = voxel_size * 5.0
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100)
    )
    return pcd_down, fpfh


def global_registration_ransac(src_down, tgt_down, src_fpfh, tgt_fpfh, voxel_size: float):
    dist_thresh = voxel_size * 3.0
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        src_down, tgt_down, src_fpfh, tgt_fpfh,
        mutual_filter=True,
        max_correspondence_distance=dist_thresh,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        ransac_n=4,
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(dist_thresh),
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999)
    )
    return result


def refine_icp(src, tgt, init_T, voxel_size: float):
    dist_thresh = voxel_size * 0.5
    # point-to-plane 通常比 point-to-point 好，但 target 需要 normals
    if not tgt.has_normals():
        tgt.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2.0, max_nn=30)
        )

    result = o3d.pipelines.registration.registration_icp(
        src, tgt,
        max_correspondence_distance=dist_thresh,
        init=init_T,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200)
    )
    return result

def draw_registration_result(source, target, transformation, bbox=None):
    s = copy.deepcopy(source)
    t = copy.deepcopy(target)

    # s.paint_uniform_color([1, 0.706, 0])
    t.paint_uniform_color([0, 0.651, 0.929])

    # 注意：你這裡是把 target 變到 source/camera frame
    t.transform(transformation)

    coor = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=[0, 0, 0])

    geoms = [s, coor]
    if bbox is not None:
        bbox_ls = o3d.geometry.LineSet.create_from_oriented_bounding_box(bbox)
        bbox_ls.paint_uniform_color([1, 0, 0])  # 線框紅色
        geoms.append(bbox_ls)

    o3d.visualization.draw_geometries(geoms)

if __name__ == "__main__":
    tgt = load_pts_xyz("peg_30.pts")   # 參考/完美點雲：target
    src0 = load_pts_xyz("peg_30.pts")  # source（用同一份做 demo）

    # 讓 source 故意偏掉（你要 recover 的變換就是它的 inverse）
    R_gt = Rotation.from_euler("xyz", [-10, -10, 10], degrees=True).as_matrix()
    t_gt = np.array([1,2,3])
    T_gt = np.eye(4)
    T_gt[:3, :3] = R_gt
    T_gt[:3, 3] = t_gt

    src = copy.deepcopy(src0)
    src.transform(T_gt)

    import time
    start = time.time()
    # （可選）加噪
    pts = np.asarray(src.points)
    pts += np.random.normal(0, 0.0001, pts.shape)
    src.points = o3d.utility.Vector3dVector(pts)

    # 自動估一個 voxel_size（你也可以手動指定）
    extent = np.linalg.norm(np.asarray(tgt.get_max_bound()) - np.asarray(tgt.get_min_bound()))
    voxel_size = max(extent / 50.0, 1e-5)  # 視尺度可改 /30, /100

    src_down, src_fpfh = preprocess_point_cloud(src, voxel_size)
    tgt_down, tgt_fpfh = preprocess_point_cloud(tgt, voxel_size)

    # 1) 粗配準：FPFH + RANSAC
    result_ransac = global_registration_ransac(src_down, tgt_down, src_fpfh, tgt_fpfh, voxel_size)
    print("[RANSAC] fitness:", result_ransac.fitness, "rmse:", result_ransac.inlier_rmse)
    print("[RANSAC] T=\n", result_ransac.transformation)

    # 2) 精配準：ICP（point-to-plane）
    result_icp = refine_icp(src, tgt, result_ransac.transformation, voxel_size)
    T = result_icp.transformation
    temp = np.linalg.inv(T)
    R = temp[:3, :3]
    t = temp[:3, 3]
    print("[ICP] time   :", time.time() - start)
    print("[ICP] fitness:", result_icp.fitness)
    print("[ICP] rmse   :", result_icp.inlier_rmse)
    print("[ICP] R=\n", R)
    print("[ICP] deg=\n", Rotation.from_matrix(R).as_euler("xyz", degrees=True))
    print("[ICP] t=\n", t)
    print("[ICP] T=\n", T)

    draw_registration_result(src, tgt, T)
