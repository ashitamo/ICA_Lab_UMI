import copy
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation
from typing import Any, Dict, List, Optional, Tuple
import os,pickle

def mesh_to_pcd(mesh: o3d.geometry.TriangleMesh | str) -> o3d.geometry.PointCloud:
    print("[Debug] mesh_to_pcd")
    if isinstance(mesh, str):
        # mode paht sub format to .pkl
        pcd_pkl_path = mesh.replace(".stl", ".pkl")
        if os.path.exists(pcd_pkl_path):
            pointcloud_np = pickle.load(open(pcd_pkl_path, "rb"))
            pointcloud = o3d.geometry.PointCloud()
            pointcloud.points = o3d.utility.Vector3dVector(pointcloud_np)
            print("[Debug] load pointcloud from", pcd_pkl_path, "pointcloud points =", len(pointcloud.points))
            return pointcloud
        pcd_mesh = o3d.io.read_triangle_mesh(mesh)
    pointcloud = pcd_mesh.sample_points_poisson_disk(100000)
    if isinstance(mesh, str):
        pickle.dump(np.asarray(pointcloud.points, dtype=np.float64), open(pcd_pkl_path, "wb"))
    print("[Debug] pointcloud points =", len(pointcloud.points))
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

def preprocess_point_cloud(pcd: o3d.geometry.PointCloud, voxel_size: float):
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 3.0
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30)
    )

    radius_feature = voxel_size * 10.0
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
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.85),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(dist_thresh),
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(500000, 0.999)
    )
    return result

def fgr(src_down, tgt_down, src_fpfh, tgt_fpfh, voxel_size: float):
    dist_thresh = voxel_size * 5.0
    result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        src_down, tgt_down, src_fpfh, tgt_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=dist_thresh
        )
    )
    return result

def refine_icp(src, tgt, init_T, voxel_size: float):
    dist_thresh = voxel_size * 1.0
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
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=1000)
    )
    
    return result

def multiscale_refine_icp(src, tgt, T_init, voxel_size):
    T = T_init.copy()
    for t in range(5):
        for i in [2.0, 1.0, 0.5, 0.25, 0.1]:
            if len(src.points) < 20000 and i == 0.25 and i == 0.1:
                continue
            v = i * voxel_size
            src_d = src.voxel_down_sample(v)
            tgt_d = tgt.voxel_down_sample(v)

            # point-to-plane 需要 normals（至少 target 要有）
            tgt_d.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=v*2.0, max_nn=30))
            src_d.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=v*2.0, max_nn=30))

            result = o3d.pipelines.registration.registration_icp(
                src_d, tgt_d,
                max_correspondence_distance=v * 3.0,
                init=T,
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=500)
            )

            T = result.transformation
            print("[ICP] voxel size:", v, "fitness:", result.fitness , "rmse:", result.inlier_rmse)
        if result.fitness > 0.8 or t>2 :
            break
    return result

def top_face_inset_corners_as_pcd(
    bbox: o3d.geometry.OrientedBoundingBox,
    inset_ratio: float = 43.5 / 174.0,
    origin: np.ndarray | None = None,   # 相機原點；預設 [0,0,0]
) -> o3d.geometry.PointCloud:
    """
    根據 minimal OBB，取「離相機較近」那一面的四個內縮角點。
    - 厚度軸 w：extent 最小那個軸
    - 兩個候選面中心：c ± w*(thickness/2)
    - 選到 origin 距離較小的那一面
    - 內縮：沿面內 u/v 各從邊緣往內退 inset_ratio * (該方向全長)

    回傳：含 4 點的 PointCloud
    """
    if origin is None:
        origin = np.zeros(3, dtype=np.float64)
    else:
        origin = np.asarray(origin, dtype=np.float64).reshape(3)

    c = np.asarray(bbox.center, dtype=np.float64)
    Rm = np.asarray(bbox.R, dtype=np.float64)          # columns are box axes
    ext = np.asarray(bbox.extent, dtype=np.float64)    # lengths along each axis

    # 1) 厚度軸：extent 最小那個
    k_thin = int(np.argmin(ext))
    w = Rm[:, k_thin]
    thickness = float(ext[k_thin])

    # 2) 面內兩軸 u, v
    idx = [0, 1, 2]
    idx.remove(k_thin)
    i_u, i_v = idx[0], idx[1]

    u = Rm[:, i_u]
    v = Rm[:, i_v]
    Lu = float(ext[i_u])
    Lv = float(ext[i_v])

    # 3) 兩個候選面中心：選離 origin 較近的
    c_plus  = c + w * (thickness * 0.5)
    c_minus = c - w * (thickness * 0.5)

    if np.linalg.norm(c_plus - origin) <= np.linalg.norm(c_minus - origin):
        c_face = c_plus
    else:
        c_face = c_minus

    # 4) 內縮後的半長
    frac = float(inset_ratio)
    hu = Lu * (0.5 - frac)
    hv = Lv * (0.5 - frac)
    if hu <= 0 or hv <= 0:
        raise ValueError(f"inset_ratio={frac} too large for extents Lu={Lu}, Lv={Lv}")

    # 5) 四個內縮角點
    corners = np.stack([
        c_face + hu*u + hv*v,
        c_face + hu*u - hv*v,
        c_face - hu*u - hv*v,
        c_face - hu*u + hv*v,
    ], axis=0)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(corners)
    return pcd


def top_and_bottom_face_centers(
    bbox: o3d.geometry.OrientedBoundingBox,
) -> Tuple[
    o3d.geometry.PointCloud, np.ndarray,   # top_pc, top_R
    o3d.geometry.PointCloud, np.ndarray    # bottom_pc, bottom_R
]:
    """
    找 OBB 六個面中心中：
      - 離原點最近的面 = top
      - 離原點最遠的面 = bottom

    每個面都回傳：
      - pc: 只含 1 點（面中心點）
      - R_face: 3x3 rotation，z 軸為該面的外側法向量，x/y 在面內（右手系）
    """
    c = np.asarray(bbox.center, dtype=np.float64)
    Rm = np.asarray(bbox.R, dtype=np.float64)       # columns are local axes
    ext = np.asarray(bbox.extent, dtype=np.float64)

    axes = [Rm[:, 0], Rm[:, 1], Rm[:, 2]]

    # 收集 6 個面：(dist, axis_index, sign, face_center)
    faces = []
    for i, a in enumerate(axes):
        half = 0.5 * float(ext[i])
        for s in (+1.0, -1.0):
            fc = c + s * a * half
            d = np.linalg.norm(fc)
            faces.append((d, i, s, fc))

    # 最近=top，最遠=bottom
    top = min(faces, key=lambda x: x[0])
    bottom = max(faces, key=lambda x: x[0])

    def _face_pose(face_tuple):
        _, k_axis, _, face_center = face_tuple
        w = axes[k_axis]

        # 外側法向量：從 bbox.center 指向該面中心（一定朝外）
        z_axis = face_center - c
        z_axis = z_axis / (np.linalg.norm(z_axis) + 1e-12)

        # 面內兩軸：取另外兩根 bbox 軸
        idx = [0, 1, 2]
        idx.remove(k_axis)
        x_axis = axes[idx[0]]
        y_axis = axes[idx[1]]

        # 正交化 + 右手系
        x_axis = x_axis - z_axis * np.dot(x_axis, z_axis)
        x_axis = x_axis / (np.linalg.norm(x_axis) + 1e-12)
        y_axis = np.cross(z_axis, x_axis)
        y_axis = y_axis / (np.linalg.norm(y_axis) + 1e-12)

        R_face = np.column_stack([x_axis, y_axis, z_axis])

        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector([face_center])

        return pc, R_face

    top_pc, top_R = _face_pose(top)
    bottom_pc, bottom_R = _face_pose(bottom)

    return top_pc, top_R, bottom_pc, bottom_R

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

