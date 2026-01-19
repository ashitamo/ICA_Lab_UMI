def extract_top_plane(mask_obj, depth_u16, depth_scale, K,
                      dist_thresh=0.003, iters=2000, min_inliers=200):
    """
    用深度圖 + RANSAC (sklearn) 從物件遮罩中擬合出最主要平面（通常是圓柱頂面），
    再將整個 2D mask 透過「相機光線與該平面的交點」投影成假 3D 點雲，
    用這些假點雲求 3D 重心與對應 2D 影像座標。

    參數：
      mask_obj   : (H,W) uint8 / bool，物件遮罩 (>0 視為在 mask 內)
      depth_u16  : (H,W) uint16，深度影像（同一個視角）
      depth_scale: float，將 depth_u16 轉成公尺的比例
      K          : dict，內參，包含 "fx","fy","ppx","ppy"
      dist_thresh: RANSAC 殘差閾值（這裡是 Z 軸上的高度殘差，約略等於平面距離）
      iters      : RANSAC 最大迭代次數 (max_trials)
      min_inliers: 最少需要的內點數量（同時也會給 RANSAC min_samples 的下限）

    回傳：
      mask_top : (H,W) uint8，RANSAC 找到的平面內點在影像中的遮罩（只用有深度的點）
      center3d : (3,) np.array，整個 mask 投影到平面後的假點雲 3D 重心 [Xc, Yc, Zc]
      normal3d : (3,) np.array，平面法向量（Z 分量被統一成朝 +Z 的方向）
      center2d : (2,) np.array，center3d 使用相機內參投影回影像的座標 [uc, vc]
                 若無法計算（點在相機背後等）則為 None
    """
    import numpy as np
    from sklearn.linear_model import RANSACRegressor, LinearRegression

    fx, fy = K["fx"], K["fy"]
    ppx, ppy = K["ppx"], K["ppy"]

    # === 1. 取出整個物件 mask 上的像素位置（影像座標） ===
    ys_all, xs_all = np.where(mask_obj > 0)
    if len(xs_all) < min_inliers:
        return None, None, None, None

    # === 2. 用有深度的點來建 3D 點雲，給 RANSAC 用 ===
    Z_all = depth_u16[ys_all, xs_all] * depth_scale
    valid_depth = Z_all > 0
    xs_d = xs_all[valid_depth]
    ys_d = ys_all[valid_depth]
    Z = Z_all[valid_depth]

    if len(xs_d) < min_inliers:
        return None, None, None, None

    X = (xs_d - ppx) * Z / fx
    Y = (ys_d - ppy) * Z / fy
    pts = np.stack([X, Y, Z], axis=1)  # (N,3)

    # === 3. 使用 sklearn 的 RANSAC 擬合平面 Z = aX + bY + c ===
    # 特徵: [X, Y]，標籤: Z
    XY = pts[:, :2]   # (N,2)
    Z_target = pts[:, 2]  # (N,)

    # RANSAC：以 LinearRegression 當基底
    # residual_threshold 是 |Z - Z_pred|，接近平面的點才算 inlier
    base_est = LinearRegression()
    ransac = RANSACRegressor(
        estimator=base_est,
        min_samples=3,              # 至少 3 點
        residual_threshold=dist_thresh,
        max_trials=iters
    )


    try:
        ransac.fit(XY, Z_target)
    except ValueError:
        # 如 min_samples 設太大或資料不夠等情況
        return None, None, None, None

    inlier_mask = ransac.inlier_mask_
    if inlier_mask is None or np.count_nonzero(inlier_mask) < min_inliers:
        return None, None, None, None

    # 取得線性平面參數：Z = aX + bY + c
    coef = ransac.estimator_.coef_
    # 確保 coef 是數組形式
    if isinstance(coef, tuple):
        a, b = float(coef[0]), float(coef[1])
    else:
        coef = np.asarray(coef).flatten()
        a, b = float(coef[0]), float(coef[1])
    
    intercept = ransac.estimator_.intercept_
    c = float(intercept) if np.isscalar(intercept) else float(intercept[0])

    # 將其轉成標準平面形式：n·[X,Y,Z] + d = 0
    # Z - aX - bY - c = 0  →  -aX - bY + Z - c = 0
    normal3d = np.array([-a, -b, 1.0], dtype=np.float64)
    d_plane = -c

    # 正規化法向量
    norm_len = np.linalg.norm(normal3d)
    if norm_len < 1e-8:
        return None, None, None, None
    normal3d /= norm_len
    d_plane /= norm_len

    # 統一讓法向量 z 分量朝上（+Z）
    if normal3d[2] < 0:
        normal3d = -normal3d
        d_plane = -d_plane

    # === 4. 用 inliers 在影像平面上建立 top 平面 mask ===
    mask_top = np.zeros_like(mask_obj, dtype=np.uint8)
    mask_top[ys_d[inlier_mask], xs_d[inlier_mask]] = 1

    # 填充孔洞：保留所有 inlier，只填充內部的孔洞
    import cv2
    # 使用 floodFill 從邊界填充背景，剩下的就是前景+孔洞
    h, w = mask_top.shape
    mask_floodfill = mask_top.copy()
    # 創建比原圖大2的mask用於floodFill
    mask_floodfill_mask = np.zeros((h + 2, w + 2), np.uint8)
    # 從 (0,0) 開始填充外部背景
    cv2.floodFill(mask_floodfill, mask_floodfill_mask, (0, 0), 255)
    # 反轉得到前景+孔洞
    mask_floodfill_inv = cv2.bitwise_not(mask_floodfill)
    # 將原始inlier與填充後的結果合併
    mask_top = mask_top | mask_floodfill_inv

    # === 5. 把 inlier mask 投影到該平面上，得到「假點雲」 ===
    # 使用 mask_top (inliers) 而不是整個物件 mask
    ys_inlier, xs_inlier = np.where(mask_top > 0)
    
    if len(xs_inlier) < min_inliers:
        return mask_top, None, normal3d, None
    
    # 對 inlier mask 內的像素建立相機射線：dir = [(u-ppx)/fx, (v-ppy)/fy, 1]
    u_inlier = xs_inlier.astype(np.float64)
    v_inlier = ys_inlier.astype(np.float64)
    dirs = np.stack([(u_inlier - ppx) / fx,
                     (v_inlier - ppy) / fy,
                     np.ones_like(u_inlier)], axis=1)  # (N_inlier,3)

    # 射線與平面交點：
    # 平面：normal3d · X + d_plane = 0
    # 射線：X = t * dir
    # => t = -d_plane / (normal3d · dir)
    denom = dirs @ normal3d
    eps = 1e-8
    valid_dir = np.abs(denom) > eps  # 避免幾乎平行的射線

    if np.count_nonzero(valid_dir) < min_inliers:
        return mask_top, None, normal3d, None

    dirs_valid = dirs[valid_dir]
    denom_valid = denom[valid_dir]

    t = -d_plane / denom_valid  # (N_valid,)

    # 只保留 t > 0（平面在相機前方）
    positive = t > 0
    if np.count_nonzero(positive) < min_inliers:
        return mask_top, None, normal3d, None

    dirs_valid = dirs_valid[positive]
    t = t[positive]

    # 假點雲：每條射線乘上 t（真正的 3D 座標）
    fake_points = dirs_valid * t[:, None]  # (N_fake,3)

    # === 6. 在平面上擬合圓形並以圓心作為 3D 重心 ===
    # 先計算虛擬點雲的平均重心作為參考
    temp_center = fake_points.mean(axis=0)
    
    # 建立平面的局部座標系：使用兩個正交向量
    # 第一個軸：任意選擇一個與法向量不平行的向量
    if abs(normal3d[2]) < 0.9:
        arbitrary = np.array([0, 0, 1.0])
    else:
        arbitrary = np.array([1.0, 0, 0])
    
    u_axis = np.cross(normal3d, arbitrary)
    u_axis /= np.linalg.norm(u_axis)
    v_axis = np.cross(normal3d, u_axis)
    v_axis /= np.linalg.norm(v_axis)
    
    # 將虛擬點雲投影到平面 2D 座標
    # 以臨時重心為原點
    fake_points_centered = fake_points - temp_center
    u_coords = fake_points_centered @ u_axis
    v_coords = fake_points_centered @ v_axis
    points_2d = np.stack([u_coords, v_coords], axis=1)
    
    # 使用最小二乘法擬合圓：(x-cx)^2 + (y-cy)^2 = r^2
    # 展開：x^2 + y^2 = 2*cx*x + 2*cy*y + (r^2 - cx^2 - cy^2)
    # A = [2*x, 2*y, 1], b = x^2 + y^2
    A_circle = np.column_stack([2 * points_2d[:, 0], 
                                 2 * points_2d[:, 1], 
                                 np.ones(len(points_2d))])
    b_circle = np.sum(points_2d**2, axis=1)
    
    # 最小二乘解
    circle_params, _, _, _ = np.linalg.lstsq(A_circle, b_circle, rcond=None)
    cx_2d, cy_2d = circle_params[0], circle_params[1]
    
    # 確保半徑能包含所有點：取圓心到所有點的最大距離
    distances = np.sqrt((points_2d[:, 0] - cx_2d)**2 + (points_2d[:, 1] - cy_2d)**2)
    radius = np.max(distances)  # 最小外接圓半徑
    
    # 將圓心從 2D 座標轉回 3D，作為最終的 center3d
    circle_center_3d = temp_center + cx_2d * u_axis + cy_2d * v_axis
    center3d = circle_center_3d  # 使用圓心作為重心
    Xc, Yc, Zc = center3d

    # 投影回 2D 影像座標

    # 投影回 2D 影像座標
    if Zc <= 0:
        center2d = None
    else:
        uc = fx * Xc / Zc + ppx
        vc = fy * Yc / Zc + ppy
        center2d = np.array([uc, vc], dtype=np.float64)

    # === 7. 可視化：原始點雲、擬合平面、虛擬點雲、重心（圓心）、擬合圓 ===
    try:
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # 確保 pts 是 numpy 數組
        pts_array = np.asarray(pts)
        inlier_mask_array = np.asarray(inlier_mask)
        
        # 7.1 Draw original point cloud (points with depth) - Blue
        ax.scatter(pts_array[:, 0], pts_array[:, 1], pts_array[:, 2], 
                  c='blue', marker='.', s=1, alpha=0.3, label='Original Points')
        
        # 7.2 Draw RANSAC inliers (points on fitted plane) - Red
        inlier_pts = pts_array[inlier_mask_array]
        ax.scatter(inlier_pts[:, 0], inlier_pts[:, 1], inlier_pts[:, 2],
                  c='red', marker='o', s=5, alpha=0.6, label='Plane Inliers')
        
        # 7.3 Draw fitted plane mesh
        # Find point cloud range to draw the plane
        x_min, x_max = float(pts_array[:, 0].min()), float(pts_array[:, 0].max())
        y_min, y_max = float(pts_array[:, 1].min()), float(pts_array[:, 1].max())
        x_range = x_max - x_min
        y_range = y_max - y_min
        
        # Extend range by 20%
        x_min -= x_range * 0.2
        x_max += x_range * 0.2
        y_min -= y_range * 0.2
        y_max += y_range * 0.2
        
        xx, yy = np.meshgrid(np.linspace(x_min, x_max, 20),
                            np.linspace(y_min, y_max, 20))
        # Use plane equation: Z = aX + bY + c
        zz = float(a) * xx + float(b) * yy + float(c)
        # plot_surface doesn't support label, need Proxy Artist
        ax.plot_surface(xx, yy, zz, alpha=0.2, color='green')
        
        # 7.4 Draw virtual point cloud (projected fake points) - Yellow
        fake_points_array = np.asarray(fake_points)
        ax.scatter(fake_points_array[:, 0], fake_points_array[:, 1], fake_points_array[:, 2],
                  c='yellow', marker='^', s=3, alpha=0.5, label='Virtual Points')
        
        # 7.5 Draw centroid position - Magenta large point
        center3d_array = np.asarray(center3d)
        ax.scatter([float(center3d_array[0])], [float(center3d_array[1])], [float(center3d_array[2])], 
                  c='magenta', marker='*', s=200, 
                  edgecolors='black', linewidths=2, label='Centroid')
        
        # 7.6 Draw normal vector (from centroid)
        arrow_length = max(x_range, y_range) * 0.3
        normal3d_array = np.asarray(normal3d)
        ax.quiver(float(center3d_array[0]), float(center3d_array[1]), float(center3d_array[2]),
                 float(normal3d_array[0]), float(normal3d_array[1]), float(normal3d_array[2]),
                 length=arrow_length, color='cyan', 
                 arrow_length_ratio=0.3, linewidth=2, label='Normal Vector')
        
        # 7.7 Draw fitted circle on the plane
        if radius > 0:
            # Generate circle points in 2D plane coordinates (圓心現在在原點，因為 center3d 就是圓心)
            theta = np.linspace(0, 2 * np.pi, 100)
            circle_u = radius * np.cos(theta)
            circle_v = radius * np.sin(theta)
            
            # Convert circle points to 3D (以 center3d 為圓心)
            circle_3d = np.zeros((100, 3))
            for i in range(100):
                circle_3d[i] = center3d + circle_u[i] * u_axis + circle_v[i] * v_axis
            
            # Plot circle
            ax.plot(circle_3d[:, 0], circle_3d[:, 1], circle_3d[:, 2],
                   color='orange', linewidth=3, label='Fitted Circle')
            
            print(f"Fitted circle - Center: {center3d}, Radius: {radius:.4f} m")
        
        # Set labels and title
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title('Plane Fitting Visualization\nBlue: Original | Red: Inliers | Green: Plane | Yellow: Virtual | Magenta: Circle Center | Orange: Circle')
        
        # Create proxy artist for plot_surface to display in legend
        from matplotlib.patches import Patch
        legend_elements = [
            Patch(facecolor='blue', alpha=0.3, label='Original Points'),
            Patch(facecolor='red', alpha=0.6, label='Plane Inliers'),
            Patch(facecolor='green', alpha=0.2, label='Fitted Plane'),
            Patch(facecolor='yellow', alpha=0.5, label='Virtual Points'),
            Patch(facecolor='magenta', label='Circle Center'),
            Patch(facecolor='cyan', label='Normal Vector'),
            Patch(facecolor='orange', label='Fitted Circle')
        ]
        ax.legend(handles=legend_elements, loc='upper right')
        
        # Set equal scaling
        z_min, z_max = float(pts_array[:, 2].min()), float(pts_array[:, 2].max())
        max_range = max(x_range, y_range, z_max - z_min)
        mid_x = (x_max + x_min) / 2
        mid_y = (y_max + y_min) / 2
        mid_z = (z_max + z_min) / 2
        ax.set_xlim(mid_x - max_range/2, mid_x + max_range/2)
        ax.set_ylim(mid_y - max_range/2, mid_y + max_range/2)
        ax.set_zlim(mid_z - max_range/2, mid_z + max_range/2)
        
        plt.tight_layout()
        # plt.show()
        
    except Exception as e:
        import traceback
        print(f"可視化失敗: {e}")
        print(f"錯誤類型: {type(e).__name__}")
        print(f"完整錯誤追蹤:")
        traceback.print_exc()
        print(f"\n調試資訊:")
        try:
            print(f"  pts type: {type(pts)}, shape: {getattr(pts, 'shape', 'N/A')}")
        except:
            print(f"  pts: 無法取得資訊")
        try:
            print(f"  inlier_mask type: {type(inlier_mask)}, shape: {getattr(inlier_mask, 'shape', 'N/A')}")
        except:
            print(f"  inlier_mask: 無法取得資訊")
        try:
            print(f"  fake_points type: {type(fake_points)}, shape: {getattr(fake_points, 'shape', 'N/A')}")
        except:
            print(f"  fake_points: 無法取得資訊")
        try:
            print(f"  center3d type: {type(center3d)}, value: {center3d}")
        except:
            print(f"  center3d: 無法取得資訊")
        try:
            print(f"  normal3d type: {type(normal3d)}, value: {normal3d}")
        except:
            print(f"  normal3d: 無法取得資訊")
        try:
            print(f"  a type: {type(a)}, value: {a}")
        except:
            print(f"  a: 無法取得資訊")
        try:
            print(f"  b type: {type(b)}, value: {b}")
        except:
            print(f"  b: 無法取得資訊")
        try:
            print(f"  c type: {type(c)}, value: {c}")
        except:
            print(f"  c: 無法取得資訊")

    return mask_top, center3d, normal3d, center2d

def extract_hole_plane_with_viz(
    mask_original, mask_dilated, depth_u16, depth_scale, K,
    img_bgr=None, hole_id=None,
    dist_thresh=0.003, iters=2000, min_inliers=200,
    show_plot=True
):
    """
    專門用於插入洞的 RANSAC 平面擬合和可視化。
    
    1. 用膨脹後的 mask 進行 RANSAC 平面擬合
    2. 將原始 mask 投影到擬合平面，生成虛擬點雲
    3. 計算虛擬點雲質心作為最終的 3D 座標
    4. 可視化所有相關點雲和平面
    
    參數：
      mask_original: (H,W) uint8/bool，原始洞的遮罩
      mask_dilated : (H,W) uint8/bool，膨脹後的遮罩（用於 RANSAC）
      depth_u16    : (H,W) uint16，深度影像
      depth_scale  : float，深度轉換比例
      K            : dict，內參 {"fx","fy","ppx","ppy"}
      img_bgr      : (H,W,3) uint8，彩色影像（可選，用於可視化）
      hole_id      : int，洞的 ID（可選，用於標題）
      dist_thresh  : RANSAC 平面內點距離閾值（公尺）
      iters        : RANSAC 迭代次數
      min_inliers  : 最少內點數量
      show_plot    : bool，是否顯示 3D 點雲圖
    
    回傳：
      xyz_centroid : (3,) tuple，虛擬點雲質心 [X, Y, Z]，失敗則為 None
      normal3d     : (3,) np.array，平面法向量，失敗則為 None
      d_plane      : float，平面方程參數 d，失敗則為 None
      stats        : dict，統計信息
    """
    import numpy as np
    import cv2
    
    fx, fy = K["fx"], K["fy"]
    ppx, ppy = K["ppx"], K["ppy"]
    
    # === 1. 用膨脹 mask 建立點雲進行 RANSAC ===
    ys_dilated, xs_dilated = np.where(mask_dilated > 0)
    
    if len(xs_dilated) < min_inliers:
        print(f"[RANSAC] 膨脹 mask 像素數不足: {len(xs_dilated)} < {min_inliers}")
        return None, None, None, {}
    
    Z_dilated = depth_u16[ys_dilated, xs_dilated] * depth_scale
    valid_dilated = Z_dilated > 0
    
    if np.sum(valid_dilated) < min_inliers:
        print(f"[RANSAC] 有效深度點數不足: {np.sum(valid_dilated)} < {min_inliers}")
        return None, None, None, {}
    
    xs_d = xs_dilated[valid_dilated]
    ys_d = ys_dilated[valid_dilated]
    Z_d = Z_dilated[valid_dilated]
    
    X_d = (xs_d - ppx) * Z_d / fx
    Y_d = (ys_d - ppy) * Z_d / fy
    pts_dilated = np.stack([X_d, Y_d, Z_d], axis=1)  # (N, 3)
    
    # === 2. RANSAC 平面擬合 ===
    n_pts = pts_dilated.shape[0]
    best_inliers = None
    best_normal = None
    best_d = None
    max_inliers = 0
    
    for _ in range(iters):
        idx = np.random.choice(n_pts, 3, replace=False)
        p0, p1, p2 = pts_dilated[idx]
        v1 = p1 - p0
        v2 = p2 - p0
        
        normal = np.cross(v1, v2)
        norm_len = np.linalg.norm(normal)
        if norm_len < 1e-6:
            continue
        
        normal /= norm_len
        d = -np.dot(normal, p0)
        
        dist = np.abs(pts_dilated @ normal + d)
        inliers = dist < dist_thresh
        count = np.count_nonzero(inliers)
        
        if count > max_inliers:
            max_inliers = count
            best_inliers = inliers
            best_normal = normal.copy()
            best_d = float(d)
    
    if best_inliers is None or max_inliers < min_inliers:
        print(f"[RANSAC] 找不到足夠內點的平面: {max_inliers} < {min_inliers}")
        return None, None, None, {}
    
    normal3d = best_normal
    d_plane = best_d
    
    # 統一法向量朝上
    if normal3d[2] < 0:
        normal3d = -normal3d
        d_plane = -d_plane
    
    print(f"[RANSAC] 成功擬合平面，內點數: {max_inliers}/{n_pts}, 法向量: {normal3d}")
    
    # === 3. 將原始 mask 投影到平面，生成虛擬點雲 ===
    ys_orig, xs_orig = np.where(mask_original > 0)
    
    if len(xs_orig) == 0:
        print("[RANSAC] 原始 mask 無像素")
        return None, None, None, {}
    
    u_all = xs_orig.astype(np.float64)
    v_all = ys_orig.astype(np.float64)
    dirs = np.stack([
        (u_all - ppx) / fx,
        (v_all - ppy) / fy,
        np.ones_like(u_all)
    ], axis=1)
    
    denom = dirs @ normal3d
    valid_dir = np.abs(denom) > 1e-8
    
    if np.sum(valid_dir) == 0:
        print("[RANSAC] 射線與平面平行")
        return None, None, None, {}
    
    dirs_valid = dirs[valid_dir]
    denom_valid = denom[valid_dir]
    t = -d_plane / denom_valid
    
    positive = t > 0
    if np.sum(positive) == 0:
        print("[RANSAC] 無有效投影點")
        return None, None, None, {}
    
    dirs_final = dirs_valid[positive]
    t_final = t[positive]
    
    fake_points = dirs_final * t_final[:, None]  # 虛擬點雲
    xyz_centroid = tuple(fake_points.mean(axis=0))
    
    print(f"[RANSAC] 虛擬點雲數量: {len(fake_points)}, 質心: {xyz_centroid}")
    
    # === 4. 建立原始 mask 真實深度點雲（用於對比） ===
    Z_orig = depth_u16[ys_orig, xs_orig] * depth_scale
    valid_orig = Z_orig > 0
    
    pts_orig_real = None
    if np.sum(valid_orig) > 0:
        xs_o = xs_orig[valid_orig]
        ys_o = ys_orig[valid_orig]
        Z_o = Z_orig[valid_orig]
        X_o = (xs_o - ppx) * Z_o / fx
        Y_o = (ys_o - ppy) * Z_o / fy
        pts_orig_real = np.stack([X_o, Y_o, Z_o], axis=1)
    
    # === 5. 統計信息 ===
    stats = {
        "dilated_points": len(pts_dilated),
        "ransac_inliers": max_inliers,
        "original_pixels": len(xs_orig),
        "virtual_points": len(fake_points),
        "real_depth_points": len(pts_orig_real) if pts_orig_real is not None else 0,
    }
    
    # === 6. 可視化 ===
    if show_plot:
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        
        fig = plt.figure(figsize=(14, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # 1. 膨脹 mask 點雲（灰色半透明）
        ax.scatter(pts_dilated[:, 0], pts_dilated[:, 1], pts_dilated[:, 2],
                  c='gray', marker='.', s=2, alpha=0.3,
                  label=f'Dilated Mask ({len(pts_dilated)} pts, for RANSAC)')
        
        # 2. 原始 mask 真實深度點雲（藍色）
        if pts_orig_real is not None:
            ax.scatter(pts_orig_real[:, 0], pts_orig_real[:, 1], pts_orig_real[:, 2],
                      c='blue', marker='.', s=8, alpha=0.6,
                      label=f'Original Mask Real Depth ({len(pts_orig_real)} pts)')
        
        # 3. 虛擬點雲（綠色）
        ax.scatter(fake_points[:, 0], fake_points[:, 1], fake_points[:, 2],
                  c='green', marker='.', s=8, alpha=0.8,
                  label=f'Virtual Point Cloud ({len(fake_points)} pts, Projected)')
        
        # 4. 擬合平面（紅色半透明平面）
        x_range = fake_points[:, 0]
        y_range = fake_points[:, 1]
        x_min, x_max = x_range.min(), x_range.max()
        y_min, y_max = y_range.min(), y_range.max()
        
        xx, yy = np.meshgrid(
            np.linspace(x_min - 0.02, x_max + 0.02, 10),
            np.linspace(y_min - 0.02, y_max + 0.02, 10)
        )
        
        if abs(normal3d[2]) > 1e-6:
            zz = -(normal3d[0] * xx + normal3d[1] * yy + d_plane) / normal3d[2]
            ax.plot_surface(xx, yy, zz, alpha=0.2, color='red')
        
        # 5. 質心（紅色大點）
        ax.scatter([xyz_centroid[0]], [xyz_centroid[1]], [xyz_centroid[2]],
                  c='red', marker='o', s=200, edgecolors='black', linewidths=2,
                  label=f'Centroid ({xyz_centroid[0]:.4f}, {xyz_centroid[1]:.4f}, {xyz_centroid[2]:.4f})')
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        
        title = f'Hole RANSAC Plane Analysis'
        if hole_id is not None:
            title += f' (ID={hole_id})'
        title += '\n(Red surface = RANSAC fitted plane)'
        ax.set_title(title)
        
        ax.legend(loc='upper left', fontsize=9)
        ax.set_box_aspect([1, 1, 1])
        plt.tight_layout()
        plt.show(block=False)
        plt.pause(0.1)
    
    # === 7. 可視化 2D 遮罩對比（可選）===
    if img_bgr is not None:
        overlay = img_bgr.copy()
        
        # 原始 mask（綠色半透明）
        green_layer = np.zeros_like(overlay)
        green_layer[mask_original > 0] = [0, 255, 0]
        overlay = cv2.addWeighted(overlay, 0.7, green_layer, 0.3, 0)
        
        # 膨脹 mask 輪廓（黃色）
        contours, _ = cv2.findContours(
            (mask_dilated > 0).astype(np.uint8),
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )
        cv2.drawContours(overlay, contours, -1, (0, 255, 255), 2)
        
        # 標記質心投影點
        if xyz_centroid[2] > 0:
            u = int(fx * xyz_centroid[0] / xyz_centroid[2] + ppx)
            v = int(fy * xyz_centroid[1] / xyz_centroid[2] + ppy)
            cv2.circle(overlay, (u, v), 5, (0, 0, 255), -1)
            cv2.putText(overlay, f"Centroid", (u + 8, v - 8),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        cv2.imshow(f"Hole Mask Analysis (ID={hole_id})", overlay)
        cv2.waitKey(1)
    
    return xyz_centroid, normal3d, d_plane, stats
