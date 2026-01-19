def extract_top_plane(mask_obj, depth_u16, depth_scale, K,
                      dist_thresh=0.003, iters=2000, min_inliers=200):
    """
    用深度圖與 RANSAC 從物件遮罩中擬合出最主要平面（通常是圓柱頂面）。
    回傳：
      mask_top (H,W,uint8)
      center3d (np.array[3])
      normal3d (np.array[3])
    """
    import numpy as np

    fx, fy, ppx, ppy = K["fx"], K["fy"], K["ppx"], K["ppy"]

    ys, xs = np.where(mask_obj > 0)
    if len(xs) < min_inliers:
        return None, None, None

    Z = depth_u16[ys, xs] * depth_scale
    valid = Z > 0
    xs, ys, Z = xs[valid], ys[valid], Z[valid]
    if len(xs) < min_inliers:
        return None, None, None

    X = (xs - ppx) * Z / fx
    Y = (ys - ppy) * Z / fy
    pts = np.stack([X, Y, Z], axis=1)

    # === 手寫版 RANSAC 平面擬合 ===
    n_pts = pts.shape[0]
    best_inliers = None
    best_normal = None
    best_d = None
    max_inliers = 0

    for _ in range(iters):
        sample = pts[np.random.choice(n_pts, 3, replace=False)]
        v1, v2 = sample[1] - sample[0], sample[2] - sample[0]
        normal = np.cross(v1, v2)
        if np.linalg.norm(normal) < 1e-6:
            continue
        normal /= np.linalg.norm(normal)
        d = -np.dot(normal, sample[0])
        dist = np.abs(pts @ normal + d)
        inliers = dist < dist_thresh
        count = np.count_nonzero(inliers)
        if count > max_inliers:
            max_inliers = count
            best_inliers = inliers
            best_normal = normal
            best_d = d

    if best_inliers is None or max_inliers < min_inliers:
        return None, None, None

    plane_points = pts[best_inliers]
    center3d = plane_points.mean(axis=0)
    normal3d = best_normal
    if normal3d[2] < 0:  # 讓法向朝上
        normal3d = -normal3d

    mask_top = np.zeros_like(mask_obj, dtype=np.uint8)
    mask_top[ys[best_inliers], xs[best_inliers]] = 1

    return mask_top, center3d, normal3d