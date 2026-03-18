import cv2
import numpy as np


def build_edge_map(roi_bgr, canny1=50, canny2=150):
    gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    gray_eq = clahe.apply(blur)
    edges = cv2.Canny(gray_eq, canny1, canny2)

    # 補小斷裂，但不要太大，避免上下線黏在一起
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 3))
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

    return gray, edges


def group_y_positions(ys, y_gap=4):
    if len(ys) == 0:
        return []

    ys = sorted(ys.tolist())
    groups = [[ys[0]]]

    for y in ys[1:]:
        if y - groups[-1][-1] <= y_gap:
            groups[-1].append(y)
        else:
            groups.append([y])

    return groups


def extract_curve_candidates_from_edges(
    edges,
    min_points=30,
    y_gap=4,
    match_tol=6,
    max_gap_x=12
):
    """
    從 edge 圖抽多條大致水平的曲線。
    與舊版不同：
    - 允許 track 中間斷掉幾個 x (max_gap_x)
    - 遮擋時不會立刻放棄該 track
    """
    h, w = edges.shape
    tracks = []

    for x in range(w):
        ys = np.where(edges[:, x] > 0)[0]
        groups = group_y_positions(ys, y_gap=y_gap)
        centers = [int(round(np.mean(g))) for g in groups]

        assigned = [False] * len(centers)

        # 先接到既有 tracks
        for track in tracks:
            last_x, last_y = track[-1]

            # 允許中間有一段 gap
            if x - last_x > max_gap_x:
                continue

            best_idx = -1
            best_dist = 1e9

            for i, cy in enumerate(centers):
                if assigned[i]:
                    continue
                d = abs(cy - last_y)
                if d < best_dist and d <= match_tol:
                    best_dist = d
                    best_idx = i

            if best_idx >= 0:
                track.append((x, centers[best_idx]))
                assigned[best_idx] = True

        # 沒分配到的點開新 track
        for i, cy in enumerate(centers):
            if not assigned[i]:
                tracks.append([(x, cy)])

    tracks = [t for t in tracks if len(t) >= min_points]
    return tracks


def merge_broken_tracks(
    tracks,
    max_join_gap=40,
    max_join_y_diff=12,
    poly_deg=2
):
    """
    把被遮擋切成左右兩段的同一條曲線再合併。
    依據：
    - x 上接近
    - 端點 y 接近
    - 用左段趨勢預測到右段起點不會差太多
    """
    if len(tracks) <= 1:
        return tracks

    tracks = [sorted(t, key=lambda p: p[0]) for t in tracks]
    tracks.sort(key=lambda t: t[0][0])

    merged = True
    while merged:
        merged = False
        new_tracks = []
        used = [False] * len(tracks)

        for i in range(len(tracks)):
            if used[i]:
                continue

            base = tracks[i]

            for j in range(i + 1, len(tracks)):
                if used[j]:
                    continue

                a = base
                b = tracks[j]

                a_end_x, a_end_y = a[-1]
                b_start_x, b_start_y = b[0]

                gap_x = b_start_x - a_end_x
                if gap_x < 1 or gap_x > max_join_gap:
                    continue

                # 端點高度差先做粗篩
                if abs(a_end_y - b_start_y) > max_join_y_diff:
                    continue

                # 用左段做簡單二次/一次擬合，預測右段起點 y
                a_arr = np.array(a, dtype=np.float32)
                xs = a_arr[:, 0]
                ys = a_arr[:, 1]

                deg = min(poly_deg, len(a) - 1)
                if deg < 1:
                    continue

                coeff = np.polyfit(xs, ys, deg=deg)
                y_pred = np.polyval(coeff, b_start_x)

                if abs(y_pred - b_start_y) <= max_join_y_diff:
                    base = a + b
                    used[j] = True
                    merged = True

            used[i] = True
            new_tracks.append(base)

        tracks = new_tracks

    return tracks


def smooth_track(track, window=9):
    pts = np.array(track, dtype=np.float32)
    xs = pts[:, 0]
    ys = pts[:, 1]

    if len(ys) < window:
        return pts.astype(np.int32)

    pad = window // 2
    ys_pad = np.pad(ys, (pad, pad), mode='edge')
    kernel = np.ones(window, dtype=np.float32) / window
    ys_smooth = np.convolve(ys_pad, kernel, mode='valid')

    out = np.stack([xs, ys_smooth], axis=1)
    return np.round(out).astype(np.int32)


def fit_and_fill_curve(track, x_min=None, x_max=None, poly_deg=2):
    """
    對 track 擬合曲線，並補齊中間被遮擋的區段。
    """
    pts = np.array(track, dtype=np.float32)
    xs = pts[:, 0]
    ys = pts[:, 1]

    if len(xs) < 2:
        return pts.astype(np.int32)

    deg = min(poly_deg, len(xs) - 1)
    coeff = np.polyfit(xs, ys, deg=deg)

    if x_min is None:
        x_min = int(xs.min())
    if x_max is None:
        x_max = int(xs.max())

    x_full = np.arange(x_min, x_max + 1, dtype=np.float32)
    y_full = np.polyval(coeff, x_full)

    curve = np.stack([x_full, y_full], axis=1)
    return np.round(curve).astype(np.int32)


def detect_horizontal_curves_in_roi(
    img,
    roi,
    canny1=50,
    canny2=150,
    min_points=25,
    y_gap=3,
    match_tol=5,
    max_gap_x=12,
    merge_tracks=True,
    max_join_gap=40,
    max_join_y_diff=12,
    smooth_window=7,
    fit_poly=True,
    poly_deg=2,
    debug=False
):
    if type(img) == str:
        img = cv2.imread(image_path)
    if img is None:
        raise ValueError(f"Cannot read image: {image_path}")

    x0, y0, w, h = roi
    roi_img = img[y0:y0+h, x0:x0+w].copy()

    gray, edges = build_edge_map(roi_img, canny1=canny1, canny2=canny2)

    tracks = extract_curve_candidates_from_edges(
        edges,
        min_points=min_points,
        y_gap=y_gap,
        match_tol=match_tol,
        max_gap_x=max_gap_x
    )

    if merge_tracks:
        tracks = merge_broken_tracks(
            tracks,
            max_join_gap=max_join_gap,
            max_join_y_diff=max_join_y_diff,
            poly_deg=poly_deg
        )

    result = img.copy()
    cv2.rectangle(result, (x0, y0), (x0 + w, y0 + h), (0, 255, 255), 1)

    curves_info = []

    for i, track in enumerate(tracks):
        track = sorted(track, key=lambda p: p[0])

        # 先平滑原始點
        smooth_pts = smooth_track(track, window=smooth_window)

        # 再擬合補齊遮擋區
        if fit_poly and len(smooth_pts) >= 4:
            draw_pts = fit_and_fill_curve(
                smooth_pts,
                x_min=int(smooth_pts[:, 0].min()),
                x_max=int(smooth_pts[:, 0].max()),
                poly_deg=poly_deg
            )
        else:
            draw_pts = smooth_pts

        # 轉回 global 座標
        draw_pts_global = draw_pts.copy()
        draw_pts_global[:, 0] += x0
        draw_pts_global[:, 1] += y0

        for j in range(len(draw_pts_global) - 1):
            p1 = tuple(draw_pts_global[j])
            p2 = tuple(draw_pts_global[j + 1])
            cv2.line(result, p1, p2, (0, 0, 255), 2)

        sx, sy = draw_pts_global[0]
        cv2.putText(
            result,
            f"{i}",
            (int(sx), int(max(sy - 5, 0))),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 0),
            1,
            cv2.LINE_AA
        )

        curves_info.append({
            "id": i,
            "num_points": len(track),
            "x_start": int(draw_pts_global[0, 0]),
            "x_end": int(draw_pts_global[-1, 0]),
            "y_mean": float(np.mean(draw_pts_global[:, 1])),
            "points": draw_pts_global
        })

    curves_info.sort(key=lambda c: c["y_mean"])

    if debug:
        cv2.imshow("gray", gray)
        cv2.imshow("edges", edges)
        cv2.imshow("result", result)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return result, curves_info


def compute_top2_curve_distance(curves_info):
    """
    計算最上面兩條曲線之間的距離統計。

    Parameters
    ----------
    curves_info : list
        detect_horizontal_curves_in_roi() 回傳的 curves_info

    Returns
    -------
    dict or None
        {
            "max": 最大距離,
            "median": 中位數距離,
            "mean": 平均距離,
            "num_samples": 計算點數
        }
    """

    if len(curves_info) < 2:
        return None

    # 確保按照高度排序（最上面在前）
    curves = sorted(curves_info, key=lambda c: c["y_mean"])

    curve1 = curves[0]["points"]
    curve2 = curves[1]["points"]

    # 轉成 dict: x -> y
    map1 = {int(p[0]): int(p[1]) for p in curve1}
    map2 = {int(p[0]): int(p[1]) for p in curve2}

    # 找共同 x
    common_x = sorted(set(map1.keys()) & set(map2.keys()))

    if len(common_x) == 0:
        return None

    distances = []

    for x in common_x:
        d = abs(map2[x] - map1[x])
        distances.append(d)

    distances = np.array(distances)

    return {
        "max": float(np.max(distances)),
        "median": float(np.median(distances)),
        "mean": float(np.mean(distances)),
        "num_samples": len(distances)
    }

if __name__ == "__main__":
    image_path = "ros2_ws/src/ipc_test/image.png"
    roi = (340, 370, 230, 100)

    result, curves = detect_horizontal_curves_in_roi(
        image_path=image_path,
        roi=roi,
        canny1=50,
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

    print("Detected curves:")
    for c in curves:
        print(
            f"[{c['id']}] "
            f"y_mean={c['y_mean']:.1f}, "
            f"x=({c['x_start']}, {c['x_end']}), "
            f"points={len(c['points'])}"
        )

    cv2.imwrite("curve_result_occlusion.png", result)