import cv2
import numpy as np
from typing import Any, Dict, List, Optional, Tuple , Sequence
import base64, json, os
from openai import OpenAI
import re 
import open3d as o3d


def _img_bgr_to_png_bytes(img_bgr: np.ndarray) -> bytes:
    """把 BGR 圖存成 PNG bytes。"""
    ok, buf = cv2.imencode(".png", img_bgr)
    if not ok:
        raise RuntimeError("Failed to encode image as PNG")
    return buf.tobytes()

def _png_bytes_to_data_uri(png_bytes: bytes) -> str:
    """把 PNG bytes 存成 data URI。"""
    b64 = base64.b64encode(png_bytes).decode("utf-8")
    return f"data:image/png;base64,{b64}"

def generate_masks(field_img_color: np.ndarray) -> List[Dict[str, Any]]:
    """
    generate_masks 的 Docstring
    
    :param field_img_color: 說明
    :type field_img_color: np.ndarray
    :return: 說明
    : field_img_mask_center, field_img_colored_mask, field_masks
    """
    # ===== SAM2 config =====
    POINTS_PER_SIDE        = 32
    PRED_IOU_THRESH        = 0.7
    STABILITY_SCORE_THRESH = 0.92
    MIN_MASK_REGION_AREA   = 100.0

    from sam2_client_1141115v2 import SAM2AutomaticMaskGenerator
    print(f"[Info] SAM2 device: {'ICALAB_AI5'}")

    mask_generator = SAM2AutomaticMaskGenerator(
        model=None,
        points_per_side=POINTS_PER_SIDE,
        pred_iou_thresh=PRED_IOU_THRESH,
        stability_score_thresh=STABILITY_SCORE_THRESH,
        min_mask_region_area=MIN_MASK_REGION_AREA,
    )

    field_rgb = cv2.cvtColor(field_img_color, cv2.COLOR_BGR2RGB)
    field_masks = mask_generator.generate(field_rgb)

    # 算中心點（用來畫 ID）
    def _mask_center(mask_item):
        m = mask_item["segmentation"].astype(np.uint8)
        if m.ndim == 3:
            m = m[..., 0]
        if not m.any():
            return None
        binmask = (m > 0).astype(np.uint8)
        M = cv2.moments(binmask)
        if M["m00"] != 0:
            return (M["m10"] / M["m00"], M["m01"] / M["m00"])
        ys, xs = np.where(binmask > 0)
        return (float(xs.mean()), float(ys.mean())) if len(xs) else None
    
    def _apply_colored_masks(img_bgr: np.ndarray, masks: list, alpha: float = 0.2, seed: int = 84) -> np.ndarray:
        """對圖片套用彩色遮罩。"""
        h, w = img_bgr.shape[:2]
        rng = np.random.default_rng(seed)
        
        for idx, item in enumerate(masks):
            m = item["segmentation"].astype(np.uint8)
            if m.ndim == 3:
                m = m[..., 0]
            if m.sum() == 0:
                continue
            
            color = rng.integers(25, 255, size=3, dtype=np.uint8)
            color_mask = np.zeros((h, w, 3), dtype=np.uint8)
            color_mask[m > 0] = color
            img_bgr = cv2.addWeighted(img_bgr, 1.0, color_mask, alpha, 0)
            cxcy = _mask_center(item)
            if cxcy is None:
                continue
            cx, cy = cxcy
            color = color.tolist()
            cv2.circle(img_bgr, (int(cx), int(cy)), 4, (0, 0, 0), -1)
            cv2.circle(img_bgr, (int(cx), int(cy)), 3, (color[0], color[1], color[2]), -1)
            cv2.putText(img_bgr, str(idx), (int(cx) + 6, int(cy) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(img_bgr, str(idx), (int(cx) + 6, int(cy) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (color[0], color[1], color[2]), 1, cv2.LINE_AA)

        return img_bgr
    
    def _apply_center_text_masks(img_bgr: np.ndarray, masks: list) -> np.ndarray:
        for idx, item in enumerate(masks):
            cxcy = _mask_center(item)
            if cxcy is None:
                continue
            cx, cy = cxcy
            cv2.circle(img_bgr, (int(cx), int(cy)), 4, (0, 0, 0), -1)
            cv2.circle(img_bgr, (int(cx), int(cy)), 3, (255, 255, 255), -1)
            cv2.putText(img_bgr, str(idx), (int(cx) + 6, int(cy) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(img_bgr, str(idx), (int(cx) + 6, int(cy) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        return img_bgr
    
    field_img_mask_center = _apply_center_text_masks(field_rgb.copy(), field_masks)
    field_img_colored_mask = _apply_colored_masks(field_rgb.copy(), field_masks)

    return field_img_mask_center, field_img_colored_mask, field_masks

def ask_vlm_field_platform_mask_id(
    field_img_with_ids_bgr: np.ndarray,
    valid_ids: Optional[Sequence[int]] = None,
    model: str = "gpt-4.1",
    temperature: float = 0.0,
) -> Optional[int]:
    """
    輸入：一張 FIELD 影像，影像中每個 segmented region 都有「colored mask」與「數字 mask id」
    任務：找出「帶洞的平面（platform top surface with circular holes）」那一塊 region 的 mask id。

    回傳：
    target_mask_id (int) 或 None
    """
    api_key = os.environ.get("OPENAI_API_KEY")
    if not api_key:
        raise RuntimeError("OPENAI_API_KEY not set")
    client = OpenAI(api_key=api_key)

    field_uri = _png_bytes_to_data_uri(_img_bgr_to_png_bytes(field_img_with_ids_bgr))

    # 把 valid_ids 放進 prompt，能顯著降低亂答（如果你有的話）
    valid_ids_str = ""
    if valid_ids is not None:
        valid_ids_str = f"The valid mask IDs are: {list(map(int, valid_ids))}.\n"

    system_prompt = (
        "You are a careful vision assistant.\n"
        "You will receive ONE image (FIELD).\n"
        "The image contains multiple colored segmentation regions, each labeled with a numeric mask ID.\n"
        f"{valid_ids_str}"
        "\n"
        "TASK:\n"
        "- Identify the PLATFORM TOP SURFACE region (a flat planar surface) that CONTAINS MULTIPLE CIRCULAR HOLES.\n"
        "- Return the numeric mask ID of that single planar region (NOT the hole IDs, not screws, not background).\n"
        "- If multiple planar regions exist, choose the one that clearly has several circular holes on it.\n"
        "\n"
        "STRICT OUTPUT FORMAT (JSON ONLY):\n"
        "{\"target_mask_id\": <int>}\n"
        "Return STRICT JSON ONLY. No extra words."
    )

    user_prompt = "Return the mask ID of the planar platform surface that contains the circular holes."
    print("[VLM] asking")
    resp = client.chat.completions.create(
        model=model,
        temperature=temperature,
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": [
                {"type": "text", "text": user_prompt},
                {"type": "image_url", "image_url": {"url": field_uri}},
            ]},
        ],
    )

    text = resp.choices[0].message.content.strip()

    # robust JSON parse
    try:
        data = json.loads(text)
    except Exception:
        m = re.search(r"\{.*\}", text, flags=re.S)
        if not m:
            raise ValueError(f"VLM did not return valid JSON: {text}")
        data = json.loads(m.group(0))

    val = data.get("target_mask_id", None)
    if val is None:
        return None

    try:
        val = int(val)
    except Exception:
        raise ValueError(f"target_mask_id is not an integer: {val}")

    if valid_ids is not None and val not in set(map(int, valid_ids)):
        print(f"[VLM warn] target_mask_id {val} not in valid list {list(map(int, valid_ids))}")
        return None

    return val

def get_platform_mask(
    processed_field_img: np.ndarray,
    field_masks: List[Dict[str, Any]],
) -> Optional[np.ndarray]:
    target_id: Optional[int] = None
    target_mask: Optional[np.ndarray] = None

    target_id = ask_vlm_field_platform_mask_id(processed_field_img, model="gpt-4.1", temperature=0.0)
    # 依 target_id 取回對應 segmentation mask
    if target_id is not None and 0 <= target_id < len(field_masks):
        m = field_masks[target_id]["segmentation"]
        m = m.astype(np.uint8)
        if m.ndim == 3:
            m = m[..., 0]
        target_mask = (m > 0).astype(np.uint8) * 255
        print("[Debug] target_id =", target_id)
    else:
        if target_id is not None:
            print(f"[Warn] target_id={target_id} out of range (0~{len(field_masks)-1})")
    return target_mask, target_id

def get_cylinder_mask(
    demo_img_color_list: List[np.ndarray],
    processed_field_img: np.ndarray,
    field_masks: List[Dict[str, Any]],
) -> Optional[np.ndarray]:
    """
    輸入:
      1) demo BGR 圖 list（numpy ndarray, uint8）
      2) field BGR 圖（numpy ndarray, uint8）
      3) processed field BGR 圖（numpy ndarray, uint8）
      4) field segmentation masks（list of dict）
    回傳：
      target_mask (H,W) -> uint8 binary mask，前景=255, 背景=0；或 None
    """
    target_id: Optional[int] = None
    target_mask: Optional[np.ndarray] = None

    try:
        # 第一次：讀出所有 cylinder IDs（你原本的函式）
        cylinder_ids = ask_vlm_field_cylinder_ids(processed_field_img, model="gpt-4.1", temperature=0.0)
        print("[Debug] All cylinder_ids(B) =", cylinder_ids)
        # 第二次：從 DEMO 圖判斷要夾哪一個
        target_cyl_id = ask_vlm_field_target_cylinder_from_demo(
            demo_img_color_list, processed_field_img, cylinder_ids, model="gpt-4.1", temperature=0.0
        )
        print("[Debug] target_cylinder_id =", target_cyl_id)
        if target_cyl_id is not None:
            target_id = int(target_cyl_id)
    except Exception as e:
        print("[VLM warn][grasp] cylinder-id reading failed:", e)

    # 依 target_id 取回對應 segmentation mask
    if target_id is not None and 0 <= target_id < len(field_masks):
        m = field_masks[target_id]["segmentation"]
        m = m.astype(np.uint8)
        if m.ndim == 3:
            m = m[..., 0]
        target_mask = (m > 0).astype(np.uint8) * 255
    else:
        if target_id is not None:
            print(f"[Warn] target_id={target_id} out of range (0~{len(field_masks)-1})")

    return target_mask, target_id


def ask_vlm_field_cylinder_ids(
    field_img_with_ids_bgr: np.ndarray,
    model: str = "gpt-4.1",
    temperature: float = 0.0,
) -> list:
    """問 Image B：讀取料架上所有圓柱的 ID，回傳 list[int]。"""
    api_key = os.environ.get("OPENAI_API_KEY")
    if not api_key:
        raise RuntimeError("OPENAI_API_KEY not set")
    client = OpenAI(api_key=api_key)

    field_uri = _png_bytes_to_data_uri(_img_bgr_to_png_bytes(field_img_with_ids_bgr))

    system_prompt = (
        "You are a careful vision assistant.\n"
        "Image B (FIELD) shows one or multiple YELLOW or ORANGE CYLINDERS located at the CENTER of the image.\n"
        "Each cylinder has a numeric ID printed directly on its surface.\n"
        "Cylinders are placed directly on the table.\n"
        "\n"
        "TASK:\n"
        "- Focus ONLY on the cylinder(s) that occupy the CENTER region of the image.\n"
        "- Read ALL visible numeric IDs printed on the cylinder surfaces.\n"
        "- Ignore any numbers or text in the background (e.g., walls, tables, or other tools).\n"
        "- Do NOT include any digits from reflections, shadows, or noise.\n"
        "\n"
        "STRICT OUTPUT FORMAT:\n"
        "{\"cylinder_ids\": [<int>, <int>, ...]}\n"
        "Return STRICT JSON ONLY. No extra words."
    )

    user_prompt = (
        "List all visible cylinder IDs printed on the yellow or orange cylinder(s) located at the center of Image B (FIELD)."
    )

    resp = client.chat.completions.create(
        model=model,
        temperature=temperature,
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": [
                {"type": "text", "text": user_prompt},
                {"type": "image_url", "image_url": {"url": field_uri}},
            ]}
        ],
    )

    text = resp.choices[0].message.content.strip()
    try:
        data = json.loads(text)
    except Exception:
        m = re.search(r"\{.*\}", text, flags=re.S)
        if not m:
            raise ValueError(f"VLM did not return valid JSON: {text}")
        data = json.loads(m.group(0))

    ids = data.get("cylinder_ids", [])
    ids = [int(x) for x in ids if isinstance(x, (int, float))]
    print("[Debug] cylinder_ids =", ids)
    return ids

def ask_vlm_field_target_cylinder_from_demo(
    demo_img_bgr_list: List[np.ndarray],
    field_img_with_ids_bgr: np.ndarray,
    cylinder_ids,
    model: str = "gpt-4.1",
    temperature: float = 0.0,
) -> Optional[int]:
    """
    用多張 DEMO + FIELD 影像，判斷 DEMO 圖中要夾的那根圓柱，
    並從 cylinder_ids 中挑出對應的 ID。
    """
    api_key = os.environ.get("OPENAI_API_KEY")
    if not api_key:
        raise RuntimeError("OPENAI_API_KEY not set")
    client = OpenAI(api_key=api_key)

    demo_uris = [
        _png_bytes_to_data_uri(_img_bgr_to_png_bytes(img))
        for img in demo_img_bgr_list
    ]
    field_uri = _png_bytes_to_data_uri(_img_bgr_to_png_bytes(field_img_with_ids_bgr))

    system_prompt = (
        "You are a careful vision assistant.\n"
        "Image A (DEMO): You will receive multiple DEMO images (time sequence) showing a gripper moving toward a specific yellow/orange cylinder.\n"
        "Image B (FIELD): You will also receive one FIELD image with all cylinders labeled by numeric IDs.\n"
        f"The valid cylinder IDs are: {cylinder_ids}.\n"
        "All cylinders are placed directly on a flat table.\n"
        "\n"
        "TASK:\n"
        "- Focus only on the cylinder that the gripper is about to grasp in sequence Image A.\n"
        "- Compare its relative position and color/shape with the labeled cylinders in Image B.\n"
        "- Find the matching cylinder ID from Image B.\n"
        "- If uncertain, select the cylinder whose relative position and appearance most closely match the one in Image A, "
        "or the one located near the center of the image.\n"
        "\n"
        "STRICT OUTPUT FORMAT:\n"
        "{\"target_cylinder_id\": <int>}\n"
        "Return STRICT JSON ONLY. No extra words."
    )

    user_prompt = (
        "These DEMO images show a gripper approaching a cylinder. "
        "Use all frames together to infer which cylinder will be grasped, "
        "then match it to the labeled FIELD image and return its ID."
    )

    message_content = [{"type": "text", "text": user_prompt}]
    for uri in demo_uris:
        message_content.append({"type": "image_url", "image_url": {"url": uri}})
    message_content.append({"type": "image_url", "image_url": {"url": field_uri}})

    resp = client.chat.completions.create(
        model=model,
        temperature=temperature,
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": message_content},
        ],
    )

    text = resp.choices[0].message.content.strip()
    try:
        data = json.loads(text)
    except Exception:
        m = re.search(r"\{.*\}", text, flags=re.S)
        if not m:
            raise ValueError(f"VLM did not return valid JSON: {text}")
        data = json.loads(m.group(0))

    val = data.get("target_cylinder_id", None)
    if val is None:
        return None

    try:
        val = int(val)
        if val not in cylinder_ids:
            print(f"[VLM warn] target_cylinder_id {val} not in valid list {cylinder_ids}")
            return None
        return val
    except Exception:
        raise ValueError(f"target_cylinder_id is not an integer: {val}")
    
def masked_pointcloud_from_o3d(
    pcd: o3d.geometry.PointCloud,
    mask: np.ndarray,
    intr: Dict[str, Any],
    *,
    assume_organized: Optional[bool] = None,
    keep_colors: bool = True,
) -> o3d.geometry.PointCloud:
    """
    回傳 mask 內對應的點雲（Open3D）。

    Args:
        pcd: Open3D PointCloud（通常來自 RealSense PointCloud2 轉換）
        mask: (H,W) mask，bool 或 uint8(0/255/1) 皆可
        intr: 相機內參 dict，至少包含 fx, fy, ppx, ppy
              例: {"fx":..., "fy":..., "ppx":..., "ppy":...}
        assume_organized:
            - True: pcd 點順序對應影像 (row-major HxW)
            - False: pcd 是 unordered（RealSense 常見 height=1），用投影回像素篩選
            - None: 自動判斷（若點數==H*W 則視為 organized）
        keep_colors: 若原本 pcd 有 colors，是否一起帶回

    Returns:
        Open3D PointCloud：只包含 mask 內的點（以及顏色）
    """
    if mask.ndim != 2:
        raise ValueError("mask must be 2D (H,W)")

    H, W = mask.shape
    mask_bool = mask.astype(bool)

    fx = float(intr["fx"]); fy = float(intr["fy"])
    cx = float(intr["ppx"]); cy = float(intr["ppy"])

    pts = np.asarray(pcd.points)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError("pcd.points must be Nx3")

    N = pts.shape[0]

    # 自動判斷 organized：點數剛好等於 H*W
    if assume_organized is None:
        assume_organized = (N == H * W)

    if assume_organized:
        print("[Debug] assume_organized")
        # 直接把點 reshape 成 (H,W,3)，用 mask 索引
        pts_img = pts.reshape(H, W, 3)
        sel_pts = pts_img[mask_bool]

        out = o3d.geometry.PointCloud()
        out.points = o3d.utility.Vector3dVector(sel_pts)

        if keep_colors and pcd.has_colors():
            cols = np.asarray(pcd.colors).reshape(H, W, 3)
            sel_cols = cols[mask_bool]
            out.colors = o3d.utility.Vector3dVector(sel_cols)

        return out
    print("[Debug] not assume_organized")
    # unordered：把每個 3D 點投影回像素 (u,v)，用 mask 篩選
    X = pts[:, 0]; Y = pts[:, 1]; Z = pts[:, 2]

    # 過濾無效深度（避免除以0或投影爆掉）
    valid = np.isfinite(X) & np.isfinite(Y) & np.isfinite(Z) & (Z > 0)
    if not np.any(valid):
        return o3d.geometry.PointCloud()

    Xv = X[valid]; Yv = Y[valid]; Zv = Z[valid]

    u = np.round(fx * (Xv / Zv) + cx).astype(np.int32)
    v = np.round(fy * (Yv / Zv) + cy).astype(np.int32)

    in_img = (u >= 0) & (u < W) & (v >= 0) & (v < H)
    if not np.any(in_img):
        return o3d.geometry.PointCloud()

    u = u[in_img]
    v = v[in_img]

    valid_idx = np.nonzero(valid)[0][in_img]  # 回到原 N 的 index

    # 用 mask 篩掉不在 mask 內的點
    in_mask = mask_bool[v, u]
    sel_idx = valid_idx[in_mask]

    out = o3d.geometry.PointCloud()
    out.points = o3d.utility.Vector3dVector(pts[sel_idx])

    if keep_colors and pcd.has_colors():
        cols = np.asarray(pcd.colors)
        out.colors = o3d.utility.Vector3dVector(cols[sel_idx])

    return out

def visualize_mask(img_bgr: np.ndarray, mask: np.ndarray, title: str = "Mask") -> None:
    """可視化單個遮罩。"""
    # 創建遮罩疊加圖
    overlay = img_bgr.copy()
    mask_binary = (mask > 0).astype(np.uint8)
    
    # 綠色遮罩
    overlay[mask_binary > 0] = [0, 255, 0]
    result = cv2.addWeighted(img_bgr, 0.7, overlay, 0.3, 0)
    
    # 畫輪廓
    contours, _ = cv2.findContours(mask_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(result, contours, -1, (0, 255, 255), 2)
    
    cv2.imshow(title, result)
