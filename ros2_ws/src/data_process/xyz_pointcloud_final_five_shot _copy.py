# xyz_pointcloud_no-rack_insert-mask_yellow-or-orange_quadrant.py
import numpy as np
from typing import Any, Dict, Optional, Tuple, List
import re
import cv2
import torch
import base64, json, os
from openai import OpenAI
from ransac_plane2 import extract_top_plane
'''
11/7 Use five demo images as reference

'''



'''
1.get_xy_xyz_for_grasp
輸入:demo BGR, field BGR, field depth, field intrinsics
demo_img -> processed_demo_img
field_img + 中心點 + ID -> processed_field_img + field_img_meta(JSON)
兩階段 VLM:
1) ask_vlm_field_cylinder_ids:讀 processed_field_img 上的黃色或橘色圓柱(無料架) ID 集合 cylinder_ids 
2) ask_vlm_field_target_cylinder_from_demo:讀 processed_field_img 和 demo_img ，限制集合 (cylinder_ids) 內取得 target_id (processed_demo_img 中夾取的黃色或橘色圓柱)
pointcloud(RANSAC)
回傳 3D座標

2.get_xy_xyz_for_insert
輸入:demo BGR list, field BGR, field depth, field intrinsics
demo_img -> processed_demo_img
field_img 疊遮罩 + 中心點 + ID -> processed_field_img + field_img_meta(JSON)
兩階段 VLM:
1) ask_vlm_demo_quadrant:讀 list demo_img 和 processed_demo_img 上插入洞的象限  
2) ask_vlm_field_quadrant_to_id_map:取得 target_id (processed_demo_img 中插入的洞)

回傳 3D座標
'''

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

def ask_vlm_demo_quadrant(
    demo_img_bgr_list: List[np.ndarray],
    demo_masked_img_bgr_list: List[np.ndarray],
    model: str = "gpt-4.1",
    temperature: float = 0.0,
) -> str:
    """問 Image A/B：圓柱插在平台上表面哪個象限？回傳 'top_left'/'top_right'/'bottom_left'/'bottom_right'。"""
    api_key = os.environ.get("OPENAI_API_KEY")
    if not api_key:
        raise RuntimeError("OPENAI_API_KEY not set")
    client = OpenAI(api_key=api_key)
    if len(demo_img_bgr_list) != len(demo_masked_img_bgr_list):
        raise ValueError("demo_img_bgr_list 與 demo_masked_img_bgr_list 張數不一致")
    if len(demo_img_bgr_list) == 0:
        raise ValueError("至少需要一張 demo 圖")

    demo_uris = []
    mask_uris = []
    for img, m_img in zip(demo_img_bgr_list, demo_masked_img_bgr_list):
        demo_uris.append(_png_bytes_to_data_uri(_img_bgr_to_png_bytes(img)))
        mask_uris.append(_png_bytes_to_data_uri(_img_bgr_to_png_bytes(m_img)))

    system_prompt = (
        "You are a careful vision assistant.\n"
        "You will be given MULTIPLE PAIRS of images. Each PAIR of images shows the SAME scene and viewpoint but at DIFFERENT TIME steps of the SAME action sequence.\n"
        "\n"
        "*** IMAGE TYPES ***\n"
        "Each pair contains:\n"
        "- Image A (UNMASKED) shows true colors (may or may not contain yellow/orange cylinders in holes).\n"
        "- Image B (MASKED) reveals clearer hole contours of the black square platform.\n"
        "\n"
        "*** GOAL ***\n"
        "Based on ALL time steps together (not individually), determine which hole (quadrant) on the TOP SURFACE "
        "of the black square platform the cylinder is being/was inserted into.\n"
        "Use the sequence to track the gripper and cylinder motion toward a specific hole.\n"
        "\n"
        "*** IMPORTANT RULES ***\n"
        "- Define quadrants RELATIVE ONLY to the platform’s TOP SURFACE (not the full image frame).\n"
        "- The four holes are arranged at top_left, top_right, bottom_left, and bottom_right of the black square platform’s top surface in all scene.\n"
        "- Use the four right-angle corners of the BLACK SQUARE PLATFORM to infer quadrants under perspective.\n"
        "- If some holes are occluded, use Image B to infer their positions from the platform outline.\n"
        "- Use the motion direction across the frames to see which empty hole the gripper is targeting.\n"
        "- All frames belong to ONE continuous insertion motion. Do NOT treat each pair independently.\n"
        "- Do NOT use the whole image's top-left as reference; use the platform's top surface frame.\n"
        "- If the cylinder partially occludes the rim, still infer which quadrant that hole belongs to.\n"
        "\n"
        "*** STRICT OUTPUT ***\n"
        "{\"quadrant\":\"top_left|top_right|bottom_left|bottom_right\"}\n"
        "Return STRICT JSON ONLY. No extra words."
    )

    user_prompt = (
        "Here are multiple sequential demo frames. Each step includes:\n"
        "- Image A: real scene (real color)\n"
        "- Image B: same scene with masked object outlines\n"
        "Use ALL frames in order to determine which hole quadrant the cylinder is being inserted into on the platform’s top surface.\n"
        "Return strict JSON only."
    )

    message_content = [{"type": "text", "text": user_prompt}]
    for d_uri, m_uri in zip(demo_uris, mask_uris):
        message_content.append({"type": "image_url", "image_url": {"url": d_uri}})
        message_content.append({"type": "image_url", "image_url": {"url": m_uri}})

    resp = client.chat.completions.create(
        model=model,
        temperature=temperature,
        messages=[
            {"role":"system","content":system_prompt},
            {"role":"user","content":message_content},
        ],
    )
    text = resp.choices[0].message.content.strip()

    # 解析 JSON
    try:
        data = json.loads(text)
    except Exception:
        m = re.search(r"\{.*\}", text, flags=re.S)
        if not m:
            raise ValueError(f"VLM did not return valid JSON: {text}")
        data = json.loads(m.group(0))

    quad = str(data.get("quadrant","")).strip().lower()
    quad = quad.replace("-", "_")  # 把 top-left 轉成 top_left
    valid = {"top_left","top_right","bottom_left","bottom_right"}
    if quad not in valid: 
        raise ValueError(f"Invalid quadrant: {quad}")
    return quad


def ask_vlm_field_quadrant_to_id_map(
    field_img_with_ids_bgr: np.ndarray,
    model: str = "gpt-4.1",
    temperature: float = 0.0,
) -> dict:
    """問 Image B：讀上表面的四個洞數字，回傳 {top_left:int, top_right:int, bottom_left:int, bottom_right:int}。"""
    api_key = os.environ.get("OPENAI_API_KEY")
    if not api_key:
        raise RuntimeError("OPENAI_API_KEY not set")
    client = OpenAI(api_key=api_key)

    field_uri = _png_bytes_to_data_uri(_img_bgr_to_png_bytes(field_img_with_ids_bgr))

    system_prompt = (
        "You are a careful vision assistant.\n"
        "Image B (FIELD) shows the TOP SURFACE of the Platform with four circular holes labeled by numeric IDs.\n"
        "Task: Read the numeric ID printed near/inside each TOP-FACE hole and return a quadrant-to-ID mapping.\n"
        "Define quadrants RELATIVE ONLY to the platform’s TOP SURFACE (use platform’s four right-angle corners under perspective).\n"
        "Rules:\n"
        "- Only consider the TOP FACE holes (four of them).\n"
        "- Ignore the cylinder, hand, background, screws, or edges.\n"
        "Output STRICT JSON only with integer IDs:\n"
        "{\"top_left\": <int>, \"top_right\": <int>, \"bottom_left\": <int>, \"bottom_right\": <int>}.\n"
        "No extra words."
    )
    user_prompt = "Return JSON only with the quadrant-to-ID mapping for Image B's TOP-FACE holes."

    resp = client.chat.completions.create(
        model=model,
        temperature=temperature,
        messages=[
            {"role":"system","content":system_prompt},
            {"role":"user","content":[
                {"type":"text","text":user_prompt},
                {"type":"image_url","image_url":{"url": field_uri}},
            ]},
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

    required = ["top_left","top_right","bottom_left","bottom_right"]
    for k in required:
        if k not in data:
            raise ValueError(f"Missing key in mapping: {k}")
        try:
            data[k] = int(data[k])
        except Exception:
            raise ValueError(f"ID for {k} is not an integer: {data[k]}")

    # Debug: 印出 hole_ids
    hole_ids = list({data["top_left"], data["top_right"], data["bottom_left"], data["bottom_right"]})
    print("[Debug] hole_ids =", hole_ids)

    return data


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
            ]},
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
    
def get_xy_xyz_for_insert(
    demo_img_color_list: List[np.ndarray],
    field_img_color: np.ndarray,
    field_img_depth: np.ndarray,
    field_img_calib: Dict[str, Any],
) -> Optional[Tuple[float, float, float]]:
    """
    輸入:
      1) demo BGR 圖 list（numpy ndarray, uint8）
      2) field BGR 圖（numpy ndarray, uint8）
      3) field 深度圖（numpy ndarray, uint16）
      4) field 內參
    回傳：
      xyz_c                         -> 3D座標（公尺）或 None
    """ 

    # ===== SAM2 config =====
    CFG  = "sam2/configs/sam2.1/sam2.1_hiera_l.yaml"
    CKPT = "sam2/checkpoints/sam2.1_hiera_large.pt"
    CFG  = "sam2/configs/sam2.1/sam2.1_hiera_b+.yaml"
    CKPT = "sam2/checkpoints/sam2.1_hiera_base_plus.pt"

    POINTS_PER_SIDE        = 32
    PRED_IOU_THRESH        = 0.7
    STABILITY_SCORE_THRESH = 0.92
    MIN_MASK_REGION_AREA   = 100.0
    MASK_ALPHA             = 0.5

    # === 載入 SAM2 模型(之後可以做成只初始化一次的版本) ===
    from sam2.build_sam import build_sam2
    # from sam2.automatic_mask_generator import SAM2AutomaticMaskGenerator
    from sam2_client_1141115v2 import SAM2AutomaticMaskGenerator
    torch.cuda.empty_cache()
    device = "cuda" if torch.cuda.is_available() else "cpu"
    # device = "cpu"
    print(f"[Info] SAM2 device: {device}")
    # sam2_model = build_sam2(CFG, CKPT, device=device)

    mask_generator = SAM2AutomaticMaskGenerator(
        model=None,
        points_per_side=POINTS_PER_SIDE,
        pred_iou_thresh=PRED_IOU_THRESH,
        stability_score_thresh=STABILITY_SCORE_THRESH,
        min_mask_region_area=MIN_MASK_REGION_AREA,
    )

    # === 對 demo_img 蓋遮罩 -> processed_demo_img ===
    demo_pairs: List[Tuple[np.ndarray, np.ndarray]] = []

    for demo_img_color in demo_img_color_list:
        demo_rgb = cv2.cvtColor(demo_img_color, cv2.COLOR_BGR2RGB)
        with torch.inference_mode():
            torch.cuda.empty_cache()
            masks = mask_generator.generate(demo_rgb)

        processed = demo_img_color.copy()
        h, w = processed.shape[:2]
        rng = np.random.default_rng(42)

        for item in masks:
            m = item["segmentation"].astype(np.uint8)
            if m.ndim == 3: m = m[..., 0]
            if m.sum() == 0: continue
            color = rng.integers(0, 255, size=3, dtype=np.uint8)
            color_mask = np.zeros((h, w, 3), dtype=np.uint8)
            color_mask[m > 0] = color
            processed = cv2.addWeighted(processed, 1.0, color_mask, MASK_ALPHA, 0)

        demo_pairs.append((demo_img_color, processed))

    # === 對 field_img 蓋遮罩 -> processed_field_img + field_img_meta(JSON) ===
    field_rgb = cv2.cvtColor(field_img_color, cv2.COLOR_BGR2RGB)
    with torch.inference_mode():
        torch.cuda.empty_cache()
        field_masks = mask_generator.generate(field_rgb)

    processed_field_img = field_img_color.copy()
    rng2 = np.random.default_rng(123)

    # 取得內參
    color_intr = field_img_calib["color"]
    fx, fy = float(color_intr["fx"]), float(color_intr["fy"])
    ppx, ppy = float(color_intr["ppx"]), float(color_intr["ppy"])
    depth_scale = float(field_img_calib.get("depth_scale", 0.001))

    # 算中心點
    def _mask_center(mask_item):
        m = mask_item["segmentation"].astype(np.uint8)
        if m.ndim == 3: m = m[..., 0]
        if not m.any(): return None
        binmask = (m > 0).astype(np.uint8)
        M = cv2.moments(binmask)
        if M["m00"] != 0:
            return (M["m10"]/M["m00"], M["m01"]/M["m00"])
        ys, xs = np.where(binmask > 0)
        return (float(xs.mean()), float(ys.mean())) if len(xs) else None
    
    field_img_meta = []
    for idx, item in enumerate(field_masks):
        cxcy = _mask_center(item)
        if cxcy is None:
            continue
        cx, cy = cxcy
        field_img_meta.append({
            "index": idx,
            "center_xy": [float(cx), float(cy)],
            "center_xyz": None,
        })

        # 疊所有遮罩、畫所有中心 + ID
        m = item["segmentation"].astype(np.uint8)
        if m.ndim == 3: m = m[..., 0]
        binmask = (m > 0).astype(np.uint8)
        color = rng2.integers(0, 255, size=3, dtype=np.uint8)
        color_mask = np.zeros_like(processed_field_img)
        color_mask[binmask > 0] = color
        processed_field_img = cv2.addWeighted(processed_field_img, 1.0, color_mask, MASK_ALPHA, 0)
        cv2.circle(processed_field_img, (int(cx), int(cy)), 4, (0,0,0), -1)
        cv2.circle(processed_field_img, (int(cx), int(cy)), 3, (255,255,255), -1)
        cv2.putText(processed_field_img, str(idx), (int(cx)+6, int(cy)-6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 3, cv2.LINE_AA)
        cv2.putText(processed_field_img, str(idx), (int(cx)+6, int(cy)-6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)

    # === 這裡放你的 SAM2 on field_img + 中心/XYZ 計算 ===
    cx, cy = -1.0, -1.0
    xyz_c: Optional[Tuple[float, float, float]] = None
    target_id: Optional[int] = None

    use_vlm = bool(os.environ.get("OPENAI_API_KEY"))
    if not use_vlm:
        print("[VLM skip] OPENAI_API_KEY not set; skip VLM logic")
    else:
        try:
            demo_imgs = [pair[0] for pair in demo_pairs]
            demo_masks = [pair[1] for pair in demo_pairs]

            # 第一次：讀出洞位置
            quadrant = ask_vlm_demo_quadrant(
                demo_img_bgr_list=demo_imgs,
                demo_masked_img_bgr_list=demo_masks,
                model="gpt-4.1",
                temperature=0.0
            )
            print("[Debug] quadrant(A) =", quadrant)
            # 第二次：從 FIELD 圖判斷對應的洞 ID
            quad2id = ask_vlm_field_quadrant_to_id_map(processed_field_img, model="gpt-4.1", temperature=0.0)
            print("[Debug] quad2id(B) =", quad2id)
            target_id = int(quad2id[quadrant])
            cv2.imshow("Processed Field Image", processed_field_img)
            cv2.waitKey(100)
            print("[Debug] id_in_demo =", target_id)
            # check id is coorect, if not let user input manually
            # check id is correct, if not let user input manually
            user_input = input("Enter target ID: ")
            target_id = int(user_input)


        except Exception as e:
            print("[VLM warn][insert] quadrant-to-id mapping failed:", e)

        if target_id is not None:
            rec = next((r for r in field_img_meta if int(r["index"]) == target_id), None)
            if rec is not None:
                cx, cy = float(rec["center_xy"][0]), float(rec["center_xy"][1])
                if rec["center_xyz"] is not None:
                    xyz_c = (
                        float(rec["center_xyz"][0]),
                        float(rec["center_xyz"][1]),
                        float(rec["center_xyz"][2]),
                    )
            else:
                print(f"[VLM warn] id = {target_id} not found in meta")

    # 算深度第 5 百分位
    def _min_depth(depth_u16, mask_item, depth_scale):
        outer_px   = 3   # 外側環帶厚度
        percentile = 10.0  # 第 10 百分位，抗雜訊

        # 取得 segmentation mask
        m = mask_item["segmentation"].astype(np.uint8)
        if m.ndim == 3:
            m = m[..., 0]
        m = (m > 0).astype(np.uint8)
        if m.sum() == 0:
            return None

        # 用 morphology 膨脹遮罩
        kernel   = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        m_dilate = cv2.dilate(m, kernel, iterations=outer_px)
        ring_out = (m_dilate.astype(bool) & (~m.astype(bool)))  # 外圈 = 膨脹後減原遮罩

        # 若外圈像素太少，退回整個遮罩避免空集合
        region = ring_out if ring_out.sum() >= 30 else (m > 0)

        vals = depth_u16[region]
        vals = vals[vals > 0]  # 去除無效深度
        if vals.size == 0:
            return None

        z = np.percentile(vals, percentile) * depth_scale
        return float(z)
    
    def _rim_mean_lr_depth(depth_u16, mask_item, depth_scale, dilate_px=3):
        """
        從『洞口遮罩』膨脹產生『外圈 ring』，取 ring 左側深度 + 右側深度 → 平均 → 得洞口平面的 Z。
        """
        m = mask_item["segmentation"].astype(np.uint8)
        if m.ndim == 3:
            m = m[..., 0]
        m = (m > 0).astype(np.uint8)
        if m.sum() == 0:
            return None

        # 1. 取 bounding box，用來確保只看洞口附近的 ring
        ys, xs = np.where(m > 0)
        y_min, y_max = ys.min(), ys.max()
        x_min, x_max = xs.min(), xs.max()
        
        # 2. 膨脹產生外圈
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        m_dilate = cv2.dilate(m, kernel, iterations=dilate_px)
        ring = (m_dilate.astype(bool) & (~m.astype(bool)))  # True = 外圈像素

        # 3. 限制 ring 在 bounding box 周圍（略放大，避免取到太外面的平面）
        pad = max(5, dilate_px + 2)
        y0, y1 = max(0, y_min - pad), min(depth_u16.shape[0], y_max + pad)
        x0, x1 = max(0, x_min - pad), min(depth_u16.shape[1], x_max + pad)
        ring_crop = ring[y0:y1, x0:x1]
        depth_crop = depth_u16[y0:y1, x0:x1]

        # 4. 在 ring_crop 中抓左半 + 右半像素深度
        h, w = ring_crop.shape
        mid = w // 2

        left_region  = depth_crop[:, :mid][ring_crop[:, :mid]]
        right_region = depth_crop[:, mid:][ring_crop[:, mid:]]

        left_valid  = left_region[left_region > 0]
        right_valid = right_region[right_region > 0]

        if left_valid.size == 0 or right_valid.size == 0:
            return None

        z_left  = np.median(left_valid)  * depth_scale
        z_right = np.median(right_valid) * depth_scale
        z_mean  = (z_left + z_right) * 0.5

        return float(z_mean)
    
    # target 的 cx,cy
    if target_id is not None:
        # 從 field_img_meta 抓出 target 的 cx, cy
        rec = next((r for r in field_img_meta if int(r["index"]) == target_id), None)
        if rec is not None:
            cx, cy = float(rec["center_xy"][0]), float(rec["center_xy"][1])
        else:
            print(f"[VLM warn] id = {target_id} not found in meta")

    if target_id is not None and target_id < len(field_masks) and cx >= 0 and cy >= 0:
        target_mask = field_masks[target_id]
        Z = _min_depth(field_img_depth, target_mask, depth_scale) # 第五百分位
        # Z = _rim_mean_lr_depth(field_img_depth, target_mask, depth_scale)
        if Z is not None and cx >= 0 and cy >= 0:
            # 反投影成相機座標系下的 X, Y, Z
            X = (cx - ppx) * Z / fx
            Y = (cy - ppy) * Z / fy
            xyz_c = (X, Y, Z)
            print(f"[Insert] 3D hole center = {xyz_c}")
        else:
            print("[Insert warn] 無法取得有效深度，xyz_c = None")

    else:
        print("[Insert warn] 無有效 target_id，無法計算 3D 座標")

    # return (cx, cy), xyz_c, processed_demo_img, processed_field_img, field_img_meta
    # for i, (raw, masked) in enumerate(demo_pairs):
    #     cv2.imshow(f"Demo {i}", raw)
    #     cv2.imshow(f"Processed Demo {i}", masked)
    cv2.imshow("Processed Field Image", processed_field_img)
    #xyz_c = np.array(xyz_c)*10
    return xyz_c

def get_xy_xyz_for_grasp(
    demo_img_color_list: List[np.ndarray],
    field_img_color: np.ndarray,
    field_img_depth: np.ndarray,
    field_img_calib: Dict[str, Any],
) -> Optional[Tuple[float, float, float]]:
    """
    輸入:
      1) demo BGR 圖 list（numpy ndarray, uint8）
      2) field BGR 圖（numpy ndarray, uint8）
      3) field 深度圖（numpy ndarray, uint16）
      4) field 內參
    回傳：
      xyz_c                         -> 3D座標（公尺）或 None
    """ 

    # ===== SAM2 config =====

    CFG  = "sam2/configs/sam2.1/sam2.1_hiera_l.yaml"
    CKPT = "sam2/checkpoints/sam2.1_hiera_large.pt"
    CFG  = "sam2/configs/sam2.1/sam2.1_hiera_b+.yaml"
    CKPT = "sam2/checkpoints/sam2.1_hiera_base_plus.pt"

    POINTS_PER_SIDE        = 32
    PRED_IOU_THRESH        = 0.7
    STABILITY_SCORE_THRESH = 0.92
    MIN_MASK_REGION_AREA   = 100.0
    MASK_ALPHA             = 0.5

    # === 載入 SAM2 模型(之後可以做成只初始化一次的版本) ===
    # from sam2.build_sam import build_sam2
    # from sam2.automatic_mask_generator import SAM2AutomaticMaskGenerator
    from sam2_client_1141115v2 import SAM2AutomaticMaskGenerator
    torch.cuda.empty_cache()
    device = "cuda" if torch.cuda.is_available() else "cpu"
    # device = "cpu"
    print(f"[Info] SAM2 device: {device}")
    # sam2_model = build_sam2(CFG, CKPT, device=device)

    mask_generator = SAM2AutomaticMaskGenerator(
        model=None,
        points_per_side=POINTS_PER_SIDE,
        pred_iou_thresh=PRED_IOU_THRESH,
        stability_score_thresh=STABILITY_SCORE_THRESH,
        min_mask_region_area=MIN_MASK_REGION_AREA,
    )

    # === field_img -> processed_field_img + field_img_meta(JSON) ===
    field_rgb = cv2.cvtColor(field_img_color, cv2.COLOR_BGR2RGB)
    with torch.inference_mode():
        torch.cuda.empty_cache()
        field_masks = mask_generator.generate(field_rgb)

    processed_field_img = field_img_color.copy()

    # 取得內參
    color_intr = field_img_calib["color"]
    fx, fy = float(color_intr["fx"]), float(color_intr["fy"])
    ppx, ppy = float(color_intr["ppx"]), float(color_intr["ppy"])
    depth_scale = float(field_img_calib.get("depth_scale", 0.001))

    # 算中心點
    def _mask_center(mask_item):
        m = mask_item["segmentation"].astype(np.uint8)
        if m.ndim == 3: m = m[..., 0]
        if not m.any(): return None
        binmask = (m > 0).astype(np.uint8)
        M = cv2.moments(binmask)
        if M["m00"] != 0:
            return (M["m10"]/M["m00"], M["m01"]/M["m00"])
        ys, xs = np.where(binmask > 0)
        return (float(xs.mean()), float(ys.mean())) if len(xs) else None

    field_img_meta = []
    for idx, item in enumerate(field_masks):
        cxcy = _mask_center(item)
        if cxcy is None:
            continue
        cx, cy = cxcy
        field_img_meta.append({
            "index": idx,
            "center_xy": [float(cx), float(cy)],
            "center_xyz": None,
        })
        
        # 畫所有中心 + ID
        m = item["segmentation"].astype(np.uint8)
        if m.ndim == 3: m = m[..., 0]
        binmask = (m > 0).astype(np.uint8)
        cv2.circle(processed_field_img, (int(cx), int(cy)), 4, (0,0,0), -1)
        cv2.circle(processed_field_img, (int(cx), int(cy)), 3, (255,255,255), -1)
        cv2.putText(processed_field_img, str(idx), (int(cx)+6, int(cy)-6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 3, cv2.LINE_AA)
        cv2.putText(processed_field_img, str(idx), (int(cx)+6, int(cy)-6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)

    # === 這裡放你的 SAM2 on field_img + 中心/XYZ 計算 ===
    cx, cy = -1.0, -1.0      # for target
    xyz_c: Optional[Tuple[float, float, float]] = None
    target_id: Optional[int] = None

    use_vlm = bool(os.environ.get("OPENAI_API_KEY"))
    if not use_vlm:
        print("[VLM skip] OPENAI_API_KEY not set; skip VLM logic")
    else:
        try:
            # 第一次：讀出所有 cylinder IDs
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

        if target_id is not None:
            rec = next((r for r in field_img_meta if int(r["index"]) == target_id), None)
            if rec is not None:
                cx, cy = float(rec["center_xy"][0]), float(rec["center_xy"][1])
                if rec["center_xyz"] is not None:
                    xyz_c = (
                        float(rec["center_xyz"][0]),
                        float(rec["center_xyz"][1]),
                        float(rec["center_xyz"][2]),
                    )
            else:
                print(f"[VLM warn] id = {target_id} not found in meta")
    
    # === RANSAC 分離頂面 ===
    if target_id is not None and target_id < len(field_masks):
        mask_obj = field_masks[target_id]["segmentation"].astype(np.uint8)

        print("分離頂面前xyz_c:", xyz_c)
        
        cylinder_only = cv2.bitwise_and(field_img_color, field_img_color, mask=mask_obj)
        cv2.imshow("Cylinder Only (SAM2 cropped)", cylinder_only)

        M_top, center3d, normal3d, center2d = extract_top_plane(
            mask_obj=mask_obj,
            depth_u16=field_img_depth,
            depth_scale=depth_scale,
            K={"fx": fx, "fy": fy, "ppx": ppx, "ppy": ppy},
            dist_thresh=0.003,  # 約3mm公差
            iters=2000,
            min_inliers=200
        )

        if M_top is not None:
            # --- refine 3D center based on 2D mask geometry ---
            cy, cx = np.mean(np.where(M_top > 0), axis=1)
            Z = np.median(field_img_depth[M_top > 0]) * depth_scale
            X = (cx - ppx) * Z / fx
            Y = (cy - ppy) * Z / fy
            center3d_refined = np.array([X, Y, Z])
            # xyz_c = center3d_refined
            xyz_c = center3d
            print(f"[RANSAC] top center: {center3d_refined}")

            # === 在影像上顯示 center3d_refined ===
            def project_point(K, p3d):
                fx, fy, cx, cy = K["fx"], K["fy"], K["ppx"], K["ppy"]
                X, Y, Z = p3d
                if Z <= 0:
                    return None
                u = int(round(fx * X / Z + cx))
                v = int(round(fy * Y / Z + cy))
                return (u, v)

            K = {"fx": fx, "fy": fy, "ppx": ppx, "ppy": ppy}
            uv = project_point(K, center3d_refined)
            # uv = center2d
            overlay = field_img_color.copy()
            if uv is not None:
                u, v = uv
                cv2.circle(overlay, (u, v), 5, (0, 0, 0), -1)
                cv2.circle(overlay, (u, v), 3, (255, 255, 255), -1)
                cv2.putText(overlay, f"Z={center3d_refined[2]*100:.1f}cm", (u + 6, v - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 2)
                cv2.putText(overlay, f"Z={center3d_refined[2]*100:.1f}cm", (u + 6, v - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
            
            cv2.imshow("Center3D projected", overlay)
        else:
            print("[RANSAC warn] 無法擬合出穩定平面，沿用 SAM2 中心點。")

    # return (cx, cy), xyz_c, processed_demo_img, processed_field_img, field_img_meta
    # for i, img in enumerate(demo_img_color_list):
    #     cv2.imshow(f"Demo {i}", img)
    cv2.imshow("Processed Field Image", processed_field_img)
    #xyz_c = np.array(xyz_c)*10
    return xyz_c

import numpy as np
from scipy.spatial.transform import Rotation as R
def c2e(p_C0_o0,T_a_C0):
    '''
    p_C0_o0 : np.array([x,y,z])
    T_a_C0 : np.array([x,y,z,rx,ry,rz,rw])
    =>
    T_a_C1 np.array([x,y,z,rx,ry,rz,rw])
    '''
    import yaml
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

    T_C0_o0 = np.array(
        [[ 1, 0, 0, p_C0_o0[0]],
         [ 0, 1, 0, p_C0_o0[1]],
         [ 0, 0, 1, p_C0_o0[2]],
         [ 0, 0, 0, 1]]
    )
    temp = R.from_quat(T_a_C0[3:]).as_matrix()
    T_a_C0 = np.array(
        [[ temp[0,0], temp[0,1], temp[0,2], T_a_C0[0]],
         [ temp[1,0], temp[1,1], temp[1,2], T_a_C0[1]],
         [ temp[2,0], temp[2,1], temp[2,2], T_a_C0[2]],
         [ 0, 0, 0, 1]]
    )
    T_a_o0 = T_a_C0 @ T_C0_o0
    T_a_E1 = T_a_o0
    rot = R.from_matrix(T_a_E1[:3,:3]).as_euler('xyz', degrees=True)
    rot[0],rot[1] = 180.0,0.0
    T_a_E1[:3,:3] = R.from_euler('xyz', rot, degrees=True).as_matrix()
    T_a_C1 = T_a_E1 @ T_E_C
    temp = R.from_matrix(T_a_C1[:3,:3]).as_quat()
    T_a_C1 = np.array([T_a_C1[0,3],T_a_C1[1,3],T_a_C1[2,3],temp[0],temp[1],temp[2],temp[3]])
    return T_a_C1

def rotGripper2Horizon(T_a_ci):
    '''
    T_a_ci : np.array([x,y,z,rx,ry,rz,rw])
    =>
    T_a_ci : np.array([x,y,z,rx,ry,rz,rw]) rotate to horizon
    '''
    T_G_C = np.array(
        [[ 9.999239e-01,-2.960013e-03, 1.197887e-02,-8.680687e-03],
        [ 5.468441e-04, 9.804773e-01, 1.966315e-01,-6.767428e-02],
        [-1.232704e-02,-1.966100e-01, 9.804043e-01, 3.350969e-02],
        [ 0.000000e+00, 0.000000e+00, 0.000000e+00, 1.000000e+00]]
    )
    T_G_E = np.array(
        [[ 1, 0, 0, 0],
        [ 0, 1, 0, 0],
        [ 0, 0, 1, 0.164],
        [ 0, 0, 0, 1]]
    )
    temp = np.eye(4)
    temp[:3,:3] = R.from_quat(T_a_ci[3:]).as_matrix()
    temp[:3,3] = np.array(T_a_ci[:3])
    T_a_ci = temp.copy()
    T_a_E = T_a_ci @ np.linalg.inv(T_G_C) @ T_G_E
    rot = R.from_matrix(T_a_E[:3,:3]).as_euler('xyz', degrees=True)
    rot[0],rot[1] = 180.0,0.0
    T_a_E[:3,:3] = R.from_euler('xyz', rot, degrees=True).as_matrix()
    T_a_ci = T_a_E @ np.linalg.inv(T_G_E) @ T_G_C
    x,y,z = T_a_ci[:3,3]
    rx,ry,rz,rw = R.from_matrix(T_a_ci[:3,:3]).as_quat()
    temp = np.array([x,y,z,rx,ry,rz,rw])

    return temp