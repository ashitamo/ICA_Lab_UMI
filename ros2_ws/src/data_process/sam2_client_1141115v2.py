# client.py
import json
import base64
import requests
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont
import io
import numpy as np
import cv2
from typing import List, Dict, Any, Optional

# SERVER_URL = "http://192.168.50.223:80"  # ex: 606連這個
SERVER_URL = "http://140.113.149.16:8113"  # ex: 外網連這個


class SAM2AutomaticMaskGenerator:
    """
    客戶端版本的 SAM2AutomaticMaskGenerator，通過網絡調用 SAM2 服務器
    API 兼容原本的 SAM2AutomaticMaskGenerator，可以無縫替代
    """
    
    def __init__(
        self,
        model=None,  # 保留參數以兼容原本的 API，但不使用
        points_per_side: Optional[int] = 32,
        points_per_batch: int = 64,
        pred_iou_thresh: float = 0.8,
        stability_score_thresh: float = 0.95,
        stability_score_offset: float = 1.0,
        mask_threshold: float = 0.0,
        box_nms_thresh: float = 0.7,
        crop_n_layers: int = 0,
        crop_nms_thresh: float = 0.7,
        crop_overlap_ratio: float = 512 / 1500,
        crop_n_points_downscale_factor: int = 1,
        point_grids: Optional[List[np.ndarray]] = None,
        min_mask_region_area: int = 0,
        output_mode: str = "binary_mask",
        use_m2m: bool = False,
        multimask_output: bool = True,
        server_url: str = SERVER_URL,
        **kwargs,
    ) -> None:
        """
        初始化客戶端版本的 SAM2AutomaticMaskGenerator
        
        Args:
            model: 保留以兼容原始 API，但不使用（服務器端已載入模型）
            points_per_side: 每邊採樣點數
            pred_iou_thresh: 預測 IOU 閾值
            stability_score_thresh: 穩定性分數閾值
            server_url: SAM2 服務器的 URL
            其他參數保留以兼容原始 API
        """
        self.points_per_side = points_per_side
        self.pred_iou_thresh = pred_iou_thresh
        self.stability_score_thresh = stability_score_thresh
        self.min_mask_region_area = min_mask_region_area
        self.server_url = server_url
        
        # 保存其他參數（雖然目前服務器端不一定全部使用）
        self.points_per_batch = points_per_batch
        self.stability_score_offset = stability_score_offset
        self.mask_threshold = mask_threshold
        self.box_nms_thresh = box_nms_thresh
        self.crop_n_layers = crop_n_layers
        self.crop_nms_thresh = crop_nms_thresh
        self.crop_overlap_ratio = crop_overlap_ratio
        self.crop_n_points_downscale_factor = crop_n_points_downscale_factor
        self.point_grids = point_grids
        self.output_mode = output_mode
        self.use_m2m = use_m2m
        self.multimask_output = multimask_output
        
    def generate(self, image: np.ndarray) -> List[Dict[str, Any]]:
        """
        在圖像上生成 masks，兼容原始 SAM2AutomaticMaskGenerator 的 API
        
        Args:
            image: RGB 格式的 numpy array，shape 為 (H, W, 3)
        
        Returns:
            List[Dict]: 每個字典包含:
                - segmentation: (H, W) 的 boolean numpy array
                - area: int, mask 的像素面積
                - bbox: [x, y, w, h] 格式的邊界框
                - predicted_iou: float, 預測的 IOU 分數
                - stability_score: float, 穩定性分數（目前用 predicted_iou 代替）
        """
        # 將 numpy array 轉換為 PIL Image
        if image.dtype == np.uint8:
            pil_img = Image.fromarray(image, mode='RGB')
        else:
            # 如果不是 uint8，先轉換
            pil_img = Image.fromarray((image * 255).astype(np.uint8), mode='RGB')
        
        # 將 PIL Image 轉換為字節流
        img_byte_arr = io.BytesIO()
        pil_img.save(img_byte_arr, format='JPEG')
        img_byte_arr.seek(0)
        
        # 發送請求到服務器
        files = {
            "image": ("image.jpg", img_byte_arr, "image/jpeg")
        }
        
        data = {
            "points_per_side": str(self.points_per_side),
            "pred_iou_thresh": str(self.pred_iou_thresh),
            "stability_score_thresh": str(self.stability_score_thresh),
        }
        
        try:
            resp = requests.post(f"{self.server_url}/segment_auto", files=files, data=data)
            
            if resp.status_code != 200:
                raise RuntimeError(f"服務器返回錯誤: {resp.status_code}, {resp.text}")
            
            result = resp.json()
        except Exception as e:
            raise RuntimeError(f"調用 SAM2 服務器失敗: {e}")
        
        # 將服務器返回的結果轉換為兼容原始 API 的格式
        masks_list = []
        
        for i in range(result['num_masks']):
            # 解碼 base64 mask
            mask_b64 = result['masks'][i]
            mask_data = base64.b64decode(mask_b64)
            mask_img = Image.open(io.BytesIO(mask_data)).convert('L')
            mask_array = np.array(mask_img) > 0  # 轉換為 boolean array
            
            # 邊界框從 [x_min, y_min, x_max, y_max] 轉換為 [x, y, w, h]
            bbox_xyxy = result['bboxes'][i]
            if bbox_xyxy is not None:
                x_min, y_min, x_max, y_max = bbox_xyxy
                bbox_xywh = [x_min, y_min, x_max - x_min, y_max - y_min]
            else:
                bbox_xywh = [0, 0, 0, 0]
            
            # 構造兼容原始 API 的字典
            mask_dict = {
                'segmentation': mask_array,
                'area': result['areas'][i],
                'bbox': bbox_xywh,
                'predicted_iou': result['scores'][i],
                'stability_score': result['scores'][i],  # 使用 predicted_iou 作為 stability_score
                'crop_box': [0, 0, image.shape[1], image.shape[0]],  # 整張圖
                'point_coords': None,  # 自動生成的，沒有特定點
            }
            
            masks_list.append(mask_dict)
        
        return masks_list

def save_mask_base64(mask_b64: str, out_path: Path):
    """保存 mask 為 PNG 檔案"""
    data = base64.b64decode(mask_b64)
    img = Image.open(io.BytesIO(data))
    img.save(out_path)


def draw_bbox_on_image(image: Image.Image, bbox: list, color=(255, 0, 0), width=3, label=None):
    """在圖像上繪製邊界框"""
    draw = ImageDraw.Draw(image)
    x_min, y_min, x_max, y_max = bbox
    
    # 繪製矩形邊框
    draw.rectangle([x_min, y_min, x_max, y_max], outline=color, width=width)
    
    # 如果有標籤，繪製文字
    if label:
        try:
            font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 20)
        except:
            font = ImageFont.load_default()
        
        # 計算文字背景
        text_bbox = draw.textbbox((x_min, y_min - 25), label, font=font)
        draw.rectangle(text_bbox, fill=color)
        draw.text((x_min, y_min - 25), label, fill=(255, 255, 255), font=font)
    
    return image


def draw_contours_on_image(image: Image.Image, contours: list, color=(0, 255, 0), width=2):
    """在圖像上繪製輪廓"""
    draw = ImageDraw.Draw(image)
    
    for contour in contours:
        if len(contour) > 1:
            # 將輪廓轉換為 tuple 列表以供 PIL 使用
            points = [(int(p[0]), int(p[1])) for p in contour]
            draw.line(points + [points[0]], fill=color, width=width)
    
    return image


def overlay_mask_on_image(image: Image.Image, mask_b64: str, color=(255, 0, 0), alpha=0.5):
    """將 mask 疊加到原圖上"""
    # 解碼 mask
    mask_data = base64.b64decode(mask_b64)
    mask_img = Image.open(io.BytesIO(mask_data)).convert('L')
    
    # 創建彩色 mask
    colored_mask = Image.new('RGBA', image.size, color + (0,))
    
    # 將 mask 轉換為 alpha 通道
    mask_array = np.array(mask_img)
    alpha_array = (mask_array * alpha).astype(np.uint8)
    colored_mask.putalpha(Image.fromarray(alpha_array))
    
    # 疊加
    result = image.convert('RGBA')
    result = Image.alpha_composite(result, colored_mask)
    
    return result.convert('RGB')


def segment_with_points(image_path: str, points: list, labels: list, 
                        out_dir: Path, prefix: str, multimask_output=True):
    """
    使用點提示進行分割
    
    Args:
        image_path: 圖像路徑
        points: 點座標列表 [[x1, y1], [x2, y2], ...]
        labels: 點標籤列表 [1, 0, ...] (1=前景, 0=背景)
        out_dir: 輸出目錄
        prefix: 輸出檔案前綴
        multimask_output: 是否返回多個候選 mask
    """
    print(f"\n{'='*60}")
    print(f"案例: {prefix}")
    print(f"圖像: {image_path}")
    print(f"點座標: {points}")
    print(f"點標籤: {labels}")
    print(f"{'='*60}")
    
    with open(image_path, "rb") as f:
        files = {
            "image": (Path(image_path).name, f, "image/jpeg")
        }

        data = {
            "points": json.dumps(points),
            "labels": json.dumps(labels),
            "multimask_output": str(multimask_output).lower(),
        }

        resp = requests.post(f"{SERVER_URL}/segment", files=files, data=data)

    if resp.status_code != 200:
        print("❌ 錯誤:", resp.status_code, resp.text)
        return

    result = resp.json()
    print(f"✓ 圖像大小: {result['width']} x {result['height']}")
    print(f"✓ 獲得 {result['num_masks']} 個 masks")
    print(f"✓ 分數: {[f'{s:.3f}' for s in result['scores']]}")

    # 讀取原圖
    original_img = Image.open(image_path).convert('RGB')

    # 保存每個 mask 及其視覺化
    for i, (mask_b64, score, bbox, contours) in enumerate(
        zip(result["masks"], result["scores"], result["bboxes"], result["contours"])
    ):
        # 保存純 mask
        mask_file = out_dir / f"{prefix}_mask_{i}.png"
        save_mask_base64(mask_b64, mask_file)
        print(f"  └─ 保存 mask: {mask_file}")
        
        # 創建視覺化圖像（帶邊框和輪廓）
        vis_img = original_img.copy()
        
        # 疊加 mask
        vis_img = overlay_mask_on_image(vis_img, mask_b64, color=(255, 0, 0), alpha=0.4)
        
        # 繪製邊界框
        if bbox:
            label = f"Mask {i} (Score: {score:.3f})"
            vis_img = draw_bbox_on_image(vis_img, bbox, color=(0, 255, 0), width=3, label=label)
            print(f"  └─ 邊界框: {bbox}")
        
        # 繪製輪廓
        if contours:
            vis_img = draw_contours_on_image(vis_img, contours, color=(255, 255, 0), width=2)
            print(f"  └─ 輪廓數量: {len(contours)}")
        
        # 保存視覺化結果
        vis_file = out_dir / f"{prefix}_visual_{i}.png"
        vis_img.save(vis_file)
        print(f"  └─ 保存視覺化: {vis_file}")


def segment_auto(image_path: str, out_dir: Path, prefix: str, 
                 points_per_side=32, max_masks=20):
    """
    自動分割整張圖像，無需提供點提示
    
    Args:
        image_path: 圖像路徑
        out_dir: 輸出目錄
        prefix: 輸出檔案前綴
        points_per_side: 每邊採樣點數
        max_masks: 最多保存多少個 masks（按面積排序）
    """
    print(f"\n{'='*60}")
    print(f"案例: {prefix} (自動分割)")
    print(f"圖像: {image_path}")
    print(f"每邊採樣點數: {points_per_side}")
    print(f"{'='*60}")
    
    with open(image_path, "rb") as f:
        files = {
            "image": (Path(image_path).name, f, "image/jpeg")
        }

        data = {
            "points_per_side": str(points_per_side),
        }

        resp = requests.post(f"{SERVER_URL}/segment_auto", files=files, data=data)

    if resp.status_code != 200:
        print("❌ 錯誤:", resp.status_code, resp.text)
        return

    result = resp.json()
    print(f"✓ 圖像大小: {result['width']} x {result['height']}")
    print(f"✓ 獲得 {result['num_masks']} 個 masks")
    
    # 讀取原圖
    original_img = Image.open(image_path).convert('RGB')
    
    # 創建一個綜合視覺化，顯示所有 masks
    all_masks_img = original_img.copy()
    
    # 限制顯示的 mask 數量
    num_to_show = min(max_masks, result['num_masks'])
    print(f"✓ 顯示前 {num_to_show} 個 masks（按面積排序）")
    
    # 顏色列表（用於區分不同的 masks）
    colors = [
        (255, 0, 0), (0, 255, 0), (0, 0, 255),
        (255, 255, 0), (255, 0, 255), (0, 255, 255),
        (128, 0, 0), (0, 128, 0), (0, 0, 128),
        (128, 128, 0), (128, 0, 128), (0, 128, 128),
    ]

    # 保存每個 mask 及其視覺化
    for i in range(num_to_show):
        mask_b64 = result["masks"][i]
        score = result["scores"][i]
        bbox = result["bboxes"][i]
        contours = result["contours"][i]
        area = result["areas"][i]
        
        color = colors[i % len(colors)]
        
        print(f"\n  Mask {i}:")
        print(f"  └─ 分數: {score:.3f}")
        print(f"  └─ 面積: {area} 像素")
        print(f"  └─ 邊界框: {bbox}")
        
        # 保存純 mask
        mask_file = out_dir / f"{prefix}_mask_{i}.png"
        save_mask_base64(mask_b64, mask_file)
        
        # 創建單個 mask 的視覺化
        vis_img = original_img.copy()
        vis_img = overlay_mask_on_image(vis_img, mask_b64, color=color, alpha=0.5)
        
        # 繪製邊界框
        label = f"Mask {i} (Area: {area})"
        vis_img = draw_bbox_on_image(vis_img, bbox, color=color, width=3, label=label)
        
        # 繪製輪廓
        if contours:
            vis_img = draw_contours_on_image(vis_img, contours, color=(255, 255, 255), width=2)
        
        # 保存單個視覺化
        vis_file = out_dir / f"{prefix}_visual_{i}.png"
        vis_img.save(vis_file)
        
        # 將此 mask 添加到綜合視覺化中
        all_masks_img = overlay_mask_on_image(all_masks_img, mask_b64, color=color, alpha=0.3)
        all_masks_img = draw_bbox_on_image(all_masks_img, bbox, color=color, width=2, label=str(i))
    
    # 保存綜合視覺化
    all_masks_file = out_dir / f"{prefix}_all_masks.png"
    all_masks_img.save(all_masks_file)
    print(f"\n  └─ 保存綜合視覺化: {all_masks_file}")


def segment_auto_overlay_only(image_path: str, out_dir: Path, prefix: str, 
                               points_per_side=32, max_masks=30, alpha=0.5):
    """
    自動分割並將所有 masks 疊加在一張圖上（不顯示邊界框）
    
    Args:
        image_path: 圖像路徑
        out_dir: 輸出目錄
        prefix: 輸出檔案前綴
        points_per_side: 每邊採樣點數
        max_masks: 最多顯示多少個 masks
        alpha: mask 透明度 (0-1)
    """
    print(f"\n{'='*60}")
    print(f"案例: {prefix} (自動分割 - 純 mask 疊加)")
    print(f"圖像: {image_path}")
    print(f"每邊採樣點數: {points_per_side}")
    print(f"{'='*60}")
    
    with open(image_path, "rb") as f:
        files = {
            "image": (Path(image_path).name, f, "image/jpeg")
        }

        data = {
            "points_per_side": str(points_per_side),
        }

        resp = requests.post(f"{SERVER_URL}/segment_auto", files=files, data=data)

    if resp.status_code != 200:
        print("❌ 錯誤:", resp.status_code, resp.text)
        return

    result = resp.json()
    print(f"✓ 圖像大小: {result['width']} x {result['height']}")
    print(f"✓ 獲得 {result['num_masks']} 個 masks")
    
    # 讀取原圖
    original_img = Image.open(image_path).convert('RGB')
    
    # 創建疊加所有 masks 的圖像
    overlay_img = original_img.copy()
    
    # 限制顯示的 mask 數量
    num_to_show = min(max_masks, result['num_masks'])
    print(f"✓ 將前 {num_to_show} 個 masks 疊加到一張圖上（按面積排序）")
    
    # 顏色列表（用於區分不同的 masks）
    colors = [
        (255, 0, 0), (0, 255, 0), (0, 0, 255),
        (255, 255, 0), (255, 0, 255), (0, 255, 255),
        (255, 128, 0), (128, 255, 0), (0, 255, 128),
        (128, 0, 255), (255, 0, 128), (0, 128, 255),
        (192, 64, 0), (64, 192, 0), (0, 192, 64),
        (192, 0, 64), (64, 0, 192), (0, 64, 192),
        (255, 192, 128), (192, 255, 128), (128, 255, 192),
        (255, 128, 192), (128, 192, 255), (192, 128, 255),
    ]

    # 疊加每個 mask（只有顏色遮罩，沒有邊界框）
    for i in range(num_to_show):
        mask_b64 = result["masks"][i]
        area = result["areas"][i]
        
        color = colors[i % len(colors)]
        
        # 只疊加 mask，不繪製邊界框
        overlay_img = overlay_mask_on_image(overlay_img, mask_b64, color=color, alpha=alpha)
        
        if i < 10:  # 只顯示前 10 個的詳細信息
            print(f"  Mask {i}: 面積 {area} 像素, 顏色 RGB{color}")
    
    if num_to_show > 10:
        print(f"  ... 以及其他 {num_to_show - 10} 個 masks")
    
    # 保存疊加結果
    overlay_file = out_dir / f"{prefix}_overlay_only.png"
    overlay_img.save(overlay_file)
    print(f"\n  └─ 保存純疊加視覺化: {overlay_file}")
    print(f"  └─ 所有 {num_to_show} 個 masks 已疊加（無邊界框）")


def segment_auto_contours_only(image_path: str, out_dir: Path, prefix: str, 
                                points_per_side=32, max_masks=30, line_width=2,
                                use_fixed_color=False, fixed_color=(0, 255, 0)):
    """
    自動分割並只繪製輪廓描邊在一張圖上（不填充色塊，不顯示邊界框）
    
    Args:
        image_path: 圖像路徑
        out_dir: 輸出目錄
        prefix: 輸出檔案前綴
        points_per_side: 每邊採樣點數
        max_masks: 最多顯示多少個 masks
        line_width: 輪廓線寬度
        use_fixed_color: 是否使用固定顏色（True=所有輪廓同色，False=多色）
        fixed_color: 固定顏色的 RGB 值（當 use_fixed_color=True 時使用）
    """
    print(f"\n{'='*60}")
    print(f"案例: {prefix} (自動分割 - 純輪廓描邊)")
    print(f"圖像: {image_path}")
    print(f"每邊採樣點數: {points_per_side}")
    if use_fixed_color:
        print(f"顏色模式: 固定顏色 RGB{fixed_color}")
    else:
        print(f"顏色模式: 多彩色")
    print(f"{'='*60}")
    
    with open(image_path, "rb") as f:
        files = {
            "image": (Path(image_path).name, f, "image/jpeg")
        }

        data = {
            "points_per_side": str(points_per_side),
        }

        resp = requests.post(f"{SERVER_URL}/segment_auto", files=files, data=data)

    if resp.status_code != 200:
        print("❌ 錯誤:", resp.status_code, resp.text)
        return

    result = resp.json()
    print(f"✓ 圖像大小: {result['width']} x {result['height']}")
    print(f"✓ 獲得 {result['num_masks']} 個 masks")
    
    # 讀取原圖
    original_img = Image.open(image_path).convert('RGB')
    
    # 創建只有輪廓的圖像
    contours_img = original_img.copy()
    
    # 限制顯示的 mask 數量
    num_to_show = min(max_masks, result['num_masks'])
    print(f"✓ 將前 {num_to_show} 個 masks 的輪廓繪製到一張圖上（按面積排序）")
    
    # 顏色列表（用於區分不同的輪廓）
    colors = [
        (255, 0, 0), (0, 255, 0), (0, 0, 255),
        (255, 255, 0), (255, 0, 255), (0, 255, 255),
        (255, 128, 0), (128, 255, 0), (0, 255, 128),
        (128, 0, 255), (255, 0, 128), (0, 128, 255),
        (192, 64, 0), (64, 192, 0), (0, 192, 64),
        (192, 0, 64), (64, 0, 192), (0, 64, 192),
        (255, 192, 128), (192, 255, 128), (128, 255, 192),
        (255, 128, 192), (128, 192, 255), (192, 128, 255),
    ]

    # 繪製每個 mask 的輪廓（只描邊，不填充）
    for i in range(num_to_show):
        contours = result["contours"][i]
        area = result["areas"][i]
        
        # 根據選項決定使用固定顏色或多彩色
        if use_fixed_color:
            color = fixed_color
        else:
            color = colors[i % len(colors)]
        
        # 只繪製輪廓，不填充
        if contours:
            contours_img = draw_contours_on_image(contours_img, contours, color=color, width=line_width)
        
        if i < 10:  # 只顯示前 10 個的詳細信息
            print(f"  Mask {i}: 面積 {area} 像素, 輪廓數量 {len(contours)}, 顏色 RGB{color}")
    
    if num_to_show > 10:
        print(f"  ... 以及其他 {num_to_show - 10} 個 masks")
    
    # 保存輪廓結果
    contours_file = out_dir / f"{prefix}_contours_only.png"
    contours_img.save(contours_file)
    print(f"\n  └─ 保存純輪廓視覺化: {contours_file}")
    print(f"  └─ 所有 {num_to_show} 個 masks 的輪廓已繪製（無填充、無邊界框）")


def add_segmentation_contours(image: Image.Image, points_per_side=32, max_masks=30, 
                               line_width=2, min_area=100, 
                               use_fixed_color=False, fixed_color=(0, 255, 0)):
    """
    在給定的圖像上添加分割輪廓（可重複使用的函數版本）
    
    Args:
        image: PIL Image 物件
        points_per_side: 每邊採樣點數
        max_masks: 最多顯示多少個 masks
        line_width: 輪廓線寬度
        min_area: 最小面積過濾（像素數）
        use_fixed_color: 是否使用固定顏色（True=所有輪廓同色，False=多色）
        fixed_color: 固定顏色的 RGB 值（當 use_fixed_color=True 時使用）
    
    Returns:
        PIL Image: 添加了輪廓的圖像
    """
    # 將 PIL Image 轉換為字節流以發送到 server
    img_byte_arr = io.BytesIO()
    image.save(img_byte_arr, format='JPEG')
    img_byte_arr.seek(0)
    
    # 發送請求
    files = {
        "image": ("temp.jpg", img_byte_arr, "image/jpeg")
    }
    
    data = {
        "points_per_side": str(points_per_side),
    }
    
    try:
        resp = requests.post(f"{SERVER_URL}/segment_auto", files=files, data=data)
        
        if resp.status_code != 200:
            print(f"⚠️ 分割請求失敗: {resp.status_code}")
            return image
        
        result = resp.json()
    except Exception as e:
        print(f"⚠️ 分割請求異常: {e}")
        return image
    
    # 創建輸出圖像
    output_img = image.copy()
    
    # 顏色列表
    colors = [
        (255, 0, 0), (0, 255, 0), (0, 0, 255),
        (255, 255, 0), (255, 0, 255), (0, 255, 255),
        (255, 128, 0), (128, 255, 0), (0, 255, 128),
        (128, 0, 255), (255, 0, 128), (0, 128, 255),
        (192, 64, 0), (64, 192, 0), (0, 192, 64),
        (192, 0, 64), (64, 0, 192), (0, 64, 192),
        (255, 192, 128), (192, 255, 128), (128, 255, 192),
        (255, 128, 192), (128, 192, 255), (192, 128, 255),
    ]
    
    # 過濾並繪製輪廓
    num_drawn = 0
    for i in range(min(max_masks, result['num_masks'])):
        area = result["areas"][i]
        
        # 面積過濾
        if area < min_area:
            continue
        
        contours = result["contours"][i]
        
        # 根據選項決定使用固定顏色或多彩色
        if use_fixed_color:
            color = fixed_color
        else:
            color = colors[num_drawn % len(colors)]
        
        # 繪製輪廓
        if contours:
            output_img = draw_contours_on_image(output_img, contours, color=color, width=line_width)
            num_drawn += 1
    
    return output_img


def segment_with_annotation(image_path: str, out_dir: Path, prefix: str,
                            points_per_side=32, line_width=2,
                            use_fixed_color=False, fixed_color=(0, 255, 0)):
    """
    案例 8: 先進行輪廓分割，然後在圖像上添加文字標註
    模擬在 putText 前做 case7 的輪廓繪製
    
    Args:
        image_path: 圖像路徑
        out_dir: 輸出目錄
        prefix: 輸出檔案前綴
        points_per_side: 每邊採樣點數
        line_width: 輪廓線寬度
        use_fixed_color: 是否使用固定顏色（True=所有輪廓同色，False=多色）
        fixed_color: 固定顏色的 RGB 值（當 use_fixed_color=True 時使用）
    """
    print(f"\n{'='*60}")
    print(f"案例: {prefix} (輪廓 + 文字標註)")
    print(f"圖像: {image_path}")
    if use_fixed_color:
        print(f"輪廓顏色: 固定顏色 RGB{fixed_color}")
    else:
        print(f"輪廓顏色: 多彩色")
    print(f"{'='*60}")
    
    # 讀取原圖
    original_img = Image.open(image_path).convert('RGB')
    
    # 步驟 1: 添加分割輪廓（類似 case7）
    print("步驟 1: 添加分割輪廓...")
    img_with_contours = add_segmentation_contours(
        original_img,
        points_per_side=points_per_side,
        max_masks=30,
        line_width=line_width,
        min_area=100,
        use_fixed_color=use_fixed_color,
        fixed_color=fixed_color
    )
    
    # 轉換為 numpy/cv2 格式以使用 cv2.putText
    import cv2
    img_array = np.array(img_with_contours)
    img_bgr = cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR)
    
    # 步驟 2: 添加文字標註（模擬您的代碼）
    print("步驟 2: 添加文字標註...")
    
    # 模擬您的變數
    frame_id = 42
    timestamp = 3.14
    gripper_state_text = "Close"
    is_keyframe = True
    
    # Different colors for keyframes vs regular frames
    if is_keyframe:
        color = (0, 0, 255)  # Red for keyframes (BGR format)
        label = "KEYFRAME"
    else:
        color = (0, 255, 255)  # Yellow for regular samples
        label = "Sample"
    
    # Add text overlay
    text_lines = [
        f"{label}",
        f"Frame ID: {frame_id}",
        f"Time: {timestamp:.2f}s",
        f"Gripper: {gripper_state_text}"
    ]
    
    y_offset = 30
    for line in text_lines:
        cv2.putText(img_bgr, line, (10, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)
        y_offset += 30
    
    # 轉回 RGB 並保存
    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    final_img = Image.fromarray(img_rgb)
    
    # 保存結果
    output_file = out_dir / f"{prefix}_annotated.png"
    final_img.save(output_file)
    print(f"✓ 保存結果: {output_file}")
    print(f"  └─ 已完成輪廓繪製 + 文字標註")


def main():
    # 準備輸出目錄
    out_dir = Path("masks_out")
    out_dir.mkdir(exist_ok=True)
    
    # 檢查測試圖像是否存在
    image_path = "test.jpg"
    if not Path(image_path).exists():
        print(f"❌ 找不到圖像: {image_path}")
        print("請確保 test.jpg 存在於當前目錄")
        return
    
    print("\n" + "="*60)
    print("SAM2 分割測試 - 多個案例示範")
    print("="*60)
    
    # 案例 1: 單個前景點
    segment_with_points(
        image_path=image_path,
        points=[[100, 200]],
        labels=[1],
        out_dir=out_dir,
        prefix="case1_single_fg",
        multimask_output=True
    )
    
    # 案例 2: 多個前景點
    segment_with_points(
        image_path=image_path,
        points=[[100, 200], [150, 250]],
        labels=[1, 1],
        out_dir=out_dir,
        prefix="case2_multi_fg",
        multimask_output=True
    )
    
    # 案例 3: 前景點 + 背景點
    segment_with_points(
        image_path=image_path,
        points=[[100, 200], [300, 400]],
        labels=[1, 0],  # 1=前景, 0=背景
        out_dir=out_dir,
        prefix="case3_fg_bg",
        multimask_output=True
    )
    
    # 案例 4: 單一 mask 輸出
    segment_with_points(
        image_path=image_path,
        points=[[200, 300]],
        labels=[1],
        out_dir=out_dir,
        prefix="case4_single_mask",
        multimask_output=False
    )
    
    # 案例 5: 自動分割整張圖像（無需點提示）
    segment_auto(
        image_path=image_path,
        out_dir=out_dir,
        prefix="case5_auto",
        points_per_side=32,  # 可調整密度
        max_masks=15  # 最多顯示 15 個 masks
    )
    
    # 案例 6: 自動分割 - 純 mask 疊加（無邊界框）
    segment_auto_overlay_only(
        image_path=image_path,
        out_dir=out_dir,
        prefix="case6_auto_overlay",
        points_per_side=32,
        max_masks=30,  # 顯示更多 masks
        alpha=0.5  # 半透明
    )
    
    # 案例 7: 自動分割 - 純輪廓描邊（不填充色塊，不顯示邊界框）- 多彩色
    segment_auto_contours_only(
        image_path=image_path,
        out_dir=out_dir,
        prefix="case7_auto_contours_multicolor",
        points_per_side=32,
        max_masks=30,  # 顯示更多 masks
        line_width=2,  # 線寬
        use_fixed_color=False  # 多彩色
    )
    
    # 案例 7b: 自動分割 - 純輪廓描邊（固定顏色）
    segment_auto_contours_only(
        image_path=image_path,
        out_dir=out_dir,
        prefix="case7b_auto_contours_fixed",
        points_per_side=32,
        max_masks=30,
        line_width=2,
        use_fixed_color=True,  # 使用固定顏色
        fixed_color=(0, 255, 0)  # 綠色
    )
    
    # 案例 8: 輪廓分割 + 文字標註（在 putText 前做輪廓繪製）- 多彩色
    segment_with_annotation(
        image_path=image_path,
        out_dir=out_dir,
        prefix="case8_contours_annotation_multicolor",
        points_per_side=32,
        line_width=2,
        use_fixed_color=False  # 多彩色
    )
    
    # 案例 8b: 輪廓分割 + 文字標註（固定顏色）
    segment_with_annotation(
        image_path=image_path,
        out_dir=out_dir,
        prefix="case8b_contours_annotation_fixed",
        points_per_side=32,
        line_width=2,
        use_fixed_color=True,  # 使用固定顏色
        fixed_color=(255, 255, 0)  # 黃色
    )
    
    print("\n" + "="*60)
    print("✓ 所有案例完成！")
    print(f"✓ 輸出目錄: {out_dir.absolute()}")
    print("="*60)

if __name__ == "__main__":
    main()