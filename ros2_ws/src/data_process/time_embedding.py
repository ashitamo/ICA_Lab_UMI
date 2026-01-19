from PIL import Image, ImageDraw, ImageFont
import pandas as pd
import os
import json
import argparse
# Usage example: python time_embedding.py demo_data6


def add_timestamp_to_photo(image_path, output_path, embed_text, font_size):
    """
    在圖片上加入時間戳文字。

    參數:
    image_path (str): 輸入圖片的路徑。
    output_path (str): 輸出圖片的路徑。
    embed_text (str): 嵌入圖片中的文字（時間戳或其他資訊）。
    font_size (int): 時間戳的字體大小。
    """
    position = (10, 10)  # 時間戳位置，可以根據需要調整
    text_color=(255, 255, 255) # 文字顏色 (白色)
    try:
        # 開啟圖片
        img = Image.open(image_path).convert("RGBA")
        
        # 建立一個可繪圖物件
        draw = ImageDraw.Draw(img)
        
        
        # --- 字體設定 ---
        # 這是您在 Ubuntu 上指定且較常存在的字體路徑
        FONT_PATH = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
        try:
            # 使用 Ubuntu 上的字體路徑和您設定的 font_size
            font = ImageFont.truetype(FONT_PATH, font_size) 
            #print(f"成功載入字體: {FONT_PATH}，大小: {font_size}")
        except IOError:
            print("警告：指定的字體檔案不存在。請檢查 FONT_PATH 是否正確，並使用預設字體。")
            font = ImageFont.load_default() # 如果字體仍載入失敗，使用預設字體作為備用   

        # 1. 測量文字大小
        # 使用 getbbox 測量文字所需的邊界框 (Bounding Box)
        # 返回 (left, top, right, bottom)
        bbox = draw.textbbox(position, embed_text, font=font)
        
        # 為了美觀，為背景增加一些邊距 (Padding)
        padding = 5 
        
        # 2. 定義背景矩形的座標
        # [x_min, y_min, x_max, y_max]
        rect_coords = [
            bbox[0] - padding,  # 左上角 X 座標
            bbox[1] - padding,  # 左上角 Y 座標
            bbox[2] + padding,  # 右下角 X 座標
            bbox[3] + padding   # 右下角 Y 座標
        ]
        
        # 3. 繪製黑色背景矩形
        # fill=bg_color 預設為 (0, 0, 0) 黑色
        bg_color=(0, 0, 0)
        draw.rectangle(rect_coords, fill=bg_color)


        # 在指定位置繪製時間戳
        draw.text(position, embed_text, font=font, fill=text_color)
        
        # 將圖片轉回 RGB 模式以便儲存為 JPG/PNG 等格式
        img = img.convert("RGB")
        
        # 儲存新的圖片
        img.save(output_path)

    except FileNotFoundError:
        print(f"錯誤: 找不到檔案 {image_path}")
    except Exception as e:
        print(f"錯誤: {e}") # 拋出實際的錯誤訊息

if __name__ == "__main__":
    # 設定資料夾路徑
    # Usage example: python time_embedding.py demo_data6
    parser = argparse.ArgumentParser(description="在圖片上加入時間戳")
    parser.add_argument("demo_folder", help="Path to demo folder (e.g., demo_data6)")
    args = parser.parse_args()
    demo_folder = args.demo_folder
    image_folder = os.path.join(demo_folder, "images")
    trajectory_path = os.path.join(demo_folder, "trajectory.json")

    # 設定輸出資料夾
    output_folder = os.path.join(demo_folder, "timestamp_images")
    if not os.path.exists(output_folder):
        os.makedirs(output_folder, exist_ok=True)
    else:
        # 刪除舊檔案
        for f in os.listdir(output_folder):
            os.remove(os.path.join(output_folder, f))

    # 載入軌跡數據
    print(f"正在載入軌跡數據： {trajectory_path}...")
    with open(trajectory_path, 'r') as file:
        trajectory_data = json.load(file)
    trajectory_df = pd.DataFrame(trajectory_data)
    total_frames = len(trajectory_df)

    print(f"正在處理 {total_frames} 個影格以加入時間戳...")
    for index, row in trajectory_df.iterrows():
        # 確保 row['id'] 和 row['timestamp'] 存在於您的 JSON 結構中
        frame_id = int(row['id'])
        timestamp = round(row['timestamp'], 2) # timestamp 變數是 numpy.float64

        # 這裡將 numpy.float64 轉換為字串，避免了 'is not iterable' 錯誤
        image_path = os.path.join(image_folder, f"frame_{frame_id}.png")
        output_image_path = os.path.join(output_folder, f"frame_{frame_id}.png")
        add_timestamp_to_photo(image_path, output_image_path, str(timestamp), font_size=30)

    print("所有圖片處理完成，帶時間戳的圖片已儲存至:", output_folder)