import cv2
import time
import base64
import os
import json
import argparse
import pandas as pd
import copy
import shutil
import re
from util.trajectory_smoothing import smooth_trajectory_with_lowpass

'''
Usage:
python3 task_manager_traject.py <demo_folder> <exe_folder>

Arguments:
- demo_folder: Path to demonstration data folder containing trajectory.json
- exe_folder: Path to execution folder containing task_segmentation.json and primitive_task_frames/

Example:
python3 task_manager_traject.py data/demo_1 exe_1

Output:
- Generates task_workflow.json in the exe_folder with enriched action parameters
'''


# --- Function Definitions ---
def load_json_plan(filepath):
    """Loads the master task plan from a JSON file."""
    try:
        with open(filepath, 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"Error: Plan file not found at {filepath}")
        return None
    except json.JSONDecodeError:
        print(f"Error: Could not decode JSON from {filepath}")
        return None


def  get_pose_at_time(pose_data, idx):
    """
    Finds the pose (position and orientation) from the loaded trajectory
    data that is closest to the given time.
    """
    # Get the data from that row 
    pose_data_at_time = pose_data.iloc[idx]
    
    coordinate = [
        round(pose_data_at_time['pos_x'], 6),
        round(pose_data_at_time['pos_y'], 6),
        round(pose_data_at_time['pos_z'], 6)
    ]

    orientation = [
        round(pose_data_at_time['quat_x'], 6),
        round(pose_data_at_time['quat_y'], 6),
        round(pose_data_at_time['quat_z'], 6),
        round(pose_data_at_time['quat_w'], 6)
    ]

    gripper_width = round(pose_data_at_time['gripper_width'], 6)
    return coordinate, orientation, gripper_width


def get_trajectory(start_time, end_time, pose_data):
    trajectory = []
    # Find the closest index to start_time and end_time
    start_idx = (pose_data['timestamp'] - start_time).abs().idxmin()
    end_idx = (pose_data['timestamp'] - end_time).abs().idxmin()    
    traj_amount = end_idx - start_idx + 1
    for idx in range(start_idx, end_idx + 1):
        coords, orien, gripper_width = get_pose_at_time(pose_data, idx)
        trajectory.append(coords + orien + [gripper_width])
    print(f"Sucessfully extracted {traj_amount} trajectory points from index {start_idx} to {end_idx}.")
    return trajectory



def get_image_path_list(start_idx, end_idx, folder_path):
    # 1. 定義正規表示式，用來解析您的檔名
    # 範例: 001_KEYFRAME_frame0027_t0.78s.png
    #       (組 1) (   組 2   ) (  組 3  ) (  組 4  ) (組 5)
    pattern = re.compile(r"^(\d+)_(.*?)_(frame\d+)_t(\d+\.\d+)s\.(jpg|png|jpeg)$")
    
    parsed_files = []
    
    # 2. 遍歷資料夾中的所有檔案
    try:
        all_files = os.listdir(folder_path)
    except FileNotFoundError:
        print(f"Error: 找不到資料夾: {folder_path}")
        return [], pd.DataFrame(), 0.0
    
    for filename in all_files:
        match = pattern.match(filename)
        
        if match:
            # 如果檔名符合我們的格式
            parts = match.groups()
            parsed_info = {
                'filename': filename,
                'full_path': os.path.join(folder_path, filename),
                'seq_id': int(parts[0]),    # 順序 ID (例如 001)
                'type': parts[1],           # 類型 (例如 KEYFRAME)
                'frame_id_str': parts[2],   # 原始幀ID (例如 frame0027)
                'timestamp': float(parts[3]) # 時間戳 (例如 0.78)
            }
            parsed_files.append(parsed_info)
        else:
            print(f"  > 略過不符格式的檔案: {filename}")

    # 3. 根據「順序 ID (seq_id)」對所有找到的檔案進行排序
    #    這是確保 VLM 看到正確順序的關鍵步驟
    if not parsed_files:
        print("錯誤：在此資料夾中沒有找到任何符合格式的圖片。")
        return [], pd.DataFrame(), 0.0
        
    parsed_files.sort(key=lambda x: x['seq_id'])
    
    print(f"成功解析並排序 {len(parsed_files)} 張圖片。\n")

    # 4. 讀取圖片，轉換為 Base64，並建立模擬的 trajectory_df
    image_path_list = []
    
    for file_info in parsed_files:
        # 只選取在指定索引範圍內的圖片
        if file_info['seq_id'] < start_idx or file_info['seq_id'] > end_idx:
            continue
        else:
            image_path = file_info['full_path']
            image_path_list.append(image_path)
    return image_path_list




# --- Main Execution ---
if __name__ == "__main__":
    print("Welcome to the Task Manager!")

    parser = argparse.ArgumentParser(description="Replan a robotic task based on an image.")
    parser.add_argument("demo_folder", type=str, help="Path to the data folder.")
    parser.add_argument("exe_folder", type=str, help="Path to the exe folder.")
    args = parser.parse_args()

    # 1. Load the original workflow
    demo_folder_path = args.demo_folder #data/demo_1
    try:
        with open(demo_folder_path + "/trajectory.json", "r") as f:
            pose_data = json.load(f)
        pose_data = pd.DataFrame(pose_data)   # ✅ convert to DataFrame
        print(f"Perception System Initialized Ready for commands.")
    except Exception as e:
        print(f"Error loading pose data: {e}")
        pose_data = None
        exit() # Exit if we can't load pose data
    

    exe_folder_path = args.exe_folder #exe_1
    task_segmentation_path = os.path.join(exe_folder_path, "task_segmentation.json")
    master_plan = load_json_plan(task_segmentation_path)
    if not master_plan:
        print("Failed to load plan (empty or None).")
        exit()
    task_workflow = copy.deepcopy(master_plan)
    
    print("Demo data folder:", demo_folder_path)
    # --- RENAMED for clarity ---
    primitive_task_frames_folder = os.path.join(exe_folder_path, "primitive_task_frames")
    print("Demo primitive task frames folder:", primitive_task_frames_folder)


    output_folder = exe_folder_path
    output_dir = os.path.join(output_folder, "task_workflow.json")

    
    primitive_tasks_list = task_workflow.get("primitive_tasks", [])
    if not primitive_tasks_list:
        print("Warning: No primitive tasks found in the plan.")
    
    # 2. Start planning the workflow
    print("----------\nStarting planing workflow\n----------")
    for task in primitive_tasks_list:
        # Access information from the primitive task dictionary
        task_id = task.get('task_id')
        task_name = task.get('primitive_tasks_name')
        
        print(f"Starting Primitive Task ID: {task_id}\nName: {task_name}\n----------")
        
        # Access the list of atomic actions for the current primitive task
        atomic_actions_list = task.get('atomic_actions', [])
        print(f"Found {len(atomic_actions_list)} atomic actions to process.")
        

        for action_step in atomic_actions_list:
            start_time = action_step.get("start_time")
            end_time = action_step.get("end_time")
            keyframe_start_idx = action_step.get("start_idx")
            keyframe_end_idx = action_step.get("end_idx")
            duration = end_time - start_time
            print(f"Processing action step: {action_step.get('action')}")
            try:
                # --- *** 修正：確保 "parameters" 字典存在 *** ---
                if "parameters" not in action_step:
                    action_step["parameters"] = {}

                # 3. 使用計算出的索引來獲取軌跡
                trajectory = get_trajectory(start_time, end_time, pose_data)
                smooth_trajectory = smooth_trajectory_with_lowpass(trajectory, duration, cutoff=3.0, order=4)
                #smooth_trajectory = trajectory_smoothing(trajectory)
                action_step["parameters"]["trajectory"] = smooth_trajectory
                
                # 4. 使用計算出的索引來獲取影像路徑
                image_path_list = get_image_path_list(keyframe_start_idx, keyframe_end_idx, primitive_task_frames_folder)
                # --- *** 修正：將 "image_path" 移入 "parameters" *** ---
                action_step["parameters"]["image_path"] = image_path_list

                
            except Exception as e:
                action_name = action_step.get("action", "Unknown")
                print(f"  Error enriching action '{action_name}': {e}. Skipping.")
                print()


    # 3. Save the enriched workflow
    try:
        # --- 修正 1：加入 encoding='utf-8' ---
        with open(output_dir, 'w', encoding='utf-8') as f:
            # --- 修正 2：加入 ensure_ascii=False ---
            json.dump(task_workflow, f, indent=2, ensure_ascii=False)
        print(f"\n--- Enrichment Complete ---")
        print(f"Successfully saved task_workflow to: " + output_dir)
    except Exception as e:
        print(f"Error saving the output file: {e}")