import cv2
import base64
import os
from openai import OpenAI
import numpy as np
import pandas as pd
import json
import argparse
from typing import List, Dict, Any, Optional, Tuple
from dataclasses import dataclass
from concurrent.futures import ThreadPoolExecutor, as_completed
import threading

'''
--- Configuration ---
It's highly recommended to set your API key as an environment variable
for security purposes. For example:
Usage:
python3 task_segmentation.py <demo_folder> <exe_folder>
python3 task_segmentation.py demo_data1 exe_data1
'''
API_KEY = os.getenv("OPENAI_API_KEY")
if not API_KEY:
    raise ValueError("OPENAI_API_KEY environment variable not set.")

# Initialize the OpenAI client
client = OpenAI(api_key=API_KEY)

# Load prompts from external JSON file
def load_prompts(prompts_file="./prompts.json"):
    """Load prompts from a JSON file."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    prompts_path = os.path.join(script_dir, prompts_file)
    
    if not os.path.exists(prompts_path):
        raise FileNotFoundError(f"Prompts file not found at: {prompts_path}")
    
    with open(prompts_path, 'r', encoding='utf-8') as f:
        return json.load(f)

# Load prompts at module level
PROMPTS = load_prompts()


# ============================================================
# Data Classes for Phase 3 (Atomic Action Extraction)
# ============================================================

@dataclass
class Segment:
    """Keyframe 之間的片段"""
    segment_id: int
    start_idx: int  # 在 frames 中的起始索引
    end_idx: int    # 在 frames 中的結束索引
    start_time: float
    end_time: float
    frames: List[str]  # base64 encoded frames
    action: Optional[str] = None
    description: Optional[str] = None
    confidence: Optional[str] = None  # high, medium, low
    standalone_description: Optional[str] = None  # Phase 1 獨立分析的描述
    standalone_action: Optional[str] = None  # Phase 1 獨立分析的動作
    
    @property
    def duration(self) -> float:
        return self.end_time - self.start_time
    
    @property
    def num_frames(self) -> int:
        return len(self.frames)


# ============================================================
# Segment Extraction
# ============================================================

def extract_segments_for_primitive_task(
    pri_task: Dict[str, Any],
    frames: List[str],
    sampled_times: List[float],
    is_keyframe_flags: List[bool]
) -> List[Segment]:
    """為單個 primitive task 提取 keyframe 之間的片段"""
    
    pt_start = float(pri_task['start_time'])
    pt_end = float(pri_task['end_time'])
    
    # 找出此 primitive task 範圍內的所有 keyframe
    keyframe_indices = []
    last_kf_before_start = None
    
    for i, (time, is_kf) in enumerate(zip(sampled_times, is_keyframe_flags)):
        if not is_kf:
            continue
            
        if time < pt_start:
            last_kf_before_start = i
        elif pt_start <= time <= pt_end:
            keyframe_indices.append(i)
    
    # 如果有在 pt_start 之前的 keyframe，且它離 pt_start 很近（< 2 秒），則包含它
    if last_kf_before_start is not None:
        time_diff = pt_start - sampled_times[last_kf_before_start]
        if time_diff < 2.0:
            keyframe_indices.insert(0, last_kf_before_start)
    
    if len(keyframe_indices) < 2:
        print(f"    ⚠ Less than 2 keyframes in primitive task range")
        print(f"       Found keyframes at indices: {keyframe_indices}")
        return []
    
    print(f"    Found {len(keyframe_indices)} keyframes:")
    for idx in keyframe_indices:
        kf_time = sampled_times[idx]
        print(f"      - {kf_time:.2f}s (index {idx})")
    
    # 建立片段
    segments = []
    for seg_id, (start_idx, end_idx) in enumerate(zip(keyframe_indices[:-1], keyframe_indices[1:])):
        start_time = sampled_times[start_idx]
        end_time = sampled_times[end_idx]
        
        # 取得這個片段的所有 frames（包含起始和結束 keyframe）
        segment_frames = frames[start_idx:end_idx+1]
        
        segment = Segment(
            segment_id=seg_id,
            start_idx=start_idx,
            end_idx=end_idx,
            start_time=start_time,
            end_time=end_time,
            frames=segment_frames
        )
        
        segments.append(segment)
        print(f"      Segment {seg_id}: {start_time:.2f}s - {end_time:.2f}s ({len(segment_frames)} frames)")
    
    return segments


# ============================================================
# Segment Analysis Functions
# ============================================================

def analyze_segment_standalone(
    segment: Segment,
    pri_task_info: Dict[str, Any]
) -> Optional[Dict[str, Any]]:
    """使用 GPT 分析單個片段（獨立分析，不考慮相鄰片段）"""
    
    prompt_config = PROMPTS.get("segment_analysis_standalone")
    if not prompt_config:
        # 如果沒有專用的 standalone prompt，使用原本的 segment_analysis
        prompt_config = PROMPTS.get("segment_analysis")
        if not prompt_config:
            print("    ⚠ 'segment_analysis' prompt not found in prompts.json")
            return None
    
    # 構建提示內容，只包含當前片段和 primitive task 背景資訊
    user_text = prompt_config["user_prompt_template"].format(
        primitive_task_name=pri_task_info.get("primitive_tasks_name", "Unknown task"),
        primitive_task_start=pri_task_info.get("start_time", 0.0),
        primitive_task_end=pri_task_info.get("end_time", 0.0),
        previous_description="（單獨分析此片段）",
        start_time=segment.start_time,
        end_time=segment.end_time,
        duration=segment.duration,
        num_frames=segment.num_frames
    )
    
    user_content = [{"type": "text", "text": user_text}]
    user_content.extend([
        {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{frame}"}}
        for frame in segment.frames
    ])
    
    system_content = (
        prompt_config["system_prompt"] + 
        "\n**Example output:**\n" + 
        json.dumps(prompt_config["example_output"], indent=2, ensure_ascii=False)
    )
    
    messages = [
        {"role": "system", "content": system_content},
        {"role": "user", "content": user_content}
    ]
    
    try:
        response = client.chat.completions.create(
            model="gpt-5-mini",
            messages=messages,
            response_format={"type": "json_object"}
        )
        json_response = response.choices[0].message.content
        result = json.loads(json_response)
        return result
    
    except Exception as e:
        print(f"      ✗ API Error: {e}")
        return None


def analyze_segment_with_context(
    segment: Segment,
    pri_task_info: Dict[str, Any],
    prev_segment: Optional[Segment] = None,
    next_segment: Optional[Segment] = None
) -> Optional[Dict[str, Any]]:
    """使用 GPT 分析片段（考慮相鄰片段的上下文）"""
    
    prompt_config = PROMPTS.get("segment_analysis_with_context")
    if not prompt_config:
        prompt_config = PROMPTS.get("segment_analysis")
        if not prompt_config:
            print("    ⚠ 'segment_analysis' prompt not found in prompts.json")
            return None
    
    # 構建上下文描述
    prev_desc = prev_segment.description if prev_segment and prev_segment.description else "（這是第一個片段）"
    next_desc = next_segment.description if next_segment and next_segment.description else "（這是最後一個片段）"
    
    # 構建提示內容
    user_text = prompt_config["user_prompt_template"].format(
        primitive_task_name=pri_task_info.get("primitive_tasks_name", "Unknown task"),
        primitive_task_start=pri_task_info.get("start_time", 0.0),
        primitive_task_end=pri_task_info.get("end_time", 0.0),
        previous_description=prev_desc,
        next_description=next_desc,
        start_time=segment.start_time,
        end_time=segment.end_time,
        duration=segment.duration,
        num_frames=segment.num_frames
    )
    
    user_content = [{"type": "text", "text": user_text}]
    user_content.extend([
        {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{frame}"}}
        for frame in segment.frames
    ])
    
    system_content = (
        prompt_config["system_prompt"] + 
        "\n**Example output:**\n" + 
        json.dumps(prompt_config["example_output"], indent=2, ensure_ascii=False)
    )
    
    messages = [
        {"role": "system", "content": system_content},
        {"role": "user", "content": user_content}
    ]
    
    try:
        response = client.chat.completions.create(
            model="gpt-5-mini",
            messages=messages,
            response_format={"type": "json_object"}
        )
        json_response = response.choices[0].message.content
        result = json.loads(json_response)
        return result
    
    except Exception as e:
        print(f"      ✗ API Error: {e}")
        return None


def label_segment_action(segment: Segment) -> Optional[Dict[str, Any]]:
    """為片段標註 atomic action（使用描述和影像來判斷）"""
    
    prompt_config = PROMPTS.get("segment_labeling")
    if not prompt_config:
        print("    ⚠ 'segment_labeling' prompt not found in prompts.json")
        return None
    
    # 構建提示內容
    user_text = prompt_config["user_prompt_template"].format(
        description=segment.description,
        start_time=segment.start_time,
        end_time=segment.end_time,
        duration=segment.duration
    )
    
    user_content = [{"type": "text", "text": user_text}]
    
    # 添加影像：使用片段的 frames（已經是 base64 編碼）
    if segment.frames:
        for frame in segment.frames:
            user_content.append({
                "type": "image_url",
                "image_url": {
                    "url": f"data:image/jpeg;base64,{frame}",
                    "detail": "low"
                }
            })
    else:
        print(f"      ⚠ Warning: Segment {segment.segment_id} has no frames")
    
    system_content = (
        prompt_config["system_prompt"] + 
        "\n**Example output:**\n" + 
        json.dumps(prompt_config["example_output"], indent=2, ensure_ascii=False)
    )
    
    messages = [
        {"role": "system", "content": system_content},
        {"role": "user", "content": user_content}
    ]
    
    try:
        print(f"      → Labeling segment {segment.segment_id}...")
        response = client.chat.completions.create(
            model="gpt-4o-mini",
            messages=messages,
            response_format={"type": "json_object"}
        )
        json_response = response.choices[0].message.content
        result = json.loads(json_response)
        print(f"      ✓ Label: {result.get('action')}")
        return result
    
    except Exception as e:
        print(f"      ✗ API Error: {e}")
        return None


# ============================================================
# Multi-threaded Processing
# ============================================================

def analyze_single_segment_phase2(
    args: Tuple[Segment, Dict[str, Any], Optional[Segment], Optional[Segment]]
) -> Tuple[Segment, bool]:
    """第二階段：結合相鄰片段分析（用於多線程）"""
    
    segment, pri_task_info, prev_segment, next_segment = args
    
    analysis = analyze_segment_with_context(segment, pri_task_info, prev_segment, next_segment)
    
    if analysis:
        segment.description = analysis.get("description", "")
        segment.action = None
        segment.confidence = None
        return segment, True
    else:
        return segment, False


def label_single_segment(segment: Segment) -> Tuple[Segment, bool]:
    """標註單個片段（用於多線程）"""
    
    label_result = label_segment_action(segment)
    
    if label_result:
        segment.action = label_result.get("action")
        segment.confidence = label_result.get("confidence", "medium")
        return segment, True
    else:
        segment.action = "Unknown"
        segment.confidence = "low"
        return segment, False


def analyze_all_segments(
    task_data: Dict[str, Any],
    frames: List[str],
    sampled_times: List[float],
    is_keyframe_flags: List[bool],
    output_folder: str,
    max_workers: int = 16
) -> Dict[int, List[Segment]]:
    """
    階段1：分析所有片段（兩階段處理）
    Phase 1: 使用 segment_analysis_standalone 獨立分析
    Phase 2: 使用 segment_analysis_with_context 結合上下文重新分析
    """
    
    print("\n" + "="*60)
    print("Stage 1: Analyzing Keyframe Segments (Two-Phase Analysis)")
    print("="*60)
    
    all_segments = {}
    
    # 首先提取所有片段
    for pri_task in task_data.get("primitive_tasks", []):
        task_id = pri_task['task_id']
        print(f"\n[Primitive Task {task_id}]")
        
        segments = extract_segments_for_primitive_task(
            pri_task, frames, sampled_times, is_keyframe_flags
        )
        segments[0].frames
        
        if not segments:
            continue
        
        all_segments[task_id] = segments
    
    if not all_segments:
        print("\n⚠ No segments to analyze")
        return all_segments
    
    # ========== Phase 1: Standalone Analysis ==========
    print("\n" + "-"*60)
    print("Phase 1: Standalone Analysis (獨立分析每個片段)")
    print("-"*60)
    
    phase1_tasks = []
    for task_id, segments in all_segments.items():
        # 找回對應的 pri_task_info
        pri_task_info = None
        for pri_task in task_data.get("primitive_tasks", []):
            if pri_task['task_id'] == task_id:
                pri_task_info = {
                    "task_id": pri_task.get("task_id"),
                    "primitive_tasks_name": pri_task.get("primitive_tasks_name"),
                    "start_time": pri_task.get("start_time"),
                    "end_time": pri_task.get("end_time")
                }
                break
        
        if not pri_task_info:
            continue
        
        for segment in segments:
            phase1_tasks.append((segment, pri_task_info))
    
    def analyze_standalone_wrapper(args):
        segment, pri_task_info = args
        result = analyze_segment_standalone(segment, pri_task_info)
        if result:
            # 保存 standalone 分析結果
            segment.standalone_description = result.get("description", "")
            segment.standalone_action = result.get("atomic_action", "")
            # 暫時也設定到主要欄位（Phase 2 會覆寫 description）
            segment.description = result.get("description", "")
            segment.atomic_action = result.get("atomic_action", "")
            return segment, True
        return segment, False
    
    print(f"\nAnalyzing {len(phase1_tasks)} segments (standalone)...")
    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        futures = [executor.submit(analyze_standalone_wrapper, task) for task in phase1_tasks]
        
        completed = 0
        for future in as_completed(futures):
            segment, success = future.result()
            completed += 1
            if success:
                print(f"  [{completed}/{len(phase1_tasks)}] ✓ Segment {segment.segment_id}")
            else:
                print(f"  [{completed}/{len(phase1_tasks)}] ✗ Segment {segment.segment_id}: Analysis failed")
    
    # 保存 Phase 1 Standalone 結果
    print("\n  💾 Saving Phase 1 standalone results...")
    save_segments_to_json(all_segments, output_folder, "stage1_standalone")
    
    # ========== Phase 2: Context-aware Analysis ==========
    print("\n" + "-"*60)
    print("Phase 2: Context-aware Analysis (結合上下文重新分析)")
    print("-"*60)
    
    phase2_tasks = []
    for task_id, segments in all_segments.items():
        # 找回對應的 pri_task_info
        pri_task_info = None
        for pri_task in task_data.get("primitive_tasks", []):
            if pri_task['task_id'] == task_id:
                pri_task_info = {
                    "task_id": pri_task.get("task_id"),
                    "primitive_tasks_name": pri_task.get("primitive_tasks_name"),
                    "start_time": pri_task.get("start_time"),
                    "end_time": pri_task.get("end_time")
                }
                break
        
        if not pri_task_info:
            continue
        
        for i, segment in enumerate(segments):
            prev_segment = segments[i-1] if i > 0 else None
            next_segment = segments[i+1] if i < len(segments) - 1 else None
            phase2_tasks.append((segment, pri_task_info, prev_segment, next_segment))
    
    print(f"\nAnalyzing {len(phase2_tasks)} segments (with context)...")
    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        futures = [executor.submit(analyze_single_segment_phase2, task) for task in phase2_tasks]
        
        completed = 0
        for future in as_completed(futures):
            segment, success = future.result()
            completed += 1
            if success:
                print(f"  [{completed}/{len(phase2_tasks)}] ✓ Segment {segment.segment_id}: {segment.description[:60]}...")
            else:
                print(f"  [{completed}/{len(phase2_tasks)}] ✗ Segment {segment.segment_id}: Analysis failed")
    
    return all_segments


def label_all_segments(all_segments: Dict[int, List[Segment]], max_workers: int = 16) -> None:
    """階段2：為所有片段標註 action（多線程）"""
    
    print("\n" + "="*60)
    print("Stage 2: Labeling Segments")
    print(f"Using {max_workers} worker threads")
    print("="*60)
    
    print_lock = threading.Lock()
    
    for task_id, segments in all_segments.items():
        print(f"\n[Primitive Task {task_id}]")
        print(f"  Labeling {len(segments)} segments with {max_workers} threads:")
        
        with ThreadPoolExecutor(max_workers=max_workers) as executor:
            future_to_segment = {
                executor.submit(label_single_segment, segment): segment 
                for segment in segments
            }
            
            completed_count = 0
            for future in as_completed(future_to_segment):
                segment, success = future.result()
                completed_count += 1
                
                with print_lock:
                    if success:
                        print(f"    [{completed_count}/{len(segments)}] Segment {segment.segment_id}: {segment.action} (confidence: {segment.confidence})")
                    else:
                        print(f"    [{completed_count}/{len(segments)}] Segment {segment.segment_id}: ⚠ Labeling failed")


# ============================================================
# Merging Functions
# ============================================================

def should_merge_same_actions(seg1: Segment, seg2: Segment) -> bool:
    """判斷兩個片段是否應該合併（基於動作類型）"""
    if not seg1.action or not seg2.action:
        return False
    
    if seg1.action == "Unknown" or seg2.action == "Unknown":
        return False
    
    return seg1.action == seg2.action


def merge_consecutive_segments(segments: List[Segment]) -> List[Segment]:
    """合併連續的相同動作片段"""
    
    if len(segments) <= 1:
        return segments
    
    print(f"\n  Merging consecutive same actions...")
    print(f"    Initial segments: {len(segments)}")
    
    merged = []
    i = 0
    
    while i < len(segments):
        current = segments[i]
        
        # 找出所有連續的相同動作
        consecutive_count = 1
        while (i + consecutive_count < len(segments) and 
               should_merge_same_actions(current, segments[i + consecutive_count])):
            consecutive_count += 1
        
        # 如果有連續的相同動作，合併它們
        if consecutive_count > 1:
            last_seg = segments[i + consecutive_count - 1]
            
            # 收集所有要合併的片段的描述
            descriptions = [segments[i + j].description for j in range(consecutive_count)]
            combined_description = " → ".join(descriptions)
            
            # 收集所有影格
            all_frames = []
            for j in range(consecutive_count):
                if j == 0:
                    all_frames.extend(segments[i + j].frames)
                else:
                    all_frames.extend(segments[i + j].frames[1:])
            
            # 建立合併後的片段
            merged_seg = Segment(
                segment_id=len(merged),
                start_idx=current.start_idx,
                end_idx=last_seg.end_idx,
                start_time=current.start_time,
                end_time=last_seg.end_time,
                frames=all_frames,
                action=current.action,
                description=combined_description,
                confidence=current.confidence
            )
            
            segment_ids = [segments[i + j].segment_id for j in range(consecutive_count)]
            print(f"      Merged {consecutive_count} '{current.action}' segments: {segment_ids}")
            merged.append(merged_seg)
            i += consecutive_count
        else:
            current.segment_id = len(merged)
            merged.append(current)
            i += 1
    
    print(f"    Final segments: {len(merged)}")
    return merged


def merge_all_same_actions(all_segments: Dict[int, List[Segment]]) -> Dict[int, List[Segment]]:
    """階段3：合併所有 primitive task 的連續相同動作"""
    
    print("\n" + "="*60)
    print("Stage 3: Merging Consecutive Same Actions")
    print("="*60)
    
    merged_segments = {}
    
    for task_id, segments in all_segments.items():
        print(f"\n[Primitive Task {task_id}]")
        merged = merge_consecutive_segments(segments)
        merged_segments[task_id] = merged
    
    return merged_segments


# ============================================================
# Result Conversion & Saving
# ============================================================

def save_segments_to_json(
    all_segments: Dict[int, List[Segment]],
    output_folder: str,
    stage_name: str
) -> None:
    """保存片段分析結果到 JSON"""
    
    segments_data = {}
    
    for task_id, segments in all_segments.items():
        segments_data[f"task_{task_id}"] = [
            {
                "segment_id": seg.segment_id,
                "start_time": seg.start_time,
                "end_time": seg.end_time,
                "duration": seg.duration,
                "num_frames": seg.num_frames,
                "action": seg.action,
                "description": seg.description,
                "confidence": seg.confidence,
                "standalone_description": seg.standalone_description,
                "standalone_action": seg.standalone_action
            }
            for seg in segments
        ]
    
    output_path = os.path.join(output_folder, f"segments_{stage_name}.json")
    
    with open(output_path, "w", encoding='utf-8') as f:
        json.dump(segments_data, f, indent=4, ensure_ascii=False)
    
    print(f"    💾 Saved: {output_path}")


def load_segments_from_json(
    output_folder: str,
    stage_name: str,
    frames: List[str],
    sampled_times: List[float]
) -> Dict[int, List[Segment]]:
    """從 JSON 載入片段分析結果"""
    
    input_path = os.path.join(output_folder, f"segments_{stage_name}.json")
    
    if not os.path.exists(input_path):
        raise FileNotFoundError(f"Stage file not found: {input_path}")
    
    with open(input_path, "r", encoding='utf-8') as f:
        segments_data = json.load(f)
    
    all_segments = {}
    
    for task_key, seg_list in segments_data.items():
        # 從 "task_1" 提取出 task_id
        task_id = int(task_key.split("_")[1])
        
        segments = []
        for seg_dict in seg_list:
            start_time = seg_dict["start_time"]
            end_time = seg_dict["end_time"]
            
            # 如果有提供 frames 和 sampled_times，則提取對應的 frames
            # 否則使用空列表（resume mode）
            segment_frames = []
            start_idx = 0
            end_idx = 0
            
            if frames and sampled_times:
                # 找出時間範圍內的影格索引
                frame_indices = [
                    i for i, t in enumerate(sampled_times)
                    if start_time <= t <= end_time
                ]
                
                if not frame_indices:
                    print(f"    ⚠ No frames found for segment {seg_dict['segment_id']} in time range [{start_time:.2f}, {end_time:.2f}]")
                else:
                    start_idx = frame_indices[0]
                    end_idx = frame_indices[-1]
                    # 提取對應的 frames
                    segment_frames = [frames[i] for i in frame_indices]
            
            # 重建 Segment 物件
            segment = Segment(
                segment_id=seg_dict["segment_id"],
                start_idx=start_idx,
                end_idx=end_idx,
                start_time=start_time,
                end_time=end_time,
                frames=segment_frames,
                action=seg_dict.get("action"),
                description=seg_dict.get("description"),
                confidence=seg_dict.get("confidence"),
                standalone_description=seg_dict.get("standalone_description"),
                standalone_action=seg_dict.get("standalone_action")
            )
            segments.append(segment)
        
        all_segments[task_id] = segments
    
    print(f"    ✓ Loaded {len(all_segments)} task(s) from: {input_path}")
    return all_segments


# ============================================================
# Original Functions
# ============================================================
# It returns a list of base64-encoded images and the duration of the video.
def process_image_folder(folder_path, trajectory_path, filename_format="color_{id}.png", fps=5.0):

    if not os.path.isdir(folder_path):
        raise FileNotFoundError(f"Image folder not found at: {folder_path}")
    if not os.path.exists(trajectory_path):
        raise FileNotFoundError(f"Trajectory file not found at: {trajectory_path}")

    # --- 1. Load Trajectory Data ---
    print(f"Loading trajectory data from {trajectory_path}...")
    with open(trajectory_path, 'r') as file:
        trajectory_data = json.load(file)
               
    # Convert to DataFrame
    trajectory_df = pd.DataFrame(trajectory_data)
    #print(trajectory_df.head())  # Verify DataFrame structure

    total_frames = len(trajectory_df)
    video_duration = trajectory_df['timestamp'].iloc[-1]
    print(f"Found {total_frames} total frames in trajectory, with a duration of {video_duration:.2f} seconds.")
    
    # --- 2. Intelligent Frame Sampling based on Time ---
    # Create a series of evenly spaced timestamps to sample
    target_timestamps = np.arange(0, video_duration, 1.0/fps)
    
    print(target_timestamps)
    base64_frames = []
    # Use a set to avoid processing the same frame twice if timestamps are very close
    processed_indices = set()

    for ts in target_timestamps:
        # For each target time, find the row in the DataFrame with the closest timestamp
        closest_row_index = (trajectory_df['timestamp'] - ts).abs().idxmin()
        
        if closest_row_index in processed_indices:
            continue
        
        processed_indices.add(closest_row_index)
        
        closest_entry = trajectory_df.iloc[closest_row_index]
        frame_id = int(closest_entry['id']) 
        
        # Construct the image path using the ID from the trajectory data
        image_path = os.path.join(folder_path, filename_format.format(id=frame_id))
        
        try:
            frame = cv2.imread(image_path)
            if frame is None:
                print(f"Warning: Could not read image file {image_path}")
                continue
            _, buffer = cv2.imencode(".jpg", frame)
            base64_frames.append(base64.b64encode(buffer).decode("utf-8"))
        except Exception as e:
            print(f"Error processing frame {image_path}: {e}")

    print(f"Selected and processed {len(base64_frames)} frames for VLM analysis.")
    return base64_frames, video_duration


# This function sends the extracted frames to the OpenAI VLM and requests task decomposition.
# It returns the JSON response from the model.
def primitive_task_decomposition(base64_frames, video_duration):
    # Load prompt configuration
    prompt_config = PROMPTS["primitive_task_decomposition"]
    
    # Format user prompt with variables
    user_text = prompt_config["user_prompt_template"].format(duration=video_duration)
    
    user_content = [{"type": "text", "text": user_text}]
    # 將所有圖片加到提示的後面
    user_content.extend([{"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{frame}"}} for frame in base64_frames])    
    
    # Build system prompt with example
    system_content = prompt_config["system_prompt"] + "\nExample output:" + json.dumps(prompt_config["example_output"], indent=2, ensure_ascii=False)
    
    prompt_messages = [
        {
            "role": "system",
            "content": system_content
        },
        {
            "role": "user",
            "content": user_content,
        },
    ]

    print("Sending request to OpenAI API. This may take a moment...")
    try:
        response = client.chat.completions.create(
            model="gpt-5-mini",
            messages=prompt_messages,
            # max_tokens=2000,
            response_format={"type": "json_object"} # Enforce JSON output
        )
        json_response = response.choices[0].message.content
        print("Successfully received response from API.")
        return json_response
    except Exception as e:
        print(f"An error occurred while communicating with the OpenAI API: {e}")
        return None
    


def atomic_action_decomposition(picked_frames, start_time, end_time):
    primitive_task_duration = end_time - start_time
    
    # Load prompt configuration
    prompt_config = PROMPTS["atomic_action_decomposition"]
    
    # Format user prompt with variables
    user_text = prompt_config["user_prompt_template"].format(duration=primitive_task_duration)
    
    user_content = [{"type": "text", "text": user_text}]
    user_content.extend([{"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{frame}"}} for frame in picked_frames])    

    # Build system prompt with example
    system_content = prompt_config["system_prompt"] + "\n**Example output:**\n" + json.dumps(prompt_config["example_output"], indent=2, ensure_ascii=False)

    prompt_messages = [
        {"role": "system", "content": system_content},
        {"role": "user", "content": user_content}
    ]  

    print("Sending request to OpenAI API. This may take a moment...")
    try:
        response = client.chat.completions.create(
            model="gpt-5-mini",
            messages=prompt_messages,
            # max_tokens=2000,
            response_format={"type": "json_object"} # Enforce JSON output
        )
        json_response = response.choices[0].message.content
        print("Successfully received response from API.")
        return json_response
    except Exception as e:
        print(f"An error occurred while communicating with the OpenAI API: {e}")
        return None



def get_atomic_action_timestamps(picked_frames, start_time, end_time, action_type, description):
    primitive_task_duration = end_time - start_time
    
    # Load prompt configuration
    prompt_config = PROMPTS["atomic_action_timestamps"]
    
    # Format user prompt with variables
    user_text = prompt_config["user_prompt_template"].format(action_type=action_type, description=description)
    
    user_content = [{"type": "text", "text": user_text}]
    user_content.extend([{"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{frame}"}} for frame in picked_frames])    

    # Build system prompt with example
    system_content = prompt_config["system_prompt"] + "\n**Example output:**\n" + json.dumps(prompt_config["example_output"], indent=2, ensure_ascii=False)

    prompt_messages = [
        {"role": "system", "content": system_content},
        {"role": "user", "content": user_content}
    ]  

    print("Sending request to OpenAI API. This may take a moment...")
    try:
        response = client.chat.completions.create(
            model="gpt-5-mini",
            messages=prompt_messages,
            # max_tokens=2000,
            response_format={"type": "json_object"} # Enforce JSON output
        )
        json_response = response.choices[0].message.content
        print("Successfully received response from API.")
        return json_response
    except Exception as e:
        print(f"An error occurred while communicating with the OpenAI API: {e}")
        return None
    

def main(args):
    print("="*60)
    print("Task Segmentation Pipeline")
    print("="*60)
    print(f"Demo folder:     {args.demo_folder}")
    print(f"Image folder:    {image_folder}")
    print(f"Trajectory path: {trajectory_path}")
    print(f"Output folder:   {args.output_folder}")
    print("="*60)

    from util.data_loader import DemonstrationDataLoader
    from util.frame_extraction import extract_keyframes_with_adaptive_sampling
    
    # 檢查是否從特定 stage 重入
    start_from_stage = args.start_from_stage
    
    # 如果指定從 stage 重入，則只載入必要的資料，跳過前面的步驟
    if start_from_stage:
        print(f"\n⚡ Resume Mode: Starting from {start_from_stage}")
        print("  Loading minimal data required for resuming...")
        
        # 載入 loader (GIF 生成需要)
        loader = DemonstrationDataLoader(
            image_folder=image_folder,
            trajectory_json_path=trajectory_path,
            filename_format=args.filename_format
        )
        loader.load_all_data()
        # 載入 task_segmentation.json (如果存在)
        task_seg_path = os.path.join(args.output_folder, "task_segmentation.json")
        if os.path.exists(task_seg_path):
            with open(task_seg_path, 'r') as f:
                decomposed_task_data = json.load(f)
            print(f"  ✓ Loaded task_segmentation.json")
        else:
            print(f"  ⚠ task_segmentation.json not found, will create empty structure")
            decomposed_task_data = {"primitive_tasks": []}
        
        # 載入已保存的 sampling info（如果有的話）
        # 這樣可以重建 frames, sampled_times, is_keyframe_flags
        frames = []
        sampled_times = []
        is_keyframe_flags = []
        
        # 嘗試從最後保存的 stage 文件中提取資訊
        # 我們需要這些資料來載入 segments
        print("  Note: Frame data will be loaded when needed from stage files")
        
    else:
        # ============================================================
        # Step 1: 載入資料並提取 Keyframes
        # ============================================================
        print("\n[Step 1: Loading Data & Extracting Keyframes]")
        
        # Load trajectory data
        loader = DemonstrationDataLoader(
            image_folder=image_folder,
            trajectory_json_path=trajectory_path,
            filename_format=args.filename_format
        )
        loader.load_all_data()
        
        # Extract keyframes with adaptive sampling
        sampled_frames, sampled_indices, is_keyframe_flags, sampled_frames_annotated = extract_keyframes_with_adaptive_sampling(
            loader=loader,
            base_samples=0.5,      # Minimum sampling rate (Hz)
            max_samples=5,         # Maximum sampling rate (Hz)
            velocity_scale=25,     # Velocity scaling factor
            local_minima_order=5,  # Keyframe detection sensitivity
            save_images=True,      # Save annotated images
            output_dir=os.path.join(args.output_folder, "primitive_task_frames"),
            save_plot=True,        # Save visualization plot
            plot_path=os.path.join(args.output_folder, "primitive_task_sampling.png")
        )
        
        # Convert annotated frames to base64
        frames = []
        sampled_times = []
        frame_timestamps = [frame.timestamp for frame in sampled_frames]
        
        print("  Converting frames to base64...")
        for i, (img_annotated, frame) in enumerate(zip(sampled_frames_annotated, sampled_frames)):
            _, buffer = cv2.imencode(".jpg", img_annotated)
            frames.append(base64.b64encode(buffer).decode("utf-8"))
            sampled_times.append(frame.timestamp)
        
        duration = loader.frames[-1].timestamp
        
        print(f"  ✓ Total frames: {len(frames)}")
        print(f"  ✓ Keyframes: {sum(is_keyframe_flags)}")
        print(f"  ✓ Adaptive samples: {len(frames) - sum(is_keyframe_flags)}")
        print(f"  ✓ Duration: {duration:.2f}s")

        if not frames:
            print("✗ No frames were extracted. Exiting.")
            return
        
        # ============================================================
        # Step 2: Primitive Task Decomposition
        # ============================================================
        print("\n[Step 2: Primitive Task Decomposition]")
        decomposed_task_json = primitive_task_decomposition(frames, duration)
        decomposed_task_data = json.loads(decomposed_task_json)
        print(f"  ✓ Found {len(decomposed_task_data.get('primitive_tasks', []))} primitive tasks")
        
        # ============================================================
        # Step 3: Atomic Action Decomposition (舊方法，僅用於初始分解)
        # ============================================================
        print("\n[Step 3: Atomic Action Decomposition (Initial)]")
        
        for pri_task in decomposed_task_data.get("primitive_tasks", []):
            task_id = pri_task['task_id']
            print(f"  Processing Primitive Task {task_id}...")
            
            start_time = float(pri_task['start_time'])
            end_time = float(pri_task['end_time'])
            
            # 找出時間範圍內的影格索引
            pri_frame_indices = [i for i, ts in enumerate(frame_timestamps) 
                                if start_time <= ts <= end_time]
            
            if not pri_frame_indices:
                print(f"    ⚠ No frames found in time range [{start_time:.2f}, {end_time:.2f}]")
                continue
            
            # 取得對應的 base64 影格
            pri_frames = [frames[i] for i in pri_frame_indices]
            
            print(f"    Using {len(pri_frames)} frames (time: {start_time:.2f}s - {end_time:.2f}s)")
            
            atomic_actions_json = atomic_action_decomposition(pri_frames, start_time, end_time)
            atomic_actions_data = json.loads(atomic_actions_json)
            pri_task["atomic_actions"] = atomic_actions_data.get("atomic_actions", [])
            print(f"    ✓ Found {len(pri_task['atomic_actions'])} atomic actions")
    
    # ============================================================
    # Step 4: Keyframe-based Atomic Action Timestamp Extraction
    # ============================================================
    print("\n[Step 4: Extracting Precise Atomic Action Timestamps]")
    
    # 檢查是否從特定 stage 重入
    all_segments = None
    
    if start_from_stage:
        # Resume mode - 載入已保存的 segments
        print(f"  ⚡ Resuming from: {start_from_stage}")
        
        if start_from_stage == "stage1":
            # 從 stage1 開始，不載入任何東西
            print("  📂 Starting fresh from stage1...")
            # 需要有 frames 和 sampled_times 才能執行 stage1
            # 但如果是 resume mode，這些應該要從之前的輸出重建
            # 暫時跳過，假設 frames 已經存在
            pass
        elif start_from_stage == "stage2":
            # 載入 stage1 的結果
            print("  📂 Loading stage1 results...")
            # 注意：load_segments_from_json 需要 frames 和 sampled_times
            # 在 resume mode 下，我們沒有這些，所以需要特殊處理
            # 暫時使用空的 frames，只載入元數據
            all_segments = load_segments_from_json(
                args.output_folder, "stage1_analyzed", [], []
            )
        elif start_from_stage == "stage3":
            # 載入 stage2 的結果
            print("  📂 Loading stage2 results...")
            all_segments = load_segments_from_json(
                args.output_folder, "stage2_labeled", [], []
            )
        else:
            print(f"  ⚠ Unknown stage: {start_from_stage}, starting from stage1")
    else:
        print(f"  Using {len(frames)} frames ({sum(is_keyframe_flags)} keyframes)")
    
    # Stage 1: 分析片段（描述動作）
    if all_segments is None and (not start_from_stage or start_from_stage == "stage1"):
        print("\n  [Stage 1: Analyzing segments...]")
        all_segments = analyze_all_segments(
            decomposed_task_data,
            frames,
            sampled_times,
            is_keyframe_flags,
            args.output_folder,
            max_workers=16
        )
        
        # 保存 Stage 1 結果
        save_segments_to_json(all_segments, args.output_folder, "stage1_analyzed")
    else:
        print("\n  [Stage 1: Skipped (loaded from file or not in resume scope)]")
    
    # Stage 2: 標註片段（atomic action 類型）
    if not start_from_stage or start_from_stage in ["stage1", "stage2"]:
        print("\n  [Stage 2: Labeling segments...]")
        label_all_segments(all_segments, max_workers=16)
        
        # 保存 Stage 2 結果
        save_segments_to_json(all_segments, args.output_folder, "stage2_labeled")
    else:
        print("\n  [Stage 2: Skipped (not in resume scope)]")
    
    # Stage 3: 合併連續相同動作
    if not start_from_stage or start_from_stage in ["stage1", "stage2", "stage3"]:
        print("\n  [Stage 3: Merging consecutive same actions...]")
        all_segments = merge_all_same_actions(all_segments)
        
        # 保存 Stage 3 結果
        save_segments_to_json(all_segments, args.output_folder, "stage3_merged")
    else:
        print("\n  [Stage 3: Skipped (not in resume scope)]")
    
    # 更新任務數據
    print("\n  [Updating task data with precise timestamps...]")
    for pri_task in decomposed_task_data.get("primitive_tasks", []):
        task_id = pri_task['task_id']
        
        if task_id in all_segments:
            segments = all_segments[task_id]
            # 將片段轉換為 atomic actions 格式
            atomic_actions = []
            for segment in segments:
                action = {
                    "action": segment.action or "Unknown",
                    "description": segment.description or "",
                    "start_time": segment.start_time,
                    "end_time": segment.end_time,
                    "start_idx": segment.start_idx,
                    "end_idx": segment.end_idx,
                    "confidence": segment.confidence or "medium"
                }
                atomic_actions.append(action)
            
            pri_task['atomic_actions'] = atomic_actions
            print(f"    ✓ Task {task_id}: {len(atomic_actions)} atomic actions with timestamps")
    
    print("\n  ✓ Step 4 completed successfully!")
    
    # ============================================================
    # Step 5: 保存結果
    # ============================================================
    print("\n[Step 5: Saving Results]")
    output_filename = os.path.join(args.output_folder, "task_segmentation.json")
    with open(output_filename, "w") as f:
        json.dump(decomposed_task_data, f, indent=4, ensure_ascii=False)
    print(f"  ✓ Saved to: {output_filename}")
    
    # ============================================================
    # Step 6: 製作 Atomic Action GIF
    # ============================================================
    print("\n[Step 6: Creating GIF Animations for Atomic Actions]")
    from util.viz_tool import save_gif
    
    gif_output_dir = os.path.join(args.output_folder, "atomic_action_gifs")
    os.makedirs(gif_output_dir, exist_ok=True)
    
    total_gifs = 0
    for task_id, segments in all_segments.items():
        task_gif_dir = os.path.join(gif_output_dir, f"task_{task_id}")
        os.makedirs(task_gif_dir, exist_ok=True)
        
        # 找到對應的 primitive task 資訊
        pri_task_name = "Unknown"
        for pri_task in decomposed_task_data.get("primitive_tasks", []):
            if pri_task['task_id'] == task_id:
                pri_task_name = pri_task.get('primitive_tasks_name', 'Unknown')
                break
        
        print(f"\n  [Primitive Task {task_id}: {pri_task_name}]")
        
        for seg_idx, segment in enumerate(segments):
            # 從 loader 中取得對應時間範圍的原始影格（非 base64）
            segment_images = []
            for frame in loader.frames:
                if segment.start_time <= frame.timestamp <= segment.end_time:
                    segment_images.append(frame.image)
            
            if not segment_images:
                print(f"    ⚠ No images found for segment {seg_idx}")
                continue
            
            # 產生 GIF 檔名
            action_name = segment.action.replace(" ", "_") if segment.action else "Unknown"
            gif_filename = f"segment_{seg_idx:02d}_{action_name}_{segment.start_time:.2f}s-{segment.end_time:.2f}s.gif"
            gif_path = os.path.join(task_gif_dir, gif_filename)
            
            # 準備文字標籤
            text_lines = [
                f"PT: {pri_task_name}",
                f"AA: {segment.action or 'Unknown'}",
                f"Time: {segment.start_time:.2f}s - {segment.end_time:.2f}s"
            ]
            
            # 儲存 GIF (調整大小以減少檔案大小，並加上文字標籤)
            save_gif(
                segment_images,
                gif_path,
                loop=0,        # Infinite loop
                resize=(640, 480),  # Resize to reduce file size
                text_lines=text_lines,  # 加入文字標籤
                text_position='top',  # 文字位置在上方
                gif_fps=args.gif_fps,  # Target FPS for GIF playback
                frame_step=args.gif_frame_step  # Subsample frames
            )
            
            total_gifs += 1
            print(f"    ✓ Segment {seg_idx}: {action_name} ({len(segment_images)} frames) -> {gif_filename}")
    
    print(f"\n  ✓ Created {total_gifs} GIF animations in: {gif_output_dir}")
    
    print("\n" + "="*60)
    print("Pipeline Completed Successfully!")
    print("="*60)
  

# --- Main Execution ---
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Analyze demonstration videos and decompose tasks using VLM.")
    parser.add_argument("demo_folder", help="Path to demo folder (e.g., demo_data1)")
    parser.add_argument("output_folder", help="Output folder for task_segmentation.json")
    parser.add_argument("--filename_format", default="frame_{id}.png", help="Image filename format (default: frame_{id}.png)")
    parser.add_argument(
        "--start-from-stage",
        choices=["stage1", "stage2", "stage3"],
        help="Resume from a specific stage (requires previous stage output files)"
    )
    parser.add_argument("--gif-fps", type=int, default=25, help="Target FPS for output GIFs (default: 5)")
    parser.add_argument("--gif-frame-step", type=int, default=3, help="Frame subsampling step for GIFs (e.g. 2 means use every 2nd frame). Default: 1 (no subsampling)")
    args = parser.parse_args()

    #! input/output paths
    image_folder = os.path.join(args.demo_folder, "images")
    trajectory_path = os.path.join(args.demo_folder, "trajectory.json")
    # Create output folder if it doesn't exist
    if not os.path.exists(args.output_folder):
        os.makedirs(args.output_folder)
        print(f"Created output folder: {args.output_folder}")
    if not os.path.exists(image_folder):
        print(f"Image folder '{image_folder}' does not exist. Please check the path.")
        exit()
    if not os.path.exists(trajectory_path):
        print(f"Trajectory file '{trajectory_path}' does not exist. Please check the path.")
        exit()
    main(args)
    
    


