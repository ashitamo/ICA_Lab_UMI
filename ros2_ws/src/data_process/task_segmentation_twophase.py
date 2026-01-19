import cv2
import base64
import os
from openai import OpenAI
import numpy as np
import pandas as pd
import json
import argparse

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

# --- Function Definitions ---
# This function processes a video file to extract frames at specified intervals.
# It returns a list of base64-encoded images and the duration of the video.
def process_image_folder(folder_path, trajectory_path, filename_format="color_{id}.png", fps=2.0):

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
    print(trajectory_df.head())  # Verify DataFrame structure

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
def primitive_task_decomposition(base64_frames, video_duration, fps):
    """
    Sends video frames to the OpenAI VLM and asks it to decompose the task.

    Args:
        base64_frames (list): A list of base64-encoded frames.
        video_duration (float): The total duration of the video.

    Returns:
        str: The JSON string response from the model.
    """
    # This prompt is engineered based on the user's request and the provided research paper.
    # It instructs the model to act as a robotics expert and follow the specific taxonomy.
    user_content = [
        {
            "type": "text",
            "text": (
                f"Analyze the following sequence of frames. These frames were sampled at approximately {fps:.1f} FPS from a video with a total duration of {video_duration:.2f} seconds. "
                f"The main goal is to decompose the task shown in the video into primitive tasks"
                f"This is a peg in hole task with one or two cylinder with yellow or orange color."
                f"The float timestamps are on the left top corner of each frame image."
                f"Provide the task decomposition as a single JSON object."
            )
        }
    ]
    # 將所有圖片加到提示的後面
    user_content.extend([{"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{frame}"}} for frame in base64_frames])    
    
    prompt_messages = [
        {
            "role": "system",
            "content": 
            """
            You are an expert in robotics and manufacturing process analysis. Your task is to analyze a sequence of video and decompose it into a structured JSON format. 
            A **Primitive Task** is a complete actions on a single object, including approaching, manipulating, and finally **retreating away from the object.
            ** A new primitive task begins only when the subject's hand starts a new `Approach` action towards a new target object.         
            The float timestamps are on the left top corner of each frame image.
            Critical Rules:
            1. For each **Primitive Task**, you must identify the precise start and end times from the video in seconds.
            2. Your output **MUST** be a single JSON object and adhere strictly to the format below. Do not include any text or explanations outside of the JSON.
            3. The example below is illustrative; your output should accurately reflect the actions and timings of the provided video. 
            {
            "task_name": "Assembly of Two Yellow Cylinders",
            "primitive_tasks": [
                {
                "task_id": 1,
                "primitive_tasks_name": "Pick and insert Yellow Cylinder in the hole of the Base",
                "start_time": 0.0,
                "end_time": 8.2,
                "atomic_actions": []
                },
                {
                "task_id": 2,
                "primitive_tasks_name": "Pick and insert Second Yellow Cylinder",
                "start_time":10.0,
                "end_time": 18.5,
                "atomic_actions": []
                }
            ]
            }
            """
        },
        {
            "role": "user",
            "content": user_content,
        },
    ]

    print("Sending request to OpenAI API. This may take a moment...")
    try:
        response = client.chat.completions.create(
            model="gpt-4o",
            messages=prompt_messages,
            max_tokens=2000,
            response_format={"type": "json_object"} # Enforce JSON output
        )
        json_response = response.choices[0].message.content
        print("Successfully received response from API.")
        return json_response
    except Exception as e:
        print(f"An error occurred while communicating with the OpenAI API: {e}")
        return None
    

def atomic_action_decomposition(picked_frames, start_time, end_time, fps):
    """
    Sends video frames to the OpenAI VLM and asks it to decompose the task into atomic actions.

    Args:
        base64_frames (list): A list of base64-encoded frames.
    """
    
    primitive_task_duration = end_time - start_time

    user_content = [
    {"type": "text", 
        "text": (
        f"Analyze these frames from a {primitive_task_duration:.2f} seconds video clip." 
        f"This clip starts at the {start_time:.2f} second mark of the original demonstration."
        f"These frames were sampled at approximately {fps:.1f} FPS from a video with a total duration of {duration:.2f} seconds."
        f"The timestamps are on the left top corner of each frame image."
        f"The main goal is to decompose the primitive task into atomic actions, add it in the 'atomic_actions' field of the primitive task."
        f"Provide the task decomposition as a single JSON object."
        )}
    ]
    
    user_content.extend([{"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{frame}"}} for frame in picked_frames])    

    prompt_messages = [
    {"role": "system", "content": """
        You are an expert robotics task analyst. Your task is to decompose the provided video segment into a list of atomic actions based on the strict definitions below.

        **Atomic Actions Definitions:**
        - **Approach**: The subject moves towards an object. This action is only valid if the hand is empty.
        - **Grasp**: Starts when subjest near to the object, and ends when gripper close and contact the object.
        - **Move**: The process of transporting a grasped object. This also includes the final **retreat** action after an object has been placed or inserted.
        - **Insert**: Fitting an object into a **geometrically constrained feature** (like a hole, slot, or cavity). End when the gripper open.

        **CRITICAL RULES:**
        1. Your output **MUST** be a single JSON object with a single key "atomic_actions".
        2. The "action" field **MUST** be one of the nine defined categories. Do not create new categories like "Release", "Retreat" or "Align".
        3. All timestamps **MUST** be absolute, based on the original video's timeline as specified in the user prompt.
        4. Action after "Insert" **MUST** be "Move" (retreat).
     

        **Example output:**
        {
        "atomic_actions": [
            {
            "action": "Approach",
            "description": "Hand moves towards the yellow cylinder on the table.",
            "start_time": 0.0,
            "end_time": 1.5
            },
            {
            "action": "Grasp",
            "description": "Fingers close to securely grasp the yellow cylinder.",
            "start_time": 1.5,
            "end_time": 2.8
            },
            {
            "action": "Move",
            "description": "Lifting the yellow cylinder and moving it towards the black base.",
            "start_time": 2.8,
            "end_time": 5.6
            },
            {
            "action": "Insert",
            "description": "Insert the yellow cylinder into the hole of the black base.",
            "start_time": 5.6,
            "end_time": 7.1
            },
            {
            "action": "Move",
            "description": "Lifting the yellow cylinder and moving it towards the black base.",
            "start_time": 7.1,
            "end_time": 10.0
            }
            ]
            }
        """
    },
    {"role": "user", "content": user_content}
    ]  

    print("Sending request to OpenAI API. This may take a moment...")
    try:
        response = client.chat.completions.create(
            model="gpt-4o",
            messages=prompt_messages,
            max_tokens=2000,
            response_format={"type": "json_object"} # Enforce JSON output
        )
        json_response = response.choices[0].message.content
        print("Successfully received response from API.")
        return json_response
    except Exception as e:
        print(f"An error occurred while communicating with the OpenAI API: {e}")
        return None




# --- Main Execution ---
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Analyze demonstration videos and decompose tasks using VLM.")
    parser.add_argument("demo_folder", help="Path to demo folder (e.g., demo_data6)")
    parser.add_argument("output_folder", help="Output folder for task_segmentation.json")
    parser.add_argument("--filename_format", default="frame_{id}.png", help="Image filename format (default: frame_{id}.png)")
    
    args = parser.parse_args()
    
    # Automatically construct paths based on demo folder
    image_folder = os.path.join(args.demo_folder, "timestamp_images")
    trajectory_path = os.path.join(args.demo_folder, "trajectory.json")
    
    fps = 5.0  # Frame rate for sampling frames
    
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

    try:
        frames, duration = process_image_folder(image_folder, trajectory_path, args.filename_format, fps)    
        print("--- Summary of Paths ---")
        print(f"Demo folder: {args.demo_folder}")
        print(f"Image folder: {image_folder}")
        print(f"Trajectory path: {trajectory_path}")
        print(f"Output folder: {args.output_folder}")
        
        if not frames:
            print("No frames were extracted. Exiting.")
        else:
            # 2. Send frames to VLM for decomposition primitive tasks
            print("===Starting Primitive Task Decomposition===")
            decomposed_task_json = primitive_task_decomposition(frames, duration, fps)
            decomposed_task_data = json.loads(decomposed_task_json)
            print(decomposed_task_json)
            
            print("===Starting Atomic Action Decomposition for Each Primitive Task===")
            # 3. decompose into atomic actions 
            for pri_task in decomposed_task_data.get("primitive_tasks", []):
                print(f"===Processing Primitive Task:{pri_task['task_id']} for Atomic Action Decomposition===")
                start_time = float(pri_task['start_time'])
                end_time = float(pri_task['end_time'])
                start_idx = int(start_time * fps)
                end_idx = int(end_time * fps)
                pri_frames = frames[start_idx:end_idx+1]
                atomic_actions_json = atomic_action_decomposition(pri_frames, start_time, end_time, fps) 
                print(f"Atomic Action Decomposition Result for Primitive Task {pri_task['task_id']}:")
                print(atomic_actions_json)
                atomic_actions_data = json.loads(atomic_actions_json)
                pri_task["atomic_actions"] = atomic_actions_data.get("atomic_actions", [])  
            
            #decomposed_task_json = atomic_action_decomposition(decomposed_primitive_task_json, frames, duration, fps)

            # 3. Save the output to a JSON file in the specified output folder
            if decomposed_task_json:
                output_filename = os.path.join(args.output_folder, "task_segmentation.json")
                with open(output_filename, "w") as f:
                    json.dump(decomposed_task_data, f, indent=4, ensure_ascii=False)
                print(f"Successfully saved task decomposition to '{output_filename}'")
                print("\n--- Content of JSON file ---")
                print(json.dumps(decomposed_task_data, indent=4, ensure_ascii=False))  

    except (FileNotFoundError, IOError) as e:
        print(f"Error: {e}")

    except Exception as e:
        print(f"An unexpected error occurred: {e}")
