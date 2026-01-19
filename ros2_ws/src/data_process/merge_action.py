import os
import glob
import sys
from PIL import Image
from matplotlib.animation import FuncAnimation
import json
import argparse
import matplotlib.pyplot as plt

   


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Display GIFs and modify actions in task segmentation')
    parser.add_argument('output_folder', help='Path to the output folder containing task_segmentation.json')
    
    args = parser.parse_args()
    output_folder = args.output_folder

    """Display GIFs found in each task folder within the primitive task frames folder using task_segmentation.json."""
    
    # Load task segmentation data
    json_file = os.path.join(output_folder, "task_segmentation.json")
    
    if not os.path.exists(json_file):
        print(f"task_segmentation.json not found in {output_folder}")
        sys.exit(1)
    
    with open(json_file, 'r') as f:
        task_data = json.load(f)

    print(f"Loaded task segmentation data from {json_file}")

    for primitive in task_data.get("primitive_tasks", []):
            task_id = primitive.get("task_id")
            task_name = primitive.get("primitive_tasks_name")
            actions = primitive.get("atomic_actions", [])
            
            # Process actions and merge consecutive identical ones
            i = 0
            while i < len(actions):
                current_action = actions[i]
                
                # Check if next action exists and is the same
                if i + 1 < len(actions) and actions[i + 1].get('action') == current_action.get('action'):
                    # Merge the actions
                    next_action = actions[i + 1]
                    print(f"Merging action {i} and {i+1} of type '{current_action.get('action')}'")
                    # Update end time to the next action's end time
                    if 'end_time' in next_action:
                        current_action['end_time'] = next_action['end_time']

                    if 'end_idx' in next_action:
                        current_action['end_idx'] = next_action['end_idx']

                    if 'description' in next_action:
                        if 'description' in current_action:
                            current_action['description'] += " " + next_action['description']
                        else:
                            current_action['description'] = next_action['description']

                    # Remove the next action from the list
                    actions.pop(i + 1)
                    print(f"Merged consecutive '{current_action.get('action')}' actions")
                    
                    # Don't increment i, check again from the same position
                    continue
                
                i += 1

    # Save the modified JSON back to the file
    with open(json_file, 'w', encoding='utf-8') as f:
        json.dump(task_data, f, indent=2, ensure_ascii=False)

    print(f"Modified task segmentation data saved to {json_file}")
