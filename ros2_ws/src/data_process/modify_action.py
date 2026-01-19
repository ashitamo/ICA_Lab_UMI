import os
import glob
import sys
from PIL import Image
from matplotlib.animation import FuncAnimation
import json
import argparse
import matplotlib.pyplot as plt


'''
Usage: python3 modify_action.py out_demo1 

out_demo1 should contain task_segmentation.json and atomic_action_gifs folder.
'''
def display_single_gif_with_input(gif_file, task_folder):
    """Display a single GIF file with animation and handle user input."""
    print(f"Displaying: {os.path.basename(gif_file)}")
    
    # Open and display the GIF
    img = Image.open(gif_file)
    
    # Enable interactive mode
    plt.ion()
    
    # Create figure and axis
    fig, ax = plt.subplots()
    ax.axis('off')
    
    frames = []
    try:
        while True:
            frames.append(img.copy())
            img.seek(img.tell() + 1)
    except EOFError:
        pass
    
    # Animation function
    def animate(frame_num):
        ax.clear()
        ax.imshow(frames[frame_num])
        ax.axis('off')
        ax.set_title(f"{task_folder} - {os.path.basename(gif_file)}")
    
    # Create animation
    anim = FuncAnimation(fig, animate, frames=len(frames), 
                       interval=100, repeat=True)
    
    plt.show()
    plt.pause(0.1)  # Small pause to ensure the window appears
    
# Ask for user input while animation is running
def get_user_action_approval():
    """Get user input to approve or reject the current action."""
    while True:
        response = input("Is this action correct? (Y/N): ").strip().upper()
        if response == 'Y':
            return True
        elif response == 'N':
            return False
        else:
            print("Please enter Y or N.")



# Example usage
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
    
    # Iterate through each task in the JSON
    gif_folder = os.path.join(output_folder, "atomic_action_gifs")
    for primitive in task_data.get("primitive_tasks", []):
        task_id = primitive.get("task_id")
        task_name = primitive.get("primitive_tasks_name")
        for idx, action in enumerate(primitive.get("atomic_actions", [])):
            print(f"\n--- Atomic Action {idx}: {action.get('action')} ---")
            task_path = os.path.join(gif_folder, f"task_{task_id}")
            
            if not os.path.exists(task_path):
                print(f"Task folder not found: {task_path}")
                continue
            
            action_name = action.get('action', f'action_{idx}')
            
            # Search for GIF files that match the pattern
            gif_pattern = os.path.join(task_path, f"segment_{idx:02d}*.gif")
            matching_gifs = glob.glob(gif_pattern)
            
            if matching_gifs:
                gif_file = matching_gifs[0]  # Use the first match
                print(f"  Action: {action_name}")
                display_single_gif_with_input(gif_file, f"{action_name}")
                approved = get_user_action_approval()   #
                if approved:
                    print("  Action approved by user.")
                else:
                    # Get new action from user
                    new_action_number = input("Enter the new action number: 1-Approach, 2-Grasp, 3-Move, 4-Insert, 5-Retreat: ").strip()
                    action_map = {
                        '1': 'Approach',
                        '2': 'Grasp',
                        '3': 'Move',
                        '4': 'Insert',
                        '5': 'Retreat'
                    }
                    if new_action_number:
                        # Update the action in the task data
                        new_action = action_map.get(new_action_number, action.get('action'))
                        action['action'] = new_action
                        print(f"  Action updated to: {new_action}")
                        
                        # Save the updated JSON back to the original file
                        with open(json_file, 'w', encoding='utf-8') as f:
                            json.dump(task_data, f, indent=2, ensure_ascii=False)
                        print("  JSON file updated successfully.")
                    else:
                        print("  No action entered. Skipping update.")

            else:
                print(f"  GIF not found for action '{action_name}' with pattern: {gif_pattern}")