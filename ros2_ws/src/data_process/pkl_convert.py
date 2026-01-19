import pickle
import json
import os
import cv2
import argparse

'''
Usage:
Output folder is automatically determined from input path
python3 pkl_convert.py demo_data6/data_list.pkl 
'''

def convert_pkl_file(input_path='data.pkl', output_folder='demo_data'):
    """
    Loads a Python pickle file, converts it to a Pandas DataFrame,
    and saves it as both a JSON and a CSV file.
    """
    # Load the Pickle File
    print(f"Attempting to load data from '{input_path}'...")
    try:
        # Open the pickle file in binary read mode ('rb')
        with open(input_path, 'rb') as f:
            # Load the data from the file
            data_from_pickle = pickle.load(f)
        print("Successfully loaded data from .pkl file.")
    except FileNotFoundError:
        print(f"Error: The input file '{input_path}' was not found.")
    except Exception as e:
        print(f"An error occurred: {e}")
        return


    # Create image folder if it doesn't exist
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    
    images_dir = os.path.join(output_folder, "images")
    os.makedirs(images_dir, exist_ok=True)
    print(f"Data will be saved in '{output_folder}' directory.")


    processed_gripper_state = []
    demo_start_time = data_from_pickle[0][0]
    #print(f"Demo start time: {demo_start_time}")

    for i, data_point in enumerate(data_from_pickle):
        timestamp = data_point[0]
        # a. Extract and save the image
        image_data = data_point[2]
        if image_data is not None:
            image_filename = f"frame_{i}.png"
            image_filepath = os.path.join(images_dir, image_filename)
            image_rgb = cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
            cv2.imwrite(image_filepath, image_rgb)
      
        # b. Extract trajectory and gripper data ---
        pose_array = data_point[1]
        gripper_data = data_point[3]
        if pose_array is not None and gripper_data is not None:
            trajectory_data = {
                "id": i,
                "timestamp": timestamp - demo_start_time,
                "pos_x": pose_array[0],
                "pos_y": pose_array[1],
                "pos_z": pose_array[2],
                "quat_x": pose_array[3],
                "quat_y": pose_array[4],
                "quat_z": pose_array[5],
                "quat_w": pose_array[6],
                "gripper_width": gripper_data 
            }
        processed_gripper_state.append(trajectory_data)


    # 4. Save the Trajectory and Gripper Log as a JSON file
    trajectory_filepath = os.path.join(output_folder, "trajectory.json")
    with open(trajectory_filepath, 'w') as f:
        json.dump(processed_gripper_state, f, indent=2)
     
    print("Successfully saved demonstration data to images and JSON file.")

if __name__ == '__main__':
    '''
    Usage:
    Output folder is automatically determined from input path
    python3 pkl_convert.py demo_data6/data_list.pkl 
    '''

    parser = argparse.ArgumentParser(description="Convert demonstration .pkl file to JSON and images.")
    parser.add_argument("input", help="Path to input .pkl file")
    args = parser.parse_args()

    # Automatically determine output folder from input path
    input_dir = os.path.dirname(args.input)
    if input_dir == "":  # If no directory specified, use current directory
        input_dir = "."
    
    convert_pkl_file(input_path=args.input, output_folder=input_dir)