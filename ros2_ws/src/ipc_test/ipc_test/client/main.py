from client import sender_receiver

def main():
    IMAGE_PATH = r"/home/lab606/ICA_Lab_UMI/ros2_ws/src/ipc_test/ipc_test/client/ty_shirt_half.jpg" 
    keypoints = sender_receiver(IMAGE_PATH)
    print(keypoints)
    
if __name__ == "__main__":
    main()