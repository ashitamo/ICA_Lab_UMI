
from scipy.spatial.transform import Rotation as R
import pickle
import cv2
with open('/home/lab606/ICA_Lab_UMI/tmdriver_ws/src/py_apriltag/data_list_new1.pkl', 'rb') as f:
    data_list = pickle.load(f)
print(len(data_list))

with open('nrap_03.txt', 'w') as f:
    for i in range(len(data_list)):
        grip_state = 0.0
        try:
            time_stamped,pose_a_i0,image,grip_state = data_list[i]
        except:
            time_stamped,pose_a_i0,image = data_list[i]
        cv2.imshow('image',image)
        cv2.waitKey(33)
        f.write(f"{time_stamped},{pose_a_i0[0]},{pose_a_i0[1]},{pose_a_i0[2]},{pose_a_i0[3]},{pose_a_i0[4]},{pose_a_i0[5]},{pose_a_i0[6]},{grip_state}\n")