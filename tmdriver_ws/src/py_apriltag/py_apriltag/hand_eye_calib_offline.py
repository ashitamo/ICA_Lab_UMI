import math
import yaml
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
import pickle

T_a_A =  np.array([[ 0, 1, 0, 0],
                  [ 1, 0, 0 ,0],
                  [ 0, 0, -1, 0],
                  [ 0, 0,  0 ,1]])
T_A_a = np.linalg.inv(T_a_A)

filename = 'calib_results14.pkl'  # change to your filename
# filename = 'calib_results3.pkl'  # change to your filename
with open(filename,'rb') as f:
    T_W_G_list,T_C_A_list,_,_ = pickle.load(f)
print(len(T_W_G_list),len(T_C_A_list))

def HEC(T_W_G_list,T_C_A_list,method=cv2.CALIB_HAND_EYE_DANIILIDIS):
    R_W_G = [T[:3,:3] for T in T_W_G_list]
    t_W_G = [T[:3,3] for T in T_W_G_list]
    R_C_A = [T[:3,:3] for T in T_C_A_list]
    t_C_A = [T[:3,3] for T in T_C_A_list]
    return cv2.calibrateHandEye(R_W_G, t_W_G, R_C_A, t_C_A, None, method)


poserr = []
roterr = []

methods = {'Tsai':cv2.CALIB_HAND_EYE_TSAI}


for method_name,method in methods.items():
    T_G_C = np.eye(4)
    r,t = HEC(T_W_G_list[1:],T_C_A_list[1:],method=method)
    T_G_C[:3,:3] = r
    T_G_C[:3,3] = t.reshape(3)

    T_C_A = T_C_A_list[0]
    T_W_G = T_W_G_list[0]
    T_W_a = T_W_G @ T_G_C @ T_C_A @ T_A_a
    T_a_W = np.linalg.inv(T_W_a)
    
    # T_a_W_List = []
    # for i in range(len(T_C_A_list)):
    #     T_C_A = T_C_A_list[i]
    #     T_W_G = T_W_G_list[i]
    #     T_W_a = T_W_G @ T_G_C @ T_C_A @ T_A_a
    #     T_a_W = np.linalg.inv(T_W_a)
    #     T_a_W_List.append(T_a_W)
    # T_a_W = np.mean(np.array(T_a_W_List),axis=0)
    
    # T_C_A_list = T_C_A_list[:50]
    # T_W_G_list = T_W_G_list[:50]

    roterr = []
    poserr = []
    for i in range(len(T_C_A_list)):
        T_C_A = T_C_A_list[i]
        T_W_G = T_W_G_list[i]
        
        T = T_W_G @ T_G_C @ T_C_A @ T_A_a @ T_a_W
        rot = R.from_matrix(T[:3,:3]).as_euler('xyz', degrees=True)
        # print("repjt rot err: {}".format(np.array2string(rot,separator=',',precision=6)))
        # print("repjt pos err: {}\n".format(np.array2string(T[:3,3],separator=',',precision=6)))
        roterr.append(rot)
        poserr.append(T[:3,3])

    roterr = np.array(roterr)
    poserr = np.array(poserr)
    print("Method: {}".format(method_name))
    print("T_G_C: {}".format(np.array2string(T_G_C,separator=',',precision=6)))
    print('xyz:',T_G_C[0,3],T_G_C[1,3],T_G_C[2,3])
    rot = R.from_matrix(T_G_C[:3,:3]).as_euler('xyz', degrees=True)
    print('Rx,Ry,Rz:',rot[0],rot[1],rot[2])
    print("T_a_W: {}".format(np.array2string(T_a_W,separator=',',precision=6)))
    print("mean rot err: {}".format(np.array2string(np.mean(np.abs(roterr),axis=0),separator=',',precision=6)))
    print("std rot err: {}".format(np.array2string(np.std(np.abs(roterr),axis=0),separator=',',precision=6)))
    print("mean pos err: {}".format(np.array2string(np.mean(np.abs(poserr),axis=0),separator=',',precision=6)))
    print("std pos err: {}".format(np.array2string(np.std(np.abs(poserr),axis=0),separator=',',precision=6)))
    print()



# Read from YAML file
with open('ICA_Lab_UMI_Config.yaml', 'r') as file:
    config_data = yaml.safe_load(file)
T_G_C = np.array(config_data['T_G_C'])
T_C_G = np.array(config_data['T_C_G'])

# Write to YAML file
with open('ICA_Lab_UMI_Config.yaml', 'r') as file:
    config_data = yaml.safe_load(file)
config_data['T_G_C'] = T_G_C.tolist()
config_data['T_C_G'] = T_C_G.tolist()
with open('ICA_Lab_UMI_Config.yaml', 'w') as file:
    yaml.dump(config_data, file, default_flow_style=False)


