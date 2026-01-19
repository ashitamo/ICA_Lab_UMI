import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

with open('/home/lab606/ICA_Lab_UMI/ICA_Lab_UMI_Config.yaml', 'r') as file:
    config_data = yaml.safe_load(file)
    T_G_C = np.array(config_data['T_G_C'])
    T_C_G = np.array(config_data['T_C_G'])
    T_G_E = np.array(config_data['T_G_E'])
    T_E_G = np.array(config_data['T_E_G'])
    T_W_a = np.array(config_data['T_W_a'])
    T_a_W = np.linalg.inv(T_W_a)
    T_a_A = np.array(config_data['T_a_A'])
    T_A_a = np.array(config_data['T_A_a'])
    T_E_C = T_E_G @ T_G_C
    T_C_E = np.linalg.inv(T_E_C)
def g2c(p):
    T_x_G = np.eye(4)
    T_x_G[:3,:3] = R.from_quat(p[4:8]).as_matrix()
    T_x_G[:3,3] = np.array(p[1:4])
    T_x_C = T_x_G @ T_G_C
    p[1] = T_x_C[0,3]
    p[2] = T_x_C[1,3]
    p[3] = T_x_C[2,3]
    p[4:8] = R.from_matrix(T_x_C[:3,:3]).as_quat()
    return p
    
def c2g(p):
    T_x_C = np.eye(4)
    T_x_C[:3,:3] = R.from_quat(p[4:8]).as_matrix()
    T_x_C[:3,3] = np.array(p[1:4])
    T_x_G = T_x_C @ T_C_G
    p[1] = T_x_G[0,3]
    p[2] = T_x_G[1,3]
    p[3] = T_x_G[2,3]
    p[4:8] = R.from_matrix(T_x_G[:3,:3]).as_quat()
    return p

def a2w(p):
    T_a_x = np.eye(4)
    T_a_x[:3,:3] = R.from_quat(p[4:8]).as_matrix()
    T_a_x[:3,3] = np.array(p[1:4])
    T_W_x = T_W_a @ T_a_x
    p[1] = T_W_x[0,3]
    p[2] = T_W_x[1,3]
    p[3] = T_W_x[2,3]
    p[4:8] = R.from_matrix(T_W_x[:3,:3]).as_quat()
    return p

def w2a(p):
    T_W_x = np.eye(4)
    T_W_x[:3,:3] = R.from_quat(p[4:8]).as_matrix()
    T_W_x[:3,3] = np.array(p[1:4])
    T_a_x = T_a_W @ T_W_x
    p[1] = T_a_x[0,3]
    p[2] = T_a_x[1,3]
    p[3] = T_a_x[2,3]
    p[4:8] = R.from_matrix(T_a_x[:3,:3]).as_quat()
    return p

def p2g(p,len):
    T_W_P = np.eye(4)
    T_W_P[:3,:3] = R.from_quat(p[4:8]).as_matrix()
    rot = R.from_quat(p[4:8]).as_euler('xyz', degrees=True)
    print(f"p2g input euler angles: {rot}")
    T_W_P[:3,3] = np.array(p[1:4])
    T_P_E = np.array(
        [[ 1, 0, 0, 0],
        [ 0, 1, 0, 0],
        [ 0, 0, 1, -len],
        [ 0, 0, 0, 1]]
    )
    T_W_E = T_W_P @ T_P_E
    T_W_G = T_W_E @ T_E_G
    p[1] = T_W_G[0,3]
    p[2] = T_W_G[1,3]
    p[3] = T_W_G[2,3]
    p[4:8] = R.from_matrix(T_W_G[:3,:3]).as_quat()
    return p

def g2p(p,len):
    T_W_G = np.eye(4)
    T_W_G[:3,:3] = R.from_quat(p[4:8]).as_matrix()
    T_W_G[:3,3] = np.array(p[1:4])
    T_E_P = np.array(
        [[ 1, 0, 0, 0],
        [ 0, 1, 0, 0],
        [ 0, 0, 1, len],
        [ 0, 0, 0, 1]]
    )
    T_W_E = T_W_G @ T_G_E @ T_E_P
    p[1] = T_W_E[0,3]
    p[2] = T_W_E[1,3]
    p[3] = T_W_E[2,3]
    p[4:8] = R.from_matrix(T_W_E[:3,:3]).as_quat()
    return p

def g2e(p):
    T_x_G = np.eye(4)
    T_x_G[:3,:3] = R.from_quat(p[4:8]).as_matrix()
    T_x_G[:3,3] = np.array(p[1:4])
    T_x_E = T_x_G @ T_G_E
    p[1] = T_x_E[0,3]
    p[2] = T_x_E[1,3]
    p[3] = T_x_E[2,3]
    p[4:8] = R.from_matrix(T_x_E[:3,:3]).as_quat()
    return p

def e2g(p):
    T_x_E = np.eye(4)
    T_x_E[:3,:3] = R.from_quat(p[4:8]).as_matrix()
    T_x_E[:3,3] = np.array(p[1:4])
    T_x_G = T_x_E @ T_E_G
    p[1] = T_x_G[0,3]
    p[2] = T_x_G[1,3]
    p[3] = T_x_G[2,3]
    p[4:8] = R.from_matrix(T_x_G[:3,:3]).as_quat()
    return p

fixedhole_W_G = {
    'top_right':    np.array([0.0,-0.115, -0.52055,0.340, 6.12323400e-17 ,1.00000000e+00 ,6.12323400e-17 ,3.74939946e-33,0.0293]),
    'bottom_right': np.array([0.0,-0.115, -0.435,  0.3425,6.12323400e-17 ,1.00000000e+00 ,6.12323400e-17 ,3.74939946e-33,0.0293]),
    'top_left':     np.array([0.0,-0.0275, -0.520,  0.3425,6.12323400e-17 ,1.00000000e+00 ,6.12323400e-17 ,3.74939946e-33,0.0293]),
    'bottom_left':  np.array([0.0,-0.0275, -0.435,  0.3425,6.12323400e-17 ,1.00000000e+00 ,6.12323400e-17 ,3.74939946e-33,0.0293]),
}

fixedhole_A_C = {
    'top_right': w2a(g2c(fixedhole_W_G['top_right'].copy())),
    'bottom_right': w2a(g2c(fixedhole_W_G['bottom_right'].copy())),
    'top_left': w2a(g2c(fixedhole_W_G['top_left'].copy())),
    'bottom_left': w2a(g2c(fixedhole_W_G['bottom_left'].copy())),
}
fixedhole_A_C = {
    'top_right':    np.array([ 0. , -0.36819674, -0.24152073,  0.30885443,  0.70020218, 0.7074058 ,  0.07957089,  0.05442807,  0.0293    ]),
    'bottom_right': np.array([ 0. , -0.28263099, -0.24019844,  0.31043499,  0.70020218,0.7074058 ,  0.07957089,  0.05442807,  0.0293    ]),
    'top_left':     np.array([ 0. , -0.36625173, -0.32902868,  0.31027575,  0.70020218,0.7074058 ,  0.07957089,  0.05442807,  0.0293    ]),
    'bottom_left':  np.array([ 0. , -0.28126324, -0.32768488,  0.30936234,  0.70020218, 0.7074058 ,  0.07957089,  0.05442807,  0.0293    ]),
}
if __name__ == "__main__":
    print(fixedhole_A_C)