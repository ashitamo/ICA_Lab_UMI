import yaml
import numpy as np
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

def xyz_w_to_cam_pose(xyz_c,current_pose):
    p_c = np.array([xyz_c[0],xyz_c[1],xyz_c[2],1.0])

    T_a_c = np.eye(4)
    T_a_c[:3,:3] = R.from_quat(current_pose[4:8]).as_matrix()
    T_a_c[:3,3] = current_pose[1:4]
    p_a = T_a_c @ p_c

    T_a_p = np.eye(4)
    rot = R.from_quat(current_pose[4:8]).as_euler('xyz', degrees=True)
    rot = R.from_euler('xyz', [180.0, 0.0, rot[2]], degrees=True).as_quat()
    T_a_p = np.array([0.0,p_a[0],p_a[1],p_a[2],rot[0],rot[1],rot[2],rot[3],current_pose[-1]])

    T_a_g = p2g(T_a_p,0.09)
    T_a_c = g2c(T_a_g)
    return T_a_c

xyz_w_to_cam_pose(
    np.array((0.01698485198067689, 0.0029507474725629676, 0.218)),
    np.array([1.0, -0.31245754995379893, -0.30811440314242744, 0.32574873446614333, 0.6841081635125055, 0.7293805732370083, 2.6264509623035608e-17, 2.954069568432461e-17, 0.027627])
)

np.array((-0.007932857819202237, 0.0009136112362796088, 0.34700000000000003)),
np.array([1.0, -0.31245754995379893, -0.30811440314242744, 0.32574873446614333, 0.6841081635125055, 0.7293805732370083, 2.6264509623035608e-17, 2.954069568432461e-17, 0.027627])


np.array((0.01698485198067689, 0.0029507474725629676, 0.218)),
np.array([1.0, -0.31245754995379893, -0.30811440314242744, 0.32574873446614333, 0.6841081635125055, 0.7293805732370083, 2.6264509623035608e-17, 2.954069568432461e-17, 0.027627])
