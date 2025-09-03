import math
import numpy as np

from scipy.spatial.transform import Rotation as R
import time
import matplotlib.pyplot as plt
T_W_W = np.eye(4)
T_W_A = np.array([[ 0, 1, 0, 0.170],
                  [ 1, 0, 0 ,-0.415],
                  [ 0, 0, -1, 0],
                  [ 0, 0,  0 ,1]])
T_A_W = np.linalg.inv(T_W_A)



T_a_A =  np.array([[ 0, 1, 0, 0],
                  [ 1, 0, 0 ,0],
                  [ 0, 0, -1, 0],
                  [ 0, 0,  0 ,1]])
T_A_a = np.linalg.inv(T_a_A)

T_W_a = T_W_A @ T_A_a
print(T_W_a)
T_C_c = np.array([[ -1, 0, 0, 0],
                  [ 0, 1, 0 ,0],
                  [ 0, 0, -1 ,0],
                  [ 0, 0,  0 ,1]])

T_c_C = np.linalg.inv(T_C_c)

class RealtimeLowPassFilter:
    def __init__(self, alpha=0.2):
        """
        Initializes a first-order low-pass filter.
        :param alpha: Filter coefficient, between 0 and 1.
                      Higher alpha means more smoothing (lower cutoff frequency).
        """
        if not (0 <= alpha <= 1):
            raise ValueError("Alpha must be between 0 and 1.")
        self.alpha = alpha
        self.filtered_value = 0.0

    def update(self, new_sample):
        """
        Processes a new sample and updates the filtered value.
        :param new_sample: The new data point to filter.
        :return: The current filtered value.
        """
        self.filtered_value = self.alpha * new_sample + (1 - self.alpha) * self.filtered_value
        return self.filtered_value
    

def calV1(p0,p1,p2):
    t0 = p0[0]
    t1 = p1[0]
    t2 = p2[0]

    p0 = p0[1:]
    p1 = p1[1:]
    p2 = p2[1:]

    v1 = (p1[:3]-p0[:3])/(t1-t0)
    v2 = (p2[:3]-p1[:3])/(t2-t1)

    r0 = p0[3:]
    r1 = p1[3:]
    r2 = p2[3:]
    
    w1 = (r1 - r0)
    for i in range(3):
        if abs(w1[i]) > np.pi:
            w1[i] = w1[i] - np.sign(w1[i])*2*np.pi
    w1 = w1/(t1-t0)

    w2 = (r2 - r1)
    for i in range(3):
        if abs(w2[i]) > np.pi:
            w2[i] = w2[i] - np.sign(w2[i])*2*np.pi
    w2 = w2/(t2-t1)


    v = (v1 + v2)/2
    w = (w1 + w2)/2

    v = np.append(v,w)

    return v

def calV2(p0,p2):
    t0 = p0[0]
    t2 = p2[0]

    p0 = p0[1:]
    p2 = p2[1:]

    v = (p2[:3]-p0[:3])/(t2-t0)

    r0 = p0[3:]
    r2 = p2[3:]

    w = (r2 - r0)
    for i in range(3):
        if abs(w[i]) > np.pi:
            w[i] = w[i] - np.sign(w[i])*2*np.pi
    w = w/(t2-t0)

    v = np.append(v,w)

    return v

def calV3(p0,p1):
    t0 = p0[0]
    t1 = p1[0]

    p0 = p0[1:]
    p1 = p1[1:]

    v = (p1[:3]-p0[:3])/(t1-t0)

    r0 = p0[3:]
    r1 = p1[3:]

    w = (r1 - r0)
    for i in range(3):
        if abs(w[i]) > np.pi:
            w[i] = w[i] - np.sign(w[i])*2*np.pi
    w = w/(t1-t0)

    v = np.append(v,w)

    return v

def a2w(p):
    T_a_ci = np.eye(4)
    T_a_ci[:3,:3] = R.from_quat(p[4:]).as_matrix()
    T_a_ci[:3,3] = np.array(p[1:4])
    T_W_ci = T_W_a @ T_a_ci
    p[1] = T_W_ci[0,3]
    p[2] = T_W_ci[1,3]
    p[3] = T_W_ci[2,3]
    p[4:] = R.from_matrix(T_W_ci[:3,:3]).as_quat()
    return p

lines = open('nrap_02.txt', 'r').readlines()
for i in range(len(lines)):
    stamp,x,y,z,px,py,pz,pw,g = lines[i].split(',')
    stamp,x,y,z,px,py,pz,pw = float(stamp),float(x),float(y),float(z),float(px),float(py),float(pz),float(pw)
    lines[i] = [stamp,x,y,z,px,py,pz,pw]

x = []
y = []
z = []
rx = []
ry = []
rz = []

v1x = []
v1y = []
v1z = []
w1x = []
w1y = []
w1z = []

v2x = []
v2y = []
v2z = []
w2x = []
w2y = []
w2z = []

v3x = []
v3y = []
v3z = []
w3x = []
w3y = []
w3z = []


t = []
alpha = 0.1
lpf = [RealtimeLowPassFilter(alpha) for i in range(6)]

for i in range(len(lines)):
    p0 = a2w(lines[(i-1)%len(lines)].copy())
    p1 = a2w(lines[i%len(lines)].copy())
    p2 = a2w(lines[(i+1)%len(lines)].copy())
    if i % len(lines) == 0:
        i = 0

    rot = R.from_quat(p0[4:]).as_euler('xyz', degrees=False)
    p0 = np.array([p0[0],p0[1],p0[2],p0[3],rot[0],rot[1],rot[2]])
    rot = R.from_quat(p1[4:]).as_euler('xyz', degrees=False)
    p1 = np.array([p1[0],p1[1],p1[2],p1[3],rot[0],rot[1],rot[2]])
    rot = R.from_quat(p2[4:]).as_euler('xyz', degrees=False)
    p2 = np.array([p2[0],p2[1],p2[2],p2[3],rot[0],rot[1],rot[2]])
    
    v1 = [0.0,0.0,0.0,0.0,0.0,0.0]
    v2 = [0.0,0.0,0.0,0.0,0.0,0.0]
    v3 = [0.0,0.0,0.0,0.0,0.0,0.0]
    if i % len(lines) != 0:
        v1 = calV1(p0.copy(),p1.copy(),p2.copy())  
        v2 = calV2(p0.copy(),p2.copy())
        v3 = calV3(p0.copy(),p1.copy())
        v3 = [lpf[i].update(v3[i]) for i in range(6)]
        duration = p1[0] - p0[0]
    else:
        duration = 2
    position = [p1[1],p1[2],p1[3],p1[4],p1[5],p1[6]]

    x.append(position[0])
    y.append(position[1])
    z.append(position[2])
    rx.append(position[3])
    ry.append(position[4])
    rz.append(position[5])

    v1x.append(v1[0])
    v1y.append(v1[1])
    v1z.append(v1[2])
    w1x.append(v1[3])
    w1y.append(v1[4])
    w1z.append(v1[5])

    v2x.append(v2[0])
    v2y.append(v2[1])
    v2z.append(v2[2])
    w2x.append(v2[3])
    w2y.append(v2[4])
    w2z.append(v2[5])

    v3x.append(v3[0])
    v3y.append(v3[1])
    v3z.append(v3[2])
    w3x.append(v3[3])
    w3y.append(v3[4])
    w3z.append(v3[5])

    t.append(p1[0])

    assert duration > 0



def plot_pos_vel_multi(ax, time, pos, v_list, v_labels, title, ylab_pos, ylab_vel):
    ax.plot(time, pos, label=ylab_pos)
    ax.set_title(title)
    ax.set_ylabel(ylab_pos)
    ax.grid(True)
    ax2 = ax.twinx()
    linestyles = ['-', '--', ':']
    for k, v in enumerate(v_list):
        ls = linestyles[k % len(linestyles)]
        ax2.plot(time, v, linestyle=ls, label=v_labels[k])
    ax2.set_ylabel(ylab_vel)
    lines = ax.get_lines() + ax2.get_lines()
    labels = [l.get_label() for l in lines]
    ax.legend(lines, labels, loc='best')


# ================= 視窗 1：平移（位置 + 三種速度） =================
fig1, axs1 = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
fig1.suptitle('Translation: Position & Velocity vs Time')

plot_pos_vel_multi(axs1[0], t, x, [v1x, v2x, v3x], ['v1','v2','v3'], 'X', 'X [m]', 'V [m/s]')
plot_pos_vel_multi(axs1[1], t, y, [v1y, v2y, v3y], ['v1','v2','v3'], 'Y', 'Y [m]', 'V [m/s]')
plot_pos_vel_multi(axs1[2], t, z, [v1z, v2z, v3z], ['v1','v2','v3'], 'Z', 'Z [m]', 'V [m/s]')
axs1[-1].set_xlabel('Time [s]')
fig1.tight_layout(); fig1.subplots_adjust(top=0.90)

# ================= 視窗 2：旋轉（角度 + 三種角速度） =================
rx_u = np.unwrap(np.array(rx))
ry_u = np.unwrap(np.array(ry))
rz_u = np.unwrap(np.array(rz))

fig2, axs2 = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
fig2.suptitle('Rotation (Euler xyz): Angle & Angular Rate vs Time')

plot_pos_vel_multi(axs2[0], t, rx_u, [w1x, w2x, w3x], ['w1','w2','w3'], 'RX', 'RX [rad]', 'ω [rad/s]')
plot_pos_vel_multi(axs2[1], t, ry_u, [w1y, w2y, w3y], ['w1','w2','w3'], 'RY', 'RY [rad]', 'ω [rad/s]')
plot_pos_vel_multi(axs2[2], t, rz_u, [w1z, w2z, w3z], ['w1','w2','w3'], 'RZ', 'RZ [rad]', 'ω [rad/s]')
axs2[-1].set_xlabel('Time [s]')
fig2.tight_layout(); fig2.subplots_adjust(top=0.90)

plt.show()


