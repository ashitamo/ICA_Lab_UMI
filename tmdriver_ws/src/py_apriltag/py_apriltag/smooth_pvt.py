import math
import numpy as np

def _unit(v, eps=1e-9):
    n = np.linalg.norm(v)
    return v / (n + eps)

def _quat_multiply(q1, q2):
    x1,y1,z1,w1 = q1; x2,y2,z2,w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ])

def _quat_conj(q):
    x,y,z,w = q; return np.array([-x,-y,-z,w])

def _quat_to_axis_angle(q):
    x,y,z,w = q
    w = np.clip(w, -1.0, 1.0)
    angle = 2.0 * math.acos(w)
    s = math.sqrt(max(1.0 - w*w, 0.0))
    if s < 1e-9: return np.array([1.0,0.0,0.0]), 0.0
    return np.array([x/s, y/s, z/s]), angle

def _shortest_quat_delta(q_from, q_to):
    q_rel = _quat_multiply(_quat_conj(q_from), q_to)
    if q_rel[3] < 0: q_rel = -q_rel
    axis, ang = _quat_to_axis_angle(q_rel)
    return axis, ang, q_rel

def _limit_rate_vec(curr, target, a_max, dt):
    # 向量幅值限速（比逐軸好）
    dv = target - curr
    max_step = a_max * max(dt, 0.0)
    m = np.linalg.norm(dv)
    if m <= max_step or max_step <= 0: return target
    return curr + dv * (max_step / m)

def _unwrap_euler(prev_xyz, new_xyz):
    # 使 euler 與上一個連續（加/減 2π）
    out = new_xyz.copy()
    for i in range(3):
        d = out[i] - prev_xyz[i]
        if   d >  np.pi: out[i] -= 2*np.pi
        elif d < -np.pi: out[i] += 2*np.pi
    return out

def _curvature_three_points(p0, p1, p2, eps=1e-6):
    # 三點曲率 κ ≈ 2*| (p1-p0)×(p2-p1) | / ( |p1-p0|*|p2-p1|*|p2-p0| )
    a = p1 - p0; b = p2 - p1; c = p2 - p0
    la, lb, lc = np.linalg.norm(a), np.linalg.norm(b), np.linalg.norm(c)
    if la < eps or lb < eps or lc < eps: return 0.0
    area2 = np.linalg.norm(np.cross(a, b))
    return 2.0 * area2 / (la * lb * lc)  # κ >= 0

class SmoothPVT:
    """
    v_lin, v_ang: 期望最大線/角速度（定速巡航上限）
    a_lin, a_ang: 線/角加速度上限（速度變化限幅）
    a_c_max:     允許的向心加速度上限（曲率限速用）
    min_T:       每段最小時間，避免太短引發抖動
    tau_v:       速度一階濾波時間常數（末端速度再平滑）
    """
    def __init__(self, v_lin=0.08, v_ang=0.8, a_lin=0.25, a_ang=2.0,
                 a_c_max=1.0, min_T=0.20, tau_v=0.30):
        self.v_lin = float(v_lin)
        self.v_ang = float(v_ang)
        self.a_lin = float(a_lin)
        self.a_ang = float(a_ang)
        self.a_c_max = float(a_c_max)
        self.min_T = float(min_T)
        self.tau_v = float(tau_v)
        self.last_v6 = np.zeros(6)
        self.last_euler = None  # for unwrap when exporting

    def _speed_with_curvature(self, p0, p1, p2):
        # 依曲率限速：v ≤ sqrt(a_c_max / κ)
        if p2 is None: return self.v_lin
        kappa = _curvature_three_points(p0, p1, p2)
        if kappa <= 1e-6: return self.v_lin
        v_curve = math.sqrt(max(self.a_c_max, 1e-9) / kappa)
        return min(self.v_lin, v_curve)

    def plan_segment(self, p0_xyz, q0, p1_xyz, q1, p2_xyz=None, q2=None):
        # ====== 1) 小角度死區 + 角速按需求分配 ======
        EPS_ANG = 0.02  # ~1.1°，小於此視為不轉
        axis01, ang01, _ = _shortest_quat_delta(q0, q1)
        if ang01 < EPS_ANG:
            ang01 = 0.0
            axis01 = np.array([0.0,0.0,0.0])

        # ====== 2) 曲率限速 + 自適應最小段時間 ======
        v_lin_allowed = self._speed_with_curvature(p0_xyz, p1_xyz, p2_xyz)
        v_ang_allowed = self.v_ang

        dist = np.linalg.norm(p1_xyz - p0_xyz)
        dir01 = _unit(p1_xyz - p0_xyz) if dist > 1e-12 else np.array([1.0,0.0,0.0])

        T_dist = dist / max(v_lin_allowed, 1e-9)
        T_ang  = (ang01 / max(v_ang_allowed, 1e-9)) if ang01 > 0 else 0.0

        # 自適應最小 T：距離和角度越小，T 仍保有下限，避免毫秒級抽動
        T_adapt_min = max(self.min_T, 0.30*dist + 0.10*ang01)  # 可微調係數
        T = max(T_dist, T_ang, T_adapt_min)

        # ====== 3) 小轉角不做方向混合，避免橫向分量 ======
        BLEND_DEG = 12.0  # 小於 12° 就不要 blend
        if p2_xyz is not None:
            dir12 = _unit(p2_xyz - p1_xyz)
            # 夾角
            cosang = float(np.clip(np.dot(dir01, dir12), -1.0, 1.0))
            turn_deg = math.degrees(math.acos(cosang))
            if turn_deg < BLEND_DEG:
                dir_lin = dir01          # 不混合
            else:
                dir_lin = _unit(dir01 + dir12)  # 只在明顯拐角才平均
        else:
            dir_lin = dir01

        # ====== 4) 角速度按角度/T 分配（而非總給上限）=====
        # 這樣角度很小就不會給過大的 ω
        w_mag = 0.0 if ang01 == 0.0 else min(v_ang_allowed, ang01 / max(T, 1e-6))
        w_vec = w_mag * axis01

        v_lin_vec = v_lin_allowed * dir_lin
        v_target = np.concatenate([v_lin_vec, w_vec])

        # ====== 5) 加速度限幅 + 速度一階濾波（保留你原本的）=====
        v_limited = np.empty_like(v_target)
        v_limited[:3] = _limit_rate_vec(self.last_v6[:3], v_target[:3], self.a_lin, T)
        v_limited[3:] = _limit_rate_vec(self.last_v6[3:], v_target[3:], self.a_ang, T)

        alpha = T / (self.tau_v + T)
        v_next = self.last_v6 + alpha * (v_limited - self.last_v6)
        self.last_v6 = v_next.copy()
        return T, v_next



import pickle
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

def quaternion_average(quaternions):
    """
    Average quaternions properly using weighted method.
    Handles quaternion double-cover.
    """
    # Ensure all quaternions have consistent sign relative to first
    q_aligned = quaternions.copy()
    q_ref = quaternions[0]
    
    for i in range(1, len(q_aligned)):
        if np.dot(q_aligned[i], q_ref) < 0:
            q_aligned[i] = -q_aligned[i]
    
    # Simple average
    q_avg = np.mean(q_aligned, axis=0)
    
    # Normalize to ensure unit quaternion
    q_avg = q_avg / np.linalg.norm(q_avg)
    
    return q_avg


def moving_average_position_and_orientation(poses, window_size=15):
    """
    Calculate moving average for both position (x,y,z) and orientation (quaternion).
    Handles quaternion double-cover issue by ensuring consistent signs.
    
    Args:
        poses: List of tuples (timestamp, pose_array) where pose_array = [x, y, z, qx, qy, qz, qw]
        window_size: Number of recent poses to average
    
    Returns:
        averaged_pose: [x, y, z, qx, qy, qz, qw] or None if not enough data
    """
    if len(poses) < window_size:
        return None
    
    # Extract recent poses
    recent_poses = np.array([p[1] for p in poses[-window_size:]])
    
    # Average position (x, y, z) - can use simple mean
    avg_position = np.mean(recent_poses[:, 0:3], axis=0)
    
    # Average quaternion (qx, qy, qz, qw) - use custom averaging
    quaternions = recent_poses[:, 3:7]  # Extract quaternions
    avg_quaternion = quaternion_average(quaternions)
    
    # Combine position and orientation
    averaged_pose = np.concatenate([avg_position, avg_quaternion])
    return averaged_pose

