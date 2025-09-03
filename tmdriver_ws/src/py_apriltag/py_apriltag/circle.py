#!/usr/bin/env python3
"""
circle_path_generator.py

產生一段在 X-Y 平面畫圓的軌跡，並寫入 path.txt
 - 取樣頻率固定 30 Hz（Δt = 1/30 s）
 - 可選「均速」或「漸加速→漸減速」兩種速度曲線
 - 可選「Look-At」模式：機體 +Z 軸始終指向 TARGET_POINT
"""
import math
from pathlib import Path
from datetime import datetime
import numpy as np
from scipy.spatial.transform import Rotation as R

# ========= 使用者可調參數 =========
RATE_HZ        = 90.0         # 取樣頻率
TOTAL_TIME_S   = 10.0         # 總時間（秒）
NUM_REV        = 1.0           # 繞圈圈數（1 = 一圈）
RADIUS         = 0.12          # 圓半徑（公尺）
CENTER_XY      = (0.0, -0.40)  # 圓心 (cx, cy)
Z_CONST        = 0.4         # Z 高度
PROFILE        = "ease"        # "constant" 或 "ease"

# ---- Look-At 相關 ----
LOOK_AT_TARGET = False                         # True 開啟 / False 關閉
TARGET_POINT   = np.array([0.17, -0.40, 0.00])  # 被「注視」的目標點 (tx, ty, tz)
UP_HINT        = np.array([0, 0, 1])             # 建立右手坐標系的參考向量
# =================================

DT   = 1.0 / RATE_HZ
N    = int(round(TOTAL_TIME_S * RATE_HZ)) + 1
CX, CY = CENTER_XY

# ---------- 補助函式 ----------
def smoothstep(p: float) -> float:
    return 0.5 * (1 - math.cos(math.pi * p))

def progress(idx: int) -> float:
    p = idx / (N - 1)
    if PROFILE == "constant":
        return p
    elif PROFILE == "ease":
        return smoothstep(p)
    else:
        raise ValueError("PROFILE 必須是 'constant' 或 'ease'")

def yaw_quat(yaw: float):
    return R.from_euler('z', yaw).as_quat()     # (x,y,z,w)

def look_at_quat(position: np.ndarray, target: np.ndarray, up_hint: np.ndarray):
    """
    建立一個右手坐標系：
      +Z 軸   → 指向 target
      +X 軸   → 與 up_hint 垂直、位於當前軌跡切線平面內（或任意合理方向）
      +Y 軸   → Z × X
    """
    z_axis = target - position
    z_axis /= np.linalg.norm(z_axis)

    # 若 up_hint 與 z_axis 近共線，選一個新的 up_hint
    if abs(np.dot(z_axis, up_hint) / np.linalg.norm(up_hint)) > 0.99:
        up_hint = np.array([1, 0, 0])

    x_axis = np.cross(up_hint, z_axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)

    rot_m = np.column_stack((x_axis, y_axis, z_axis))  # 列向量是軸
    return R.from_matrix(rot_m).as_quat()              # (x,y,z,w)
# --------------------------------

def write_path():
    path_file = Path("circle.txt").resolve()
    with path_file.open("w", encoding="utf-8") as f:
        p0 = np.array([CX + RADIUS+0.1, CY + 0.1, Z_CONST,0,0,0,0])
        p0[3:] = R.from_euler('xyz',[180,0,0], degrees=True).as_quat()
        start = 0.0
        t_abs = start
        sec   = int(t_abs)
        nsec  = int((t_abs - sec) * 1e9)
        f.write(f"{sec}.{nsec:09d},{p0[0]},{p0[1]},{p0[2]},{p0[3]},{p0[4]},{p0[5]},{p0[6]}\n") 
        
        p1 = np.array([CX + RADIUS + 0.1, CY, Z_CONST,0,0,0,0])
        p1[3:] = R.from_euler('xyz',[180,0,0], degrees=True).as_quat()
        start = 0.5
        t_abs = start
        sec   = int(t_abs)
        nsec  = int((t_abs - sec) * 1e9)
        f.write(f"{sec}.{nsec:09d},{p1[0]},{p1[1]},{p1[2]},{p1[3]},{p1[4]},{p1[5]},{p1[6]}\n")
        
        p2 = np.array([CX + RADIUS, CY, Z_CONST,0,0,0,0])
        p2[3:] = R.from_euler('xyz',[180,0,0], degrees=True).as_quat()
        start = 1.0
        t_abs = start
        sec   = int(t_abs)
        nsec  = int((t_abs - sec) * 1e9)
        f.write(f"{sec}.{nsec:09d},{p2[0]},{p2[1]},{p2[2]},{p2[3]},{p2[4]},{p2[5]},{p2[6]}\n")
        
        start = 1.5
        for i in range(N):
            # ---- 時間戳 ----
            t_abs = start + i * DT
            sec   = int(t_abs)
            nsec  = int((t_abs - sec) * 1e9)

            # ---- 位置 ----
            s      = progress(i)
            theta  = s * NUM_REV * 2.0 * math.pi
            x      = CX + RADIUS * math.cos(theta)
            y      = CY + RADIUS * math.sin(theta)
            z      = Z_CONST
            pos    = np.array([x, y, z])

            # ---- 姿態 ----
            if LOOK_AT_TARGET:
                qx, qy, qz, qw = look_at_quat(pos, TARGET_POINT, UP_HINT)
            else:
                qx, qy, qz, qw  = R.from_euler('xyz', (180, 0, 0), degrees=True).as_quat()

            # ---- 寫檔 ----
            f.write(f"{sec}.{nsec:09d},{x},{y},{z},{qx},{qy},{qz},{qw}\n")

    print(f"✔ 已寫入 {path_file}，共 {N} 筆資料，取樣間隔 {DT:.3f}s")

if __name__ == "__main__":
    write_path()
