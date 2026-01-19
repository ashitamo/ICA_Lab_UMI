#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
plot_sampling.py
讀取 DemonstrationDataLoader 的資料，根據速度序列決定取樣點，並畫出速度曲線與取樣點。

使用方式（等距空間取樣）：
    python plot_sampling.py --image-folder data/demo_data1/images \
        --traj-json data/demo_data1/trajectory.json \
        --filename-format "frame_{id}.png" \
        --mode space_uniform \
        --dx 0.02 \
        --series filtered

使用方式（速度比例取樣率）：
    python plot_sampling.py --image-folder data/demo_data1/images \
        --traj-json data/demo_data1/trajectory.json \
        --filename-format "frame_{id}.png" \
        --mode rate_proportional \
        --r-min 5.0 --k 12.0 --r-max 50.0 \
        --series filtered
"""
# Add parent directory to path
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import argparse
from typing import Tuple, Optional
import numpy as np
import matplotlib.pyplot as plt

# ---------- 你專案中的資料載入器 ----------
from util.data_loader import DemonstrationDataLoader


# ---------- 小工具 ----------
def _ensure_monotonic(t: np.ndarray):
    if not np.all(np.diff(t) > 0):
        raise ValueError("timestamps 必須嚴格遞增。")

def _cum_trapz_irregular(y: np.ndarray, t: np.ndarray) -> np.ndarray:
    """
    對非等距時間序列做梯形積分的累積值，回傳 s，滿足：
    s[0]=0，s[i] = ∫_{t[0]}^{t[i]} y(τ) dτ
    """
    n = len(t)
    s = np.zeros(n, dtype=float)
    if n <= 1:
        return s
    dt = np.diff(t)
    y_avg = 0.5 * (y[:-1] + y[1:])
    s[1:] = np.cumsum(y_avg * dt)
    return s

def _invert_monotonic(x: np.ndarray, y: np.ndarray, y_targets: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    已知 y(x) 單調遞增（離散點），給定遞增的 y_targets，
    回傳對應的 x_hat（線性內插）與有效 mask。
    """
    idx = np.searchsorted(y, y_targets, side="right")
    mask = (idx > 0) & (idx < len(y))
    idx_valid = idx[mask]
    x0, x1 = x[idx_valid - 1], x[idx_valid]
    y0, y1 = y[idx_valid - 1], y[idx_valid]
    w = (y_targets[mask] - y0) / (y1 - y0 + 1e-15)
    x_hat = x0 + w * (x1 - x0)
    return x_hat, mask

def _times_to_nearest_indices(t_axis: np.ndarray, T: np.ndarray) -> np.ndarray:
    """
    將任意時間 T 映射到最接近的原始時間點索引（方便對應影格）。
    """
    pos = np.searchsorted(t_axis, T)
    pos = np.clip(pos, 1, len(t_axis)-1)
    left = pos - 1
    right = pos
    choose_left = np.abs(t_axis[left] - T) <= np.abs(t_axis[right] - T)
    idx = np.where(choose_left, left, right)
    return idx


# ---------- 方案A：等距空間取樣 ----------
def sample_times_space_uniform(
    timestamps: np.ndarray,
    v_abs: np.ndarray,
    dx: float,
    v_eps: float = 1e-12,
    include_t1: bool = True
) -> Tuple[np.ndarray, np.ndarray]:
    """
    讓相鄰樣點的位移固定為 dx，回傳 (T, idx)。
    - T：取樣時間序列
    - idx：對應到原始時間軸上最接近的索引
    """
    _ensure_monotonic(timestamps)
    if dx <= 0:
        raise ValueError("dx 必須 > 0")

    v = np.abs(v_abs).astype(float)
    v[v < v_eps] = v_eps  # 避免停滯
    s = _cum_trapz_irregular(v, timestamps)

    max_s = s[-1]
    if max_s <= 0:
        # 幾乎沒有移動
        T = np.array([timestamps[0], timestamps[-1]]) if include_t1 else np.array([timestamps[0]])
        idx = _times_to_nearest_indices(timestamps, T)
        return T, idx

    n_steps = int(np.floor(max_s / dx))
    targets = dx * np.arange(0, n_steps + 1)
    T_core, _ = _invert_monotonic(timestamps, s, targets)

    # 確保包含 t0
    if len(T_core) == 0 or T_core[0] > timestamps[0] + 1e-15:
        T_core = np.insert(T_core, 0, timestamps[0])
    # 視需求是否補上 t1
    if include_t1 and (T_core[-1] < timestamps[-1] - 1e-15):
        T_core = np.append(T_core, timestamps[-1])

    idx = _times_to_nearest_indices(timestamps, T_core)
    return T_core, idx


# ---------- 方案B：速度比例取樣率 ----------
def sample_times_rate_proportional(
    timestamps: np.ndarray,
    v_abs: np.ndarray,
    r_min: float,
    k: float,
    r_max: Optional[float] = None,
    v_eps: float = 1e-12,
    include_t1: bool = True
) -> Tuple[np.ndarray, np.ndarray]:
    """
    取樣率 r(t) = r_min + k * |v|（可設定上限 r_max）。
    每當累積 C(t)=∫ r dt 跨過整數 0,1,2,... 取樣（0 對應起點）。
    回傳 (T, idx)。
    """
    _ensure_monotonic(timestamps)
    if r_min < 0 or k < 0:
        raise ValueError("r_min 與 k 需為非負。")

    v = np.abs(v_abs).astype(float)
    v[v < v_eps] = v_eps
    r = r_min + k * v
    if r_max is not None:
        r = np.minimum(r, r_max)

    C = _cum_trapz_irregular(r, timestamps)
    max_C = C[-1]

    m_steps = int(np.floor(max_C))
    targets = np.arange(0, m_steps + 1, dtype=float)  # 包含 0

    T_core, _ = _invert_monotonic(timestamps, C, targets)
    if len(T_core) == 0 or T_core[0] > timestamps[0] + 1e-15:
        T_core = np.insert(T_core, 0, timestamps[0])
    if include_t1 and (T_core[-1] < timestamps[-1] - 1e-15):
        T_core = np.append(T_core, timestamps[-1])

    idx = _times_to_nearest_indices(timestamps, T_core)
    return T_core, idx


def sample_times_rate_proportional_opt(
    timestamps: np.ndarray,
    v_abs: np.ndarray,
    *,
    r_min: float,
    k: float,
    r_max: Optional[float] = None,
    start_idx: int = 0,
    end_idx: Optional[int] = None,
    head_tail_margin_s: float = 0.0,
    v_eps: float = 1e-12,
    min_dt: Optional[float] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    速度比例取樣（僅在指定索引區段內進行，並避開頭尾 margin）。
    r(t) = r_min + k*|v(t)|，可設 r_max 上限。
    只在 [timestamps[start_idx], timestamps[end_idx-1]] 之間取樣，
    但會再去掉兩端 head_tail_margin_s 的區間，不在留白區內取樣。

    參數
    ----
    timestamps : 單調嚴格遞增的一維時間陣列 (N,)
    v_abs      : 與 timestamps 對齊的速度絕對值 (N,)
    r_min      : 最低取樣率（Hz，非負）
    k          : 取樣率係數（非負）
    r_max      : 取樣率上限（Hz，可為 None 表示無上限）
    start_idx  : 取樣區段起始索引（含）
    end_idx    : 取樣區段結束索引（不含）。None 則用 len(timestamps)
    head_tail_margin_s : 區段頭尾各留白秒數（不取樣）
    v_eps      : 速度下限，避免 0 導致 C(t) 平坦
    min_dt     : 取樣點間的最小時間間隔（秒）；None 表示不做此約束

    回傳
    ----
    T   : 取樣時間（絕對時間）一維陣列
    idx : 每個 T 在原 timestamps 上最近的索引
    """
    _ensure_monotonic(timestamps)
    if r_min < 0 or k < 0:
        raise ValueError("r_min 與 k 必須為非負。")
    n = len(timestamps)
    if end_idx is None:
        end_idx = n
    if not (0 <= start_idx < end_idx <= n):
        raise ValueError("start_idx/end_idx 越界或區間為空。")

    # 區段資料（仍保留「絕對時間」）
    t_seg = timestamps[start_idx:end_idx]
    v_seg = np.abs(v_abs[start_idx:end_idx]).astype(float)
    if len(t_seg) < 2:
        return np.array([]), np.array([], dtype=int)

    v_seg[v_seg < v_eps] = v_eps
    r = r_min + k * v_seg
    if r_max is not None:
        r = np.minimum(r, r_max)

    # 對區段做 C(t) 累積積分（以 t_seg[0] 為 0）
    C = _cum_trapz_irregular(r, t_seg)  # C[0]=0, C 單調遞增

    # 有效取樣視窗：避開頭尾
    t_start_eff = t_seg[0] + max(0.0, head_tail_margin_s)
    t_end_eff   = t_seg[-1] - max(0.0, head_tail_margin_s)
    if not (t_start_eff < t_end_eff):
        # margin 太大，沒有可取樣區間
        return np.array([]), np.array([], dtype=int)

    # 找對應的 C_start, C_end（用內插）
    C_start_arr, m1 = _invert_monotonic(t_seg, C, np.array([t_start_eff]))
    C_end_arr,   m2 = _invert_monotonic(t_seg, C, np.array([t_end_eff]))
    if not (m1.any() and m2.any()):
        # 萬一 C 在這段內沒有變化（極端情況），無取樣
        return np.array([]), np.array([], dtype=int)
    C_start = C_start_arr[0]
    C_end   = C_end_arr[0]

    # 目標整數門檻：ceil(C_start) ... floor(C_end)
    lo = int(np.ceil(C_start - 1e-12))
    hi = int(np.floor(C_end + 1e-12))
    if hi < lo:
        return np.array([]), np.array([], dtype=int)
    targets = np.arange(lo, hi + 1, dtype=float)

    # 反解 t：C(t) = targets
    T_raw, mask = _invert_monotonic(t_seg, C, targets)
    if T_raw.size == 0:
        return np.array([]), np.array([], dtype=int)

    # 可選：最小時間間隔去抖
    if min_dt is not None and min_dt > 0:
        T_filt = [T_raw[0]]
        for t in T_raw[1:]:
            if t - T_filt[-1] >= min_dt:
                T_filt.append(t)
        T_out = np.array(T_filt)
    else:
        T_out = T_raw

    # 對應回原序列最近索引
    idx = _times_to_nearest_indices(timestamps, T_out)
    return T_out, idx

# ---------- 主程式 ----------
def main():
    parser = argparse.ArgumentParser(description="Plot speed vs time with sampling points.")
    parser.add_argument("--series", choices=["raw", "filtered"], default="filtered",
                        help="使用原始速度（raw）或濾波速度（filtered）")
    parser.add_argument("--mode", choices=["space_uniform", "rate_proportional"], default="rate_proportional",
                        help="取樣策略")

    # 方案A參數
    parser.add_argument("--dx", type=float, default=0.01, help="等距空間取樣的步長 Δx")
    # 方案B參數
    parser.add_argument("--r-min", type=float, default=0.5, help="最低取樣率（Hz）")
    parser.add_argument("--k", type=float, default=20.0, help="取樣率係數（乘在 |v| 上）")
    parser.add_argument("--r-max", type=float, default=30, help="最大取樣率（Hz），設為 0 代表不設上限")
    # 圖檔
    parser.add_argument("--out", default="sampling_plot.png", help="輸出圖檔名稱")
    args = parser.parse_args()

    # 載入資料
    loader = DemonstrationDataLoader(
        image_folder="data/demo_data1/images",
        trajectory_json_path="data/demo_data1/trajectory.json",
        filename_format="frame_{id}.png"
    )
    loader.load_all_data()
    timestamps = np.array([frame.timestamp for frame in loader.frames], dtype=float)
    v_abs_values = np.array([frame.v_abs for frame in loader.frames], dtype=float)
    v_abs_filtered_values = np.array([frame.v_abs_filtered for frame in loader.frames], dtype=float)
    print(f"[INFO] 載入資料點數: {len(timestamps)}")

    # 選擇速度序列
    v_used = v_abs_filtered_values if args.series == "filtered" else v_abs_values

    # 產生取樣點
    if args.mode == "space_uniform":
        T, idx = sample_times_space_uniform(
            timestamps, v_used, dx=float(args.dx), include_t1=True
        )
        mode_desc = f"space_uniform (dx={args.dx})"
    else:
        r_max = None if (args.r_max is not None and float(args.r_max) <= 0) else float(args.r_max)
        T, idx = sample_times_rate_proportional(
            timestamps, v_used, r_min=float(args.r_min), k=float(args.k), r_max=r_max, include_t1=True
        )
        mode_desc = f"rate_proportional (r_min={args.r_min}, k={args.k}, r_max={'None' if r_max is None else r_max})"

    # 畫圖
    plt.figure(figsize=(12, 5))
    plt.plot(timestamps, v_abs_filtered_values, label="v_abs_filtered")
    plt.scatter(T, v_used[idx], s=24, label="sample points")
    plt.xlabel("Time [s]")
    plt.ylabel("Speed |v| [unit/s]")
    plt.title(f"Filtered Speed vs Time with Sampling Points\nmode={mode_desc}, series={args.series}")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(args.out, dpi=150)
    print(f"[OK] 取樣點數: {len(T)}，圖已輸出到: {args.out}")

    # 若你想直接顯示，取消註解下一行：
    # plt.show()


if __name__ == "__main__":
    main()