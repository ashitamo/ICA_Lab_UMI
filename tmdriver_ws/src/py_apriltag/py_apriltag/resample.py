#!/usr/bin/env python3
"""
Resample and time-scale a pose path (CSV).

Input format (CSV, no header):
    sec.nanosec,x,y,z,qx,qy,qz,qw

Output format (CSV, no header):
    sec.nanosec,x,y,z,qx,qy,qz,qw

Features
--------
- Multiple output frequencies in a single run: --freqs 30 60 120
- Playback speed scaling: --speeds 0.5 1.0 2.0
  * speed > 1.0  → faster playback (shorter total duration)
  * speed < 1.0  → slower playback (longer total duration)
- Positions: linear interpolation
- Orientations: quaternion SLERP with hemisphere alignment
- Output timestamps:
  * By default, keep the absolute start time equal to the input's first timestamp
    and scale total duration by 1/speed: t_out = t0 + k*(1/freq)
  * Source sampling time is t_src = t0_in + speed * (t_out - t0_out)
  * Optionally use --start zero to start from 0.000000000

Usage
-----
    python resample_and_timescale_path.py path.txt \
        --freqs 60 120 \
        --speeds 0.5 1.0 2.0 \
        --start absolute
"""
import argparse
import csv
import math
from typing import Tuple
import numpy as np
from pathlib import Path


# ----------------------------- I/O ---------------------------------
def read_path_csv(path: str) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    ts, pos, quat = [], [], []
    with open(path, 'r', newline='') as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].strip().startswith('#'):
                continue
            try:
                t = float(row[0])
                x, y, z = float(row[1]), float(row[2]), float(row[3])
                qx, qy, qz, qw = float(row[4]), float(row[5]), float(row[6]), float(row[7])
            except (IndexError, ValueError):
                continue
            ts.append(t)
            pos.append([x, y, z])
            quat.append([qx, qy, qz, qw])

    if len(ts) < 2:
        raise ValueError("Need at least 2 valid samples to resample.")

    ts = np.asarray(ts, dtype=float)
    pos = np.asarray(pos, dtype=float)
    quat = np.asarray(quat, dtype=float)

    # sort by time and deduplicate exact timestamp repeats
    order = np.argsort(ts)
    ts, pos, quat = ts[order], pos[order], quat[order]
    uniq_idx = np.concatenate(([0], np.where(np.diff(ts) > 0)[0] + 1))
    return ts[uniq_idx], pos[uniq_idx], quat[uniq_idx]


def write_path_csv(path: str, ts: np.ndarray, pos: np.ndarray, quat: np.ndarray) -> None:
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        for t, p, q in zip(ts, pos, quat):
            sec = int(math.floor(t))
            ns  = int(round((t - sec) * 1e9))
            if ns >= 1_000_000_000:
                sec += 1
                ns -= 1_000_000_000
            writer.writerow([
                f"{sec}.{ns:09d}",
                f"{p[0]:.9f}",
                f"{p[1]:.9f}",
                f"{p[2]:.9f}",
                f"{q[0]:.9f}",
                f"{q[1]:.9f}",
                f"{q[2]:.9f}",
                f"{q[3]:.9f}",
            ])


# ----------------------------- Math utils --------------------------
def normalize_quat(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q, axis=-1, keepdims=True)
    n = np.where(n == 0.0, 1.0, n)
    return q / n


def slerp(q0: np.ndarray, q1: np.ndarray, u: float) -> np.ndarray:
    q0 = normalize_quat(q0.reshape(4))
    q1 = normalize_quat(q1.reshape(4))
    dot = float(np.dot(q0, q1))
    # hemisphere alignment
    if dot < 0.0:
        q1 = -q1
        dot = -dot

    DOT_THRESH = 0.9995
    if dot > DOT_THRESH:
        out = (1.0 - u) * q0 + u * q1
        return normalize_quat(out)

    theta0 = math.acos(max(-1.0, min(1.0, dot)))
    sin_theta0 = math.sin(theta0)
    theta = theta0 * u
    s0 = math.sin(theta0 - theta) / sin_theta0
    s1 = math.sin(theta) / sin_theta0
    return s0 * q0 + s1 * q1


def piecewise_linear_interp(ts_in: np.ndarray, y_in: np.ndarray, t: float) -> np.ndarray:
    """ Interpolate y at time t, assuming ts_in is strictly increasing. """
    if t <= ts_in[0]:
        return y_in[0]
    if t >= ts_in[-1]:
        return y_in[-1]
    j = int(np.searchsorted(ts_in, t) - 1)
    t0, t1 = ts_in[j], ts_in[j+1]
    u = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
    return (1.0 - u) * y_in[j] + u * y_in[j+1]


def piecewise_slerp(ts_in: np.ndarray, q_in: np.ndarray, t: float) -> np.ndarray:
    """ Quaternion SLERP at time t using bracketing segment. """
    if t <= ts_in[0]:
        return q_in[0]
    if t >= ts_in[-1]:
        return q_in[-1]
    j = int(np.searchsorted(ts_in, t) - 1)
    t0, t1 = ts_in[j], ts_in[j+1]
    u = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
    q0, q1 = q_in[j], q_in[j+1]
    # safety hemisphere alignment
    if np.dot(q0, q1) < 0:
        q1 = -q1
    return slerp(q0, q1, float(u))


# ----------------------------- Core logic --------------------------
def resample_times(ts_in: np.ndarray, freq: float, speed: float, start_mode: str) -> np.ndarray:
    """ Build output timestamps for a given freq and speed.
        Dur_out = (tN - t0) / speed
        If start_mode == 'absolute', start at ts_in[0]
        If start_mode == 'zero', start at 0.0
    """
    assert freq > 0 and speed > 0
    t0, tN = ts_in[0], ts_in[-1]
    dur_in = tN - t0
    dur_out = dur_in / speed

    dt_ns = int(round((1.0 / freq) * 1e9))
    if start_mode == 'absolute':
        cur_ns = int(round(t0 * 1e9))
    else:
        cur_ns = 0

    out = []
    end_ns = cur_ns + int(round(dur_out * 1e9))
    while True:
        out.append(cur_ns / 1e9)
        cur_ns += dt_ns
        if cur_ns > end_ns:
            break
    return np.asarray(out, dtype=float)


def resample_and_timescale(ts_in: np.ndarray, pos_in: np.ndarray, quat_in: np.ndarray,
                           freq: float, speed: float, start_mode: str) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    # Pre-normalize and enforce hemisphere continuity on input quaternions to stabilize
    quat_in = normalize_quat(quat_in.copy())
    for i in range(1, len(quat_in)):
        if np.dot(quat_in[i-1], quat_in[i]) < 0:
            quat_in[i] = -quat_in[i]

    ts_out = resample_times(ts_in, freq=freq, speed=speed, start_mode=start_mode)

    # Map each output time to a source time on the original trajectory:
    #   t_src = t0_in + speed * (t_out - t0_out)
    t0_in = ts_in[0]
    t0_out = ts_in[0] if start_mode == 'absolute' else 0.0
    t_src = t0_in + speed * (ts_out - t0_out)

    # Interpolate at each t_src
    pos_out = np.stack([piecewise_linear_interp(ts_in, pos_in, t) for t in t_src], axis=0)
    quat_out = np.stack([piecewise_slerp(ts_in, quat_in, t) for t in t_src], axis=0)
    quat_out = normalize_quat(quat_out)
    return ts_out, pos_out, quat_out


# ----------------------------- Main --------------------------------
def main():
    ap = argparse.ArgumentParser(description="Resample and time-scale a path file with SLERP orientations.")
    ap.add_argument("--input",required=False, default="darwcircle/draw_circle_01.txt",type=str, help="Input CSV path (sec.nanosec,x,y,z,qx,qy,qz,qw)")
    ap.add_argument("--freqs", type=float, nargs='+', default=[90.0], help="Output frequencies in Hz (one or more)")
    ap.add_argument("--speeds", type=float, nargs='+', default=[1.0], help="Playback speeds (one or more). >1=faster")
    ap.add_argument("--start", choices=["absolute","zero"], default="absolute",
                    help="absolute: keep original start time; zero: start at 0.0")
    ap.add_argument("--outdir", type=str, default=None, help="Optional output directory")
    args = ap.parse_args()

    ts, pos, quat = read_path_csv(args.input)
    base = Path(args.input).with_suffix('')
    outdir = Path(args.outdir) if args.outdir else base.parent

    freqs = [f for f in args.freqs if f > 0]
    speeds = [s for s in args.speeds if s > 0]
    if not freqs:
        raise ValueError("No valid frequencies > 0 were provided.")
    if not speeds:
        raise ValueError("No valid speeds > 0 were provided.")

    for f in freqs:
        for s in speeds:
            ts_out, pos_out, quat_out = resample_and_timescale(ts, pos, quat, freq=f, speed=s, start_mode=args.start)
            out_name = f"{base.name}_resampled_{int(round(f))}Hz_speed{str(s).replace('.','p')}x.txt"
            out_path = outdir / out_name
            write_path_csv(out_path, ts_out, pos_out, quat_out)
            print(f"Wrote: {out_path}  |  N={len(ts_out)}  |  freq={f} Hz  speed={s}x  start={args.start}")


if __name__ == "__main__":
    main()
