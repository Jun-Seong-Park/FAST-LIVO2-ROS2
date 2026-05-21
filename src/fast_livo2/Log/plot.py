#!/usr/bin/env python3
"""
FAST-LIVO2 상태/IMU 로그 시각화.

데이터 포맷 (LIVMapper.cpp::handleLIO/handleVIO 기준)
  mat_pre.txt  (19 cols)  : ESIKF 갱신 *전* 상태 (forward propagation 결과)
  mat_out.txt  (20 cols)  : ESIKF 갱신 *후* 상태

  col  0      : time [s]   (= last_lio_update_time - _first_lidar_time)
  col  1- 3   : euler RPY [deg]      (RotMtoEuler(_state.rot_end) * 57.3)
  col  4- 6   : position  [m]        (_state.pos_end)
  col  7- 9   : velocity  [m/s]      (_state.vel_end)
  col 10-12   : bias_gyro [rad/s]    (_state.bias_g)
  col 13-15   : bias_acc  [m/s^2]    (_state.bias_a)
  col 16      : inv_expo_time        (_state.inv_expo_time)
  col 17-18   : 0 padding             (V3D(inv_expo_time,0,0) 의 빈 슬롯)
  col 19      : feats_undistort size (mat_out 만)

  imu.txt   (7 cols) : col 0 time,  1-3 gyro,  4-6 acc
"""

import os
import sys
import numpy as np
import matplotlib

matplotlib.use("Agg")  # headless 환경 대응
import matplotlib.pyplot as plt

LOG_DIR = os.path.dirname(os.path.abspath(__file__))


def load(name, expected_cols=None):
    path = os.path.join(LOG_DIR, name)
    if not os.path.isfile(path) or os.path.getsize(path) == 0:
        print(f"[skip] {name}: 파일 없음/비어있음", file=sys.stderr)
        return None
    a = np.loadtxt(path)
    if a.ndim == 1:
        a = a.reshape(1, -1)
    if expected_cols is not None and a.shape[1] < expected_cols:
        print(
            f"[warn] {name}: cols={a.shape[1]} < expected {expected_cols}",
            file=sys.stderr,
        )
    return a


def plot_state(pre, out, save_path):
    """attitude / translation / velocity / bg / ba / exposure 6 subplot."""
    fig, axs = plt.subplots(3, 2, figsize=(16, 11), sharex=True)

    groups = [
        ("Attitude (deg)",      slice(1, 4),    ["roll", "pitch", "yaw"], (0, 0)),
        ("Translation (m)",     slice(4, 7),    ["x", "y", "z"],          (1, 0)),
        ("Velocity (m/s)",      slice(7, 10),   ["vx", "vy", "vz"],       (2, 0)),
        ("Gyro bias (rad/s)",   slice(10, 13),  ["bgx", "bgy", "bgz"],    (0, 1)),
        ("Accel bias (m/s^2)",  slice(13, 16),  ["bax", "bay", "baz"],    (1, 1)),
    ]
    colors = ["tab:red", "tab:green", "tab:blue"]

    t_pre = pre[:, 0] if pre is not None else None
    t_out = out[:, 0] if out is not None else None

    for title, sl, comp_labels, (r, c) in groups:
        ax = axs[r, c]
        ax.set_title(title)
        for k, (cl, col) in enumerate(zip(comp_labels, colors)):
            if pre is not None and pre.shape[1] > sl.stop - 1:
                ax.plot(t_pre, pre[:, sl][:, k], "-",  color=col, alpha=0.45,
                        linewidth=0.9, label=f"pre {cl}")
            if out is not None and out.shape[1] > sl.stop - 1:
                ax.plot(t_out, out[:, sl][:, k], "-", color=col, alpha=1.0,
                        linewidth=1.1, label=f"out {cl}")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8, ncol=2, loc="best")

    # exposure
    ax = axs[2, 1]
    ax.set_title("Inverse Exposure Time")
    if pre is not None and pre.shape[1] > 16:
        ax.plot(t_pre, pre[:, 16], "-", color="tab:gray",   alpha=0.6,
                linewidth=0.9, label="pre")
    if out is not None and out.shape[1] > 16:
        ax.plot(t_out, out[:, 16], "-", color="tab:orange", alpha=1.0,
                linewidth=1.1, label="out")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, loc="best")

    for r in range(3):
        axs[r, 0].set_ylabel("value")
    axs[2, 0].set_xlabel("time [s]")
    axs[2, 1].set_xlabel("time [s]")

    fig.suptitle("FAST-LIVO2 State (pre = before ESIKF update, out = after)", fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(save_path, dpi=200)
    plt.close(fig)
    print(f"[ok] saved {save_path}")


def plot_imu(imu, save_path):
    fig, axs = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    t = imu[:, 0]
    gyr_lab = ["gyr-x", "gyr-y", "gyr-z"]
    acc_lab = ["acc-x", "acc-y", "acc-z"]
    colors  = ["tab:red", "tab:green", "tab:blue"]

    axs[0].set_title("Gyroscope (rad/s)")
    for i in range(3):
        axs[0].plot(t, imu[:, 1 + i], "-", color=colors[i],
                    linewidth=0.7, alpha=0.9, label=gyr_lab[i])
    axs[0].grid(True, alpha=0.3)
    axs[0].legend(loc="best", fontsize=9)

    axs[1].set_title("Accelerometer (m/s^2)")
    for i in range(3):
        axs[1].plot(t, imu[:, 4 + i], "-", color=colors[i],
                    linewidth=0.7, alpha=0.9, label=acc_lab[i])
    axs[1].grid(True, alpha=0.3)
    axs[1].legend(loc="best", fontsize=9)
    axs[1].set_xlabel("time [s]")

    fig.suptitle("IMU Raw Measurements", fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(save_path, dpi=200)
    plt.close(fig)
    print(f"[ok] saved {save_path}")


def plot_trajectory(out, save_path):
    """평면 궤적 (x-y) + 고도 (z vs time)."""
    if out is None or out.shape[1] < 7:
        return
    fig, axs = plt.subplots(1, 2, figsize=(15, 6))
    t = out[:, 0]
    x, y, z = out[:, 4], out[:, 5], out[:, 6]

    sc = axs[0].scatter(x, y, c=t, cmap="viridis", s=4)
    axs[0].plot(x, y, "-", color="gray", alpha=0.3, linewidth=0.6)
    axs[0].set_title("Trajectory (top-down x-y)")
    axs[0].set_xlabel("x [m]")
    axs[0].set_ylabel("y [m]")
    axs[0].set_aspect("equal", adjustable="datalim")
    axs[0].grid(True, alpha=0.3)
    fig.colorbar(sc, ax=axs[0], label="time [s]")

    axs[1].plot(t, z, "-", color="tab:blue", linewidth=1.0)
    axs[1].set_title("Altitude (z vs time)")
    axs[1].set_xlabel("time [s]")
    axs[1].set_ylabel("z [m]")
    axs[1].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(save_path, dpi=200)
    plt.close(fig)
    print(f"[ok] saved {save_path}")


def plot_features(out, save_path):
    """ESIKF 후 feats_undistort size (mat_out col 19)."""
    if out is None or out.shape[1] < 20:
        return
    fig, ax = plt.subplots(figsize=(14, 4))
    ax.plot(out[:, 0], out[:, 19], "-", color="tab:purple", linewidth=0.8)
    ax.set_title("Effective LiDAR Feature Count per Frame")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("# feats_undistort")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(save_path, dpi=200)
    plt.close(fig)
    print(f"[ok] saved {save_path}")


def main():
    pre = load("mat_pre.txt", 19)
    out = load("mat_out.txt", 20)
    imu = load("imu.txt", 7)

    if pre is not None or out is not None:
        plot_state(pre, out, os.path.join(LOG_DIR, "state.png"))
        plot_trajectory(out, os.path.join(LOG_DIR, "trajectory.png"))
        plot_features(out, os.path.join(LOG_DIR, "features.png"))
    if imu is not None:
        plot_imu(imu, os.path.join(LOG_DIR, "imu.png"))


if __name__ == "__main__":
    main()
