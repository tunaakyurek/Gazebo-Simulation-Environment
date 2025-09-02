#!/usr/bin/env python3
import json
import glob
import os
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt


def load_latest_log():
    files = sorted(glob.glob('px4_ekf_data_*.json'), key=os.path.getmtime)
    if not files:
        raise FileNotFoundError('No px4_ekf_data_*.json files found')
    with open(files[-1], 'r') as f:
        data = json.load(f)
    return files[-1], data


def to_np_series(data, key):
    seq = [d.get(key) for d in data]
    seq = [x for x in seq if x is not None]
    return np.array(seq) if seq else None


def main():
    log_file, data = load_latest_log()
    ts = np.array([d['timestamp'] for d in data])
    t0 = ts[0] if len(ts) else 0.0
    t = ts - t0

    true_pos = to_np_series(data, 'true_position')
    ekf_pos = to_np_series(data, 'ekf_position')
    true_vel = to_np_series(data, 'true_velocity')
    ekf_vel = to_np_series(data, 'ekf_velocity')

    # Position comparison
    if ekf_pos is not None:
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        axes = axes.reshape(2, 2)

        # 3D Trajectory
        ax3d = fig.add_subplot(2, 2, 1, projection='3d')
        if true_pos is not None and len(true_pos) == len(ekf_pos):
            ax3d.plot(true_pos[:, 0], true_pos[:, 1], true_pos[:, 2], 'b-', label='True')
        ax3d.plot(ekf_pos[:, 0], ekf_pos[:, 1], ekf_pos[:, 2], 'r--', label='EKF')
        ax3d.set_title('3D Trajectory')
        ax3d.set_xlabel('X (m)'); ax3d.set_ylabel('Y (m)'); ax3d.set_zlabel('Z (m)')
        ax3d.legend();

        # Position vs time
        ax = axes[0, 1]
        if true_pos is not None and len(true_pos) == len(ekf_pos):
            ax.plot(t, true_pos[:, 0], 'b-', alpha=0.6, label='True X')
            ax.plot(t, true_pos[:, 1], 'g-', alpha=0.6, label='True Y')
            ax.plot(t, true_pos[:, 2], 'k-', alpha=0.6, label='True Z')
        ax.plot(t, ekf_pos[:, 0], 'r--', label='EKF X')
        ax.plot(t, ekf_pos[:, 1], 'm--', label='EKF Y')
        ax.plot(t, ekf_pos[:, 2], 'c--', label='EKF Z')
        ax.set_title('Position vs Time'); ax.set_xlabel('t (s)'); ax.set_ylabel('pos (m)'); ax.grid(True); ax.legend()

        # Velocity vs time
        ax = axes[1, 0]
        if true_vel is not None and len(true_vel) == len(ekf_vel):
            ax.plot(t, true_vel[:, 0], 'b-', alpha=0.6, label='True Vx')
            ax.plot(t, true_vel[:, 1], 'g-', alpha=0.6, label='True Vy')
            ax.plot(t, true_vel[:, 2], 'k-', alpha=0.6, label='True Vz')
        if ekf_vel is not None:
            ax.plot(t[:len(ekf_vel)], ekf_vel[:, 0], 'r--', label='EKF Vx')
            ax.plot(t[:len(ekf_vel)], ekf_vel[:, 1], 'm--', label='EKF Vy')
            ax.plot(t[:len(ekf_vel)], ekf_vel[:, 2], 'c--', label='EKF Vz')
        ax.set_title('Velocity vs Time'); ax.set_xlabel('t (s)'); ax.set_ylabel('vel (m/s)'); ax.grid(True); ax.legend()

        # Position error
        ax = axes[1, 1]
        if true_pos is not None and len(true_pos) == len(ekf_pos):
            err = ekf_pos - true_pos
            ax.plot(t, np.linalg.norm(err, axis=1), 'r-')
            ax.set_title('Position Error Norm'); ax.set_xlabel('t (s)'); ax.set_ylabel('m'); ax.grid(True)
        else:
            ax.text(0.5, 0.5, 'No True Position Available', ha='center', va='center')
            ax.set_axis_off()

        plt.tight_layout()
        out = f"px4_ekf_analysis_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
        plt.savefig(out, dpi=200, bbox_inches='tight')
        print(f"Saved {out}")
    else:
        print('EKF positions not found in log.')


if __name__ == '__main__':
    main()
