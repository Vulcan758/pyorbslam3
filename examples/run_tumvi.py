#!/usr/bin/env python3
"""Run pyorbslam3 on a TUM-VI (EuRoC/mav0 layout) sequence.

Examples:
    python3 examples/run_tumvi.py --mode mono
    python3 examples/run_tumvi.py --mode mono-imu
    python3 examples/run_tumvi.py --mode stereo-imu --viewer
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import pyorbslam3 as pos  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.abspath(os.path.join(HERE, ".."))

MODES = {
    "mono": (pos.Sensor.MONOCULAR, "mono.yaml", False, False),
    "mono-imu": (pos.Sensor.IMU_MONOCULAR, "mono_inertial.yaml", True, False),
    "stereo": (pos.Sensor.STEREO, "stereo.yaml", False, True),
    "stereo-imu": (pos.Sensor.IMU_STEREO, "stereo_inertial.yaml", True, True),
}


def load_cam_csv(cam_dir):
    stamps, files = [], []
    with open(os.path.join(cam_dir, "data.csv")) as f:
        for row in csv.reader(f):
            if not row or row[0].startswith("#"):
                continue
            stamps.append(int(row[0]) * 1e-9)
            files.append(os.path.join(cam_dir, "data", row[1].strip()))
    return np.asarray(stamps), files


def load_imu_csv(imu_dir):
    data = np.loadtxt(os.path.join(imu_dir, "data.csv"), delimiter=",", comments="#")
    data[:, 0] *= 1e-9  # ns -> s; columns already [t, gx, gy, gz, ax, ay, az]
    return data


def load_groundtruth(dataset):
    gt_file = os.path.join(dataset, "mav0", "mocap0", "data.csv")
    if not os.path.isfile(gt_file):
        return None
    gt = np.loadtxt(gt_file, delimiter=",", comments="#")
    gt[:, 0] *= 1e-9
    return gt  # [t, px, py, pz, qw, qx, qy, qz]


def evaluate_ate(traj_file, gt, max_dt=0.02):
    """Associate trajectory (EuRoC format) with mocap and compute ATE RMSE
    after a similarity (Umeyama) alignment. Returns (rmse, scale, n_pairs)."""
    traj = np.loadtxt(traj_file)
    if traj.ndim != 2 or len(traj) < 10:
        return None
    t_est, p_est = traj[:, 0] * 1e-9, traj[:, 1:4]
    idx = np.searchsorted(gt[:, 0], t_est)
    idx = np.clip(idx, 1, len(gt) - 1)
    left_closer = np.abs(gt[idx - 1, 0] - t_est) < np.abs(gt[idx, 0] - t_est)
    idx[left_closer] -= 1
    ok = np.abs(gt[idx, 0] - t_est) < max_dt
    if ok.sum() < 10:
        return None
    src, dst = p_est[ok], gt[idx[ok], 1:4]

    mu_s, mu_d = src.mean(0), dst.mean(0)
    xs, xd = src - mu_s, dst - mu_d
    cov = xd.T @ xs / len(xs)
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    var_s = (xs**2).sum() / len(xs)
    scale = np.trace(np.diag(D) @ S) / var_s
    t = mu_d - scale * R @ mu_s
    err = (scale * (R @ src.T)).T + t - dst
    rmse = float(np.sqrt((err**2).sum(1).mean()))
    return rmse, float(scale), int(ok.sum())


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--mode", choices=MODES, default="mono-imu")
    ap.add_argument(
        "--dataset",
        default=os.path.join(PKG_ROOT, "..", "dataset", "tum", "dataset-room1_512_16"),
    )
    ap.add_argument("--vocab", default=os.path.join(PKG_ROOT, "vocabulary", "ORBvoc.txt"))
    ap.add_argument("--settings", default=None, help="Override settings yaml")
    ap.add_argument("--max-frames", type=int, default=0, help="0 = all frames")
    ap.add_argument("--viewer", action="store_true")
    ap.add_argument("--out-dir", default=os.path.join(PKG_ROOT, "results"))
    args = ap.parse_args()

    sensor, default_yaml, use_imu, use_stereo = MODES[args.mode]
    settings = args.settings or os.path.join(PKG_ROOT, "configs", "tumvi", default_yaml)
    os.makedirs(args.out_dir, exist_ok=True)

    mav0 = os.path.join(args.dataset, "mav0")
    stamps, files0 = load_cam_csv(os.path.join(mav0, "cam0"))
    files1 = None
    if use_stereo:
        stamps1, files1 = load_cam_csv(os.path.join(mav0, "cam1"))
        assert len(stamps1) == len(stamps), "cam0/cam1 not synchronized"
    imu = load_imu_csv(os.path.join(mav0, "imu0")) if use_imu else None

    if use_imu:
        # Skip frames that precede the first IMU measurement.
        first = np.searchsorted(stamps, imu[0, 0], side="right")
        stamps, files0 = stamps[first:], files0[first:]
        if files1:
            files1 = files1[first:]

    n = len(stamps) if args.max_frames <= 0 else min(args.max_frames, len(stamps))
    print(f"[run_tumvi] mode={args.mode} frames={n} settings={settings}")

    slam = pos.Slam(args.vocab, settings, sensor, use_viewer=args.viewer)

    states = []
    t_prev = -np.inf
    imu_init_frame = None
    wall_start = time.time()
    for i in range(n):
        t = stamps[i]
        im0 = cv2.imread(files0[i], cv2.IMREAD_GRAYSCALE)
        if im0 is None:
            print(f"[warn] unreadable image {files0[i]}, skipping")
            continue
        chunk = None
        if use_imu:
            sel = (imu[:, 0] > t_prev) & (imu[:, 0] <= t)
            chunk = imu[sel]
        if use_stereo:
            im1 = cv2.imread(files1[i], cv2.IMREAD_GRAYSCALE)
            res = slam.track_stereo(im0, im1, t, imu=chunk)
        else:
            res = slam.track_monocular(im0, t, imu=chunk)
        t_prev = t
        states.append(int(res.state))
        if use_imu and imu_init_frame is None and slam.imu_initialized():
            imu_init_frame = i
            print(f"[run_tumvi] IMU initialized at frame {i} (t={t - stamps[0]:.1f}s)")
        if i % 250 == 0:
            print(
                f"[run_tumvi] frame {i}/{n} state={res.state} "
                f"tracked_pts={res.num_tracked_points}"
            )
    wall = time.time() - wall_start

    states = np.asarray(states)
    n_ok = int((states == int(pos.TrackingState.OK)).sum())
    print(f"\n=== {args.mode} summary ===")
    print(f"frames processed : {len(states)}  ({len(states) / wall:.1f} fps)")
    print(f"tracking OK      : {n_ok} ({100.0 * n_ok / len(states):.1f}%)")
    if use_imu:
        print(f"IMU initialized  : {'yes, frame ' + str(imu_init_frame) if imu_init_frame is not None else 'NO'}")

    traj = os.path.join(args.out_dir, f"traj_{args.mode}.txt")
    kf_traj = os.path.join(args.out_dir, f"kf_traj_{args.mode}.txt")
    slam.shutdown()
    slam.save_trajectory_euroc(traj)
    slam.save_keyframe_trajectory_euroc(kf_traj)
    print(f"trajectory saved : {traj}")

    gt = load_groundtruth(args.dataset)
    if gt is not None and os.path.isfile(traj):
        result = evaluate_ate(traj, gt)
        if result:
            rmse, scale, pairs = result
            print(f"ATE RMSE (sim3)  : {rmse * 100:.2f} cm over {pairs} poses, scale={scale:.3f}")
        else:
            print("ATE              : not enough associated poses to evaluate")


if __name__ == "__main__":
    main()
