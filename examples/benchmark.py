#!/usr/bin/env python3
"""Benchmark pyorbslam3 on public datasets and score against ground truth.

Supported dataset layouts (--dataset):
  tumvi     TUM-VI  (EuRoC/mav0 layout, 512x512 fisheye, mocap0 ground truth)
  euroc     EuRoC MAV (mav0 layout, state_groundtruth_estimate0)
  kitti     KITTI raw "synced+rectified" drive (image_00/image_01, optional
            odometry poses file for ground truth, else OXTS)
  tum_rgbd  TUM RGB-D (rgb.txt / depth.txt / groundtruth.txt)

Modes (--mode): mono, mono-imu, stereo, stereo-imu, rgbd (availability per dataset).

Each run appends one JSON line to <out-dir>/benchmarks.jsonl and writes the
EuRoC-format trajectory; make_report.py turns the JSONL into BENCHMARKS.md.

Examples:
  python3 examples/benchmark.py --dataset tumvi --root dataset/tum/dataset-room1_512_16 --mode stereo-imu
  python3 examples/benchmark.py --dataset euroc --root dataset/euroc/V1_01_easy --mode mono-imu
  python3 examples/benchmark.py --dataset kitti --root dataset/kitti/2011_09_26/2011_09_26_drive_0067_sync \
      --gt dataset/kitti/poses/03.txt --mode stereo
  python3 examples/benchmark.py --dataset tum_rgbd --root dataset/tum_rgbd/rgbd_dataset_freiburg1_desk --mode rgbd
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import pyorbslam3 as pos  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.abspath(os.path.join(HERE, ".."))
DEFAULT_VOCAB = os.environ.get(
    "PYORBSLAM3_VOCAB", os.path.join(PKG_ROOT, "vocabulary", "ORBvoc.txt")
)

# mode -> (sensor, uses_imu, uses_stereo, uses_depth)
MODES = {
    "mono": (pos.Sensor.MONOCULAR, False, False, False),
    "mono-imu": (pos.Sensor.IMU_MONOCULAR, True, False, False),
    "stereo": (pos.Sensor.STEREO, False, True, False),
    "stereo-imu": (pos.Sensor.IMU_STEREO, True, True, False),
    "rgbd": (pos.Sensor.RGBD, False, False, True),
}

# dataset -> {mode: default settings yaml (relative to configs/)}
DEFAULT_SETTINGS = {
    "tumvi": {
        "mono": "tumvi/mono.yaml",
        "mono-imu": "tumvi/mono_inertial.yaml",
        "stereo": "tumvi/stereo.yaml",
        "stereo-imu": "tumvi/stereo_inertial.yaml",
    },
    "euroc": {
        "mono": "euroc/mono.yaml",
        "mono-imu": "euroc/mono_inertial.yaml",
        "stereo": "euroc/stereo.yaml",
        "stereo-imu": "euroc/stereo_inertial.yaml",
    },
    "kitti": {
        "mono": "kitti/mono_2011_09_26.yaml",
        "stereo": "kitti/stereo_2011_09_26.yaml",
    },
    "tum_rgbd": {
        "mono": "tum_rgbd/mono_fr{fr}.yaml",
        "rgbd": "tum_rgbd/rgbd_fr{fr}.yaml",
    },
}


# ----------------------------------------------------------------------------
# Dataset loaders. Each returns a dict:
#   stamps (N,) seconds, files0 [N], files1 [N] | None, depth [N] | None,
#   imu (M,7) [t,gx,gy,gz,ax,ay,az] | None, gt (K,>=4) [t,px,py,pz,...] | None
# ----------------------------------------------------------------------------

def _mav0_cam(cam_dir):
    stamps, files = [], []
    with open(os.path.join(cam_dir, "data.csv")) as f:
        for row in csv.reader(f):
            if not row or row[0].startswith("#"):
                continue
            stamps.append(int(row[0]) * 1e-9)
            files.append(os.path.join(cam_dir, "data", row[1].strip()))
    return np.asarray(stamps), files


def _mav0_imu(imu_dir):
    data = np.loadtxt(os.path.join(imu_dir, "data.csv"), delimiter=",", comments="#")
    data[:, 0] *= 1e-9  # columns already [t, gx, gy, gz, ax, ay, az]
    return data


def load_mav0(root, gt_subdir):
    mav0 = os.path.join(root, "mav0")
    stamps, files0 = _mav0_cam(os.path.join(mav0, "cam0"))
    stamps1, files1 = _mav0_cam(os.path.join(mav0, "cam1"))
    if len(stamps1) != len(stamps):
        # keep only pairs with identical timestamps
        common = np.intersect1d(stamps, stamps1)
        m0 = {s: f for s, f in zip(stamps, files0)}
        m1 = {s: f for s, f in zip(stamps1, files1)}
        stamps = common
        files0 = [m0[s] for s in common]
        files1 = [m1[s] for s in common]
    imu = _mav0_imu(os.path.join(mav0, "imu0"))
    gt = None
    gt_file = os.path.join(mav0, gt_subdir, "data.csv")
    if os.path.isfile(gt_file):
        gt = np.loadtxt(gt_file, delimiter=",", comments="#")
        gt[:, 0] *= 1e-9
    return dict(stamps=stamps, files0=files0, files1=files1, depth=None, imu=imu, gt=gt)


def load_tumvi(root, _gt):
    return load_mav0(root, "mocap0")


def load_euroc(root, _gt):
    return load_mav0(root, "state_groundtruth_estimate0")


def _kitti_timestamps(path):
    out = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            # "2011-09-26 13:02:25.594360375" -> seconds
            d, t = line.split(" ")
            hh, mm, ss = t.split(":")
            day = dt.datetime.strptime(d, "%Y-%m-%d")
            out.append(
                (day - dt.datetime(1970, 1, 1)).total_seconds()
                + int(hh) * 3600 + int(mm) * 60 + float(ss)
            )
    return np.asarray(out)


def load_kitti(root, gt_file):
    stamps = _kitti_timestamps(os.path.join(root, "image_00", "timestamps.txt"))
    files0 = sorted(
        os.path.join(root, "image_00", "data", f)
        for f in os.listdir(os.path.join(root, "image_00", "data"))
    )
    files1 = sorted(
        os.path.join(root, "image_01", "data", f)
        for f in os.listdir(os.path.join(root, "image_01", "data"))
    )
    n = min(len(stamps), len(files0), len(files1))
    stamps, files0, files1 = stamps[:n], files0[:n], files1[:n]
    gt = None
    if gt_file and os.path.isfile(gt_file):
        poses = np.loadtxt(gt_file)  # (K,12) rows of 3x4 [R|t] in cam0 frame
        k = min(len(poses), n)
        gt = np.column_stack([stamps[:k], poses[:k, 3], poses[:k, 7], poses[:k, 11]])
    else:
        oxts_dir = os.path.join(root, "oxts")
        if os.path.isdir(oxts_dir):
            gt = _kitti_oxts_positions(oxts_dir, stamps)
    return dict(stamps=stamps, files0=files0, files1=files1, depth=None, imu=None, gt=gt)


def _kitti_oxts_positions(oxts_dir, stamps):
    """Fallback ground truth: GPS lat/lon/alt -> local metric ENU positions."""
    files = sorted(os.listdir(os.path.join(oxts_dir, "data")))
    rows = [np.loadtxt(os.path.join(oxts_dir, "data", f)) for f in files]
    lat, lon, alt = (np.asarray([r[i] for r in rows]) for i in range(3))
    er = 6378137.0
    scale = np.cos(lat[0] * np.pi / 180.0)
    x = scale * lon * np.pi * er / 180.0
    y = scale * er * np.log(np.tan((90.0 + lat) * np.pi / 360.0))
    x, y, z = x - x[0], y - y[0], alt - alt[0]
    k = min(len(x), len(stamps))
    return np.column_stack([stamps[:k], x[:k], y[:k], z[:k]])


def _tum_list(path):
    stamps, files = [], []
    with open(path) as f:
        for line in f:
            if not line.strip() or line.startswith("#"):
                continue
            t, name = line.split()[:2]
            stamps.append(float(t))
            files.append(name)
    return np.asarray(stamps), files


def load_tum_rgbd(root, _gt):
    rgb_t, rgb_f = _tum_list(os.path.join(root, "rgb.txt"))
    dep_t, dep_f = _tum_list(os.path.join(root, "depth.txt"))
    # associate each rgb frame with the nearest depth frame (<= 20 ms apart)
    idx = np.clip(np.searchsorted(dep_t, rgb_t), 1, len(dep_t) - 1)
    left_closer = np.abs(dep_t[idx - 1] - rgb_t) < np.abs(dep_t[idx] - rgb_t)
    idx[left_closer] -= 1
    ok = np.abs(dep_t[idx] - rgb_t) <= 0.02
    stamps = rgb_t[ok]
    files0 = [os.path.join(root, rgb_f[i]) for i in np.flatnonzero(ok)]
    depth = [os.path.join(root, dep_f[j]) for j in idx[ok]]
    gt = None
    gt_file = os.path.join(root, "groundtruth.txt")
    if os.path.isfile(gt_file):
        gt = np.loadtxt(gt_file, comments="#")  # t tx ty tz qx qy qz qw
    return dict(stamps=stamps, files0=files0, files1=None, depth=depth, imu=None, gt=gt)


LOADERS = {"tumvi": load_tumvi, "euroc": load_euroc, "kitti": load_kitti, "tum_rgbd": load_tum_rgbd}


# ----------------------------------------------------------------------------
# Evaluation
# ----------------------------------------------------------------------------

def _umeyama(src, dst, with_scale):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    xs, xd = src - mu_s, dst - mu_d
    cov = xd.T @ xs / len(xs)
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    scale = 1.0
    if with_scale:
        var_s = (xs ** 2).sum() / len(xs)
        scale = float(np.trace(np.diag(D) @ S) / var_s)
    t = mu_d - scale * R @ mu_s
    err = (scale * (R @ src.T)).T + t - dst
    return float(np.sqrt((err ** 2).sum(1).mean())), scale


def evaluate_ate(traj_file, gt, max_dt=0.02):
    """Associate an EuRoC-format trajectory (ns timestamps) with ground truth
    [t, px, py, pz, ...] and return dict(ate_sim3, scale, ate_se3, pairs)."""
    traj = np.loadtxt(traj_file)
    if traj.ndim != 2 or len(traj) < 10 or gt is None:
        return None
    t_est, p_est = traj[:, 0] * 1e-9, traj[:, 1:4]
    idx = np.clip(np.searchsorted(gt[:, 0], t_est), 1, len(gt) - 1)
    left_closer = np.abs(gt[idx - 1, 0] - t_est) < np.abs(gt[idx, 0] - t_est)
    idx[left_closer] -= 1
    ok = np.abs(gt[idx, 0] - t_est) < max_dt
    if ok.sum() < 10:
        return None
    src, dst = p_est[ok], gt[idx[ok], 1:4]
    ate_sim3, scale = _umeyama(src, dst, True)
    ate_se3, _ = _umeyama(src, dst, False)
    return dict(ate_sim3=ate_sim3, scale=scale, ate_se3=ate_se3, pairs=int(ok.sum()))


# ----------------------------------------------------------------------------
# Runner
# ----------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dataset", choices=LOADERS, required=True)
    ap.add_argument("--root", required=True, help="Sequence directory")
    ap.add_argument("--mode", choices=MODES, required=True)
    ap.add_argument("--gt", default=None, help="KITTI: odometry poses file (NN.txt)")
    ap.add_argument("--vocab", default=DEFAULT_VOCAB)
    ap.add_argument("--settings", default=None, help="Override settings yaml")
    ap.add_argument("--max-frames", type=int, default=0, help="0 = all frames")
    ap.add_argument("--viewer", action="store_true")
    ap.add_argument("--out-dir", default=os.path.join(PKG_ROOT, "results"))
    ap.add_argument("--tag", default=None, help="Sequence name used in results (default: basename of --root)")
    args = ap.parse_args()

    if args.mode not in DEFAULT_SETTINGS[args.dataset] and args.settings is None:
        sys.exit(f"mode {args.mode} is not available for dataset {args.dataset} "
                 f"(available: {', '.join(DEFAULT_SETTINGS[args.dataset])})")
    sensor, use_imu, use_stereo, use_depth = MODES[args.mode]
    seq = args.tag or os.path.basename(os.path.normpath(args.root))

    settings = args.settings
    if settings is None:
        rel = DEFAULT_SETTINGS[args.dataset][args.mode]
        if "{fr}" in rel:
            fr = "3" if "freiburg3" in seq else ("2" if "freiburg2" in seq else "1")
            rel = rel.format(fr=fr)
        settings = os.path.join(PKG_ROOT, "configs", rel)
    os.makedirs(args.out_dir, exist_ok=True)

    data = LOADERS[args.dataset](args.root, args.gt)
    stamps, files0, files1, depth, imu, gt = (
        data["stamps"], data["files0"], data["files1"], data["depth"], data["imu"], data["gt"]
    )
    if use_imu:
        if imu is None:
            sys.exit("dataset has no IMU data")
        first = int(np.searchsorted(stamps, imu[0, 0], side="right"))
        stamps, files0 = stamps[first:], files0[first:]
        if files1:
            files1 = files1[first:]
    n = len(stamps) if args.max_frames <= 0 else min(args.max_frames, len(stamps))
    print(f"[benchmark] {args.dataset}/{seq} mode={args.mode} frames={n} settings={settings}")

    slam = pos.Slam(args.vocab, settings, sensor, use_viewer=args.viewer)
    states, track_times = [], []
    t_prev = -np.inf
    imu_init_frame = None
    wall_start = time.time()
    for i in range(n):
        t = stamps[i]
        im0 = cv2.imread(files0[i], cv2.IMREAD_UNCHANGED if use_depth else cv2.IMREAD_GRAYSCALE)
        if im0 is None:
            print(f"[warn] unreadable image {files0[i]}, skipping")
            continue
        chunk = None
        if use_imu:
            chunk = imu[(imu[:, 0] > t_prev) & (imu[:, 0] <= t)]
        t0 = time.perf_counter()
        if use_stereo:
            im1 = cv2.imread(files1[i], cv2.IMREAD_GRAYSCALE)
            res = slam.track_stereo(im0, im1, t, imu=chunk)
        elif use_depth:
            dm = cv2.imread(depth[i], cv2.IMREAD_UNCHANGED).astype(np.float32)
            res = slam.track_rgbd(im0, dm, t)
        else:
            res = slam.track_monocular(im0, t, imu=chunk)
        track_times.append(time.perf_counter() - t0)
        t_prev = t
        states.append(int(res.state))
        if use_imu and imu_init_frame is None and slam.imu_initialized():
            imu_init_frame = i
            print(f"[benchmark] IMU initialized at frame {i} (t={t - stamps[0]:.1f}s)")
        if i % 250 == 0:
            print(f"[benchmark] frame {i}/{n} state={res.state} tracked_pts={res.num_tracked_points}")
    wall = time.time() - wall_start

    states = np.asarray(states)
    n_ok = int((states == int(pos.TrackingState.OK)).sum())
    n_lost = int(((states == int(pos.TrackingState.LOST)) | (states == int(pos.TrackingState.RECENTLY_LOST))).sum())
    traj = os.path.join(args.out_dir, f"traj_{args.dataset}_{seq}_{args.mode}.txt")
    kf_traj = os.path.join(args.out_dir, f"kf_traj_{args.dataset}_{seq}_{args.mode}.txt")
    slam.shutdown()
    slam.save_trajectory_euroc(traj)
    slam.save_keyframe_trajectory_euroc(kf_traj)

    ate = evaluate_ate(traj, gt) if os.path.isfile(traj) else None
    record = dict(
        dataset=args.dataset, sequence=seq, mode=args.mode,
        frames=int(len(states)), ok=n_ok, ok_pct=round(100.0 * n_ok / max(1, len(states)), 1),
        lost=n_lost,
        fps=round(len(states) / wall, 1),
        track_ms_median=round(1000.0 * float(np.median(track_times)), 1),
        imu_init_s=(round(float(stamps[imu_init_frame] - stamps[0]), 1) if imu_init_frame is not None else None),
        ate_sim3_cm=(round(100 * ate["ate_sim3"], 2) if ate else None),
        scale=(round(ate["scale"], 3) if ate else None),
        ate_se3_cm=(round(100 * ate["ate_se3"], 2) if ate else None),
        gt_pairs=(ate["pairs"] if ate else None),
        trajectory=os.path.relpath(traj, PKG_ROOT),
        settings=os.path.relpath(settings, PKG_ROOT),
        date=dt.date.today().isoformat(),
    )
    with open(os.path.join(args.out_dir, "benchmarks.jsonl"), "a") as f:
        f.write(json.dumps(record) + "\n")

    print(f"\n=== {args.dataset}/{seq} {args.mode} ===")
    print(f"frames processed : {record['frames']}  ({record['fps']} fps wall, "
          f"{record['track_ms_median']} ms median track call)")
    print(f"tracking OK      : {n_ok} ({record['ok_pct']}%)  lost/recently-lost: {n_lost}")
    if use_imu:
        print(f"IMU initialized  : {'yes, t=' + str(record['imu_init_s']) + ' s' if imu_init_frame is not None else 'NO'}")
    if ate:
        print(f"ATE RMSE sim(3)  : {record['ate_sim3_cm']:.2f} cm  scale={record['scale']:.3f}  "
              f"({ate['pairs']} poses)")
        print(f"ATE RMSE SE(3)   : {record['ate_se3_cm']:.2f} cm  (no scale correction)")
    else:
        print("ATE              : no ground truth / not enough associated poses")
    print(f"trajectory saved : {traj}")


if __name__ == "__main__":
    main()
