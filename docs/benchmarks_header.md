# pyorbslam3 benchmarks

Results of running the bindings on public datasets with `examples/benchmark.py`.
Every row is one full sequence run; the raw records are in
`results/benchmarks.jsonl` and the trajectories in `results/`. Regenerate this
file with `python3 examples/make_report.py -o BENCHMARKS.md --header docs/benchmarks_header.md --footer docs/benchmarks_notes.md`.

## Setup

* Machine: Intel Core i5-1035G1 (4 cores / 8 threads, 1.0 GHz base), 15 GB RAM,
  Ubuntu 22.04, Python 3.10, OpenCV 4.5.4. No GPU is used by ORB-SLAM3.
* Bindings linked against ORB-SLAM3 upstream commit `4452a3c4` with the two
  patches in `docker/patches/`. Viewer off.
* Settings files: the stock ORB-SLAM3 examples for each dataset
  (`configs/<dataset>/`), with two corrections: the TUM-VI stereo file's
  `Camera.overlapping*` keys renamed to `Camera1.*` (the upstream spelling is
  ignored by the parser), and the KITTI stereo file rewritten as two identical
  `PinHole` cameras instead of `Camera.type: "Rectified"`, which crashes in the
  settings printer upstream.
* Frames are fed sequentially as fast as tracking allows (no real-time pacing),
  so `fps` is the throughput on this CPU, not a dataset rate. `track ms` is the
  median wall time of one track call.

## Metrics

* **Tracked OK** — frames for which ORB-SLAM3 reported `TrackingState.OK`. In
  inertial modes the frames before IMU initialization count as not OK, which is
  why mono-inertial rates sit below the non-inertial ones.
* **IMU init** — seconds of sequence time until `imu_initialized()` turned true.
* **ATE sim(3)** — RMSE of position error after a similarity (Umeyama) alignment
  of the estimated trajectory to ground truth, over the poses that associate to
  a ground-truth sample within 20 ms. **scale** is the similarity scale found by
  that alignment: 1.0 means the estimate is already metric. For monocular runs
  the scale is arbitrary by construction; for stereo, RGB-D and inertial runs it
  should be close to 1.0.
* **ATE SE(3)** — the same error with scale fixed at 1 (rigid alignment only).
  This is the number that matters for a metric system; it is meaningless for
  plain monocular runs.

Ground truth: TUM-VI mocap (`mocap0`), EuRoC `state_groundtruth_estimate0`,
TUM RGB-D `groundtruth.txt`, KITTI raw OXTS GPS/INS converted to a local metric
frame (the odometry-benchmark drives with published poses are not in the raw
download set, so GPS/INS is used; its accuracy is at the decimetre level, which
sets a floor on the KITTI numbers).

ORB-SLAM3 is multi-threaded and not deterministic; repeated runs of the same
sequence vary by a few percent in tracked frames and ATE.
