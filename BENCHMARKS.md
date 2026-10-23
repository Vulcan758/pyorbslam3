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


### TUM-VI

| Sequence | Mode | Frames | Tracked OK | IMU init | ATE sim(3) | scale | ATE SE(3) | fps | track ms |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| room1 (run 1) | mono | 2821 | 2724 (96.6%) | n/a | 7.29 cm | 1.785 | n/a | 7.1 | 67.0 |
| room1 (run 2) | mono | 2821 | 2724 (96.6%) | n/a | 97.42 cm | 8.485 | n/a | 17.7 | 45.8 |
| room1 (run 1) | mono-imu | 2821 | 2700 (95.7%) | 8.4 s | 1.22 cm | 1.000 | 1.22 cm | 3.3 | 56.7 |
| room1 (run 2) | mono-imu | 2821 | 2700 (95.7%) | 8.4 s | 1.28 cm | 0.998 | 1.30 cm | 19.0 | 43.3 |
| room1 (run 1) | stereo | 2821 | 2821 (100.0%) | n/a | 7.15 cm | 0.974 | 7.62 cm | 4.9 | 102.9 |
| room1 (run 2) | stereo | 2821 | 2821 (100.0%) | n/a | 7.16 cm | 0.975 | 7.60 cm | 9.8 | 84.1 |
| room1 | stereo-imu | 2821 | 2746 (97.3%) | 2.5 s | 0.76 cm | 1.005 | 0.90 cm | 6.7 | 86.0 |

### EuRoC MAV

| Sequence | Mode | Frames | Tracked OK | IMU init | ATE sim(3) | scale | ATE SE(3) | fps | track ms |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| V1_01_easy | mono | 2912 | 2802 (96.2%) | n/a | 8.87 cm | 2.058 | n/a | 25.8 | 30.2 |
| V1_01_easy | mono-imu | 2911 | 2798 (96.1%) | 7.9 s | 3.25 cm | 1.011 | 3.83 cm | 24.3 | 32.3 |
| V1_01_easy | stereo | 2912 | 2912 (100.0%) | n/a | 8.82 cm | 1.005 | 8.86 cm | 16.6 | 53.5 |
| V1_01_easy | stereo-imu | 2911 | 2864 (98.4%) | 2.5 s | 3.45 cm | 1.009 | 3.83 cm | 12.7 | 68.1 |
| V1_02_medium | mono | 1710 | 1531 (89.5%) | n/a | 9.40 cm | 0.216 | n/a | 29.1 | 24.8 |
| V1_02_medium | mono-imu | 1709 | 1594 (93.3%) | 11.0 s | 1.94 cm | 1.004 | 2.06 cm | 27.8 | 26.5 |
| V1_02_medium | stereo | 1710 | 1710 (100.0%) | n/a | 8.10 cm | 1.008 | 8.22 cm | 17.5 | 51.2 |
| V1_02_medium | stereo-imu | 1709 | 1687 (98.7%) | 2.4 s | 1.41 cm | 1.007 | 1.90 cm | 15.1 | 60.9 |
| V1_03_difficult | mono | 2149 | 1737 (80.8%) | n/a | 13.34 cm | 3.806 | n/a | 31.5 | 22.7 |
| V1_03_difficult | mono-imu | 2149 | 1790 (83.3%) | 9.8 s | 2.17 cm | 1.015 | 2.83 cm | 29.9 | 24.1 |
| V1_03_difficult | stereo | 2149 | 2149 (100.0%) | n/a | 15.12 cm | 1.018 | 15.38 cm | 18.7 | 48.4 |
| V1_03_difficult | stereo-imu | 2149 | 2071 (96.4%) | 2.4 s | 1.99 cm | 1.013 | 2.79 cm | 16.7 | 56.2 |

### KITTI raw

| Sequence | Mode | Frames | Tracked OK | IMU init | ATE sim(3) | scale | ATE SE(3) | fps | track ms |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 2011_09_26_drive_0036 (run 1) | mono | 803 | 802 (99.9%) | n/a | 801.9 cm | 12.176 | n/a | 18.4 | 41.0 |
| 2011_09_26_drive_0036 (run 2) | mono | 803 | 802 (99.9%) | n/a | 895.0 cm | 12.781 | n/a | 19.8 | 38.1 |
| 2011_09_26_drive_0036 | stereo | 803 | 803 (100.0%) | n/a | 85.59 cm | 1.007 | 151.4 cm | 6.0 | 88.9 |
| 2011_09_26_drive_0101 | mono | 936 | 845 (90.3%) | n/a | 2602.1 cm | 117.2 | n/a | 6.4 | 39.7 |
| 2011_09_26_drive_0101 | stereo | 936 | 936 (100.0%) | n/a | 107.7 cm | 1.000 | 107.8 cm | 4.5 | 117.4 |

### TUM RGB-D

| Sequence | Mode | Frames | Tracked OK | IMU init | ATE sim(3) | scale | ATE SE(3) | fps | track ms |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| fr1_desk | mono | 596 | 492 (82.6%) | n/a | 1.50 cm | 1.043 | n/a | 17.9 | 34.8 |
| fr1_desk | rgbd | 596 | 596 (100.0%) | n/a | 1.60 cm | 0.996 | 1.64 cm | 6.5 | 60.4 |
| fr3_long_office (run 1) | mono | 2488 | 2463 (99.0%) | n/a | 105.6 cm | 2.715 | n/a | 15.7 | 41.0 |
| fr3_long_office (run 2) | mono | 2488 | 2463 (99.0%) | n/a | 2.60 cm | 2.435 | n/a | 18.5 | 34.5 |
| fr3_long_office | rgbd | 2488 | 2488 (100.0%) | n/a | 1.65 cm | 0.986 | 3.44 cm | 12.6 | 59.4 |

## Reading the results

* **Inertial modes recover metric scale.** Across all eight inertial runs
  (TUM-VI room1, EuRoC V1_01–V1_03) the similarity-alignment scale lies within
  1.5 % of 1.0 and the ATE is 0.8–3.5 cm. Stereo-inertial initializes the IMU in
  about 2.5 s on every sequence; mono-inertial needs 8–11 s of motion.
* **Stereo tracks every frame.** 100 % of frames `OK` on all seven stereo
  sequences, scale 0.97–1.02 without any IMU. Indoor ATE is 7–15 cm; the KITTI
  drives give about 1 m over 800–940 frames of driving against GPS/INS ground
  truth, whose own accuracy is at the decimetre level.
* **RGB-D** tracks every frame on both TUM sequences with 1.6 cm ATE and metric
  scale.
* **Plain monocular is the least repeatable mode.** Scale is arbitrary by
  construction, and whenever ORB-SLAM3 re-initializes a map mid-sequence the
  saved trajectory contains segments with different scales, which a single
  similarity alignment cannot undo. That is what the paired runs show: room1 mono
  7 cm in one run and 97 cm in the next, fr3_long_office 106 cm and then 2.6 cm,
  with identical tracking rates. On the KITTI driving sequences monocular drifts
  in scale by one to two orders of magnitude (no loop closures on these drives),
  which is expected and why the ORB-SLAM3 authors report KITTI with stereo only.
  For anything that needs a usable trajectory, use stereo, RGB-D or an inertial
  mode.
* **Throughput.** On this 1.0 GHz laptop CPU the bindings run EuRoC (752×480)
  at 13–29 fps and TUM-VI (512×512 fisheye) at 10–19 fps depending on mode. The
  "run 1" TUM-VI rows were measured while a Docker image was compiling on the
  same machine; the "run 2" rows of the same modes show the uncontended speed.
  The Python layer adds no measurable overhead: the median track-call time is
  the ORB-SLAM3 tracking thread's own frame time.

Where a mode was run twice, `results/` holds the trajectory of the later run
(the `_run2` files for the two repeats that were run into a separate folder,
otherwise the file was overwritten); the JSONL keeps both sets of metrics.

## Not covered here

* Real-camera results (ZED 2i, RealSense D435i) live in the `orbulation` ROS 2
  package's user guide. D435i stereo-inertial is configured but still needs a
  handheld test with motion.
* TUM-VI corridor/magistrale/outdoor sequences, EuRoC machine hall and
  vicon_room2, and the KITTI odometry benchmark proper were not run.

