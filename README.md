# pyorbslam3

Python bindings for [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) with
**working inertial and non-inertial modes**, built to be wrapped by ROS2 nodes.

They link against the ORB-SLAM3 static libraries already compiled by the old
`ORB-SLAM-Python` project in this workspace (`ORB-SLAM-Python/build/.../_deps`),
so no ORB-SLAM3 rebuild is needed.

## Why not the old ORB-SLAM-Python bindings?

* **No tracking state** — ORB-SLAM3 returns an *identity pose* when it is lost
  or not yet initialized; without the state you cannot tell a real pose from
  garbage. That is what made IMU runs look broken: mono-inertial spends the
  first ~10 s in `NOT_INITIALIZED` and returns identity the whole time.
* **Undocumented IMU layout** (`[ax..az, gx..gz, t]`, silently wrong if you
  pass the natural csv/ROS order).
* **The GIL was held during tracking** — every `Track*` call froze all Python
  threads, which breaks ROS2 executors.
* **`exit(-1)` on bad input** — a wrong settings path or calling
  `TrackMonocular` on a stereo system killed the whole Python process.

`pyorbslam3` fixes all four and adds map-point / keypoint access, IMU
initialization status, and trajectory saving in TUM/EuRoC/KITTI formats.

It also works around an **upstream ORB-SLAM3 bug**: `LocalMapping::
mpCurrentKeyFrame` is never initialized in its constructor, so calling
`System::GetTimeFromIMUInit()` before the first keyframe is processed
dereferences a garbage pointer and segfaults the process. The binding only
forwards that call once tracking has been `OK` on two separate frames;
before that `time_from_imu_init()` safely returns 0.

## Build

```bash
cd pyorbslam3
./build.sh          # → pyorbslam3/_core.cpython-310-x86_64-linux-gnu.so
```

### Vocabulary

`vocabulary/ORBvoc.txt` is the standard ORB-SLAM3 vocabulary and isn't
committed to this repo (139 MB). Fetch it with:

```bash
curl -L https://github.com/UZ-SLAMLab/ORB_SLAM3/raw/master/Vocabulary/ORBvoc.txt.tar.gz \
  | tar -xz -C vocabulary/
```

Requires: the existing `ORB-SLAM-Python` build tree (path configurable with
`-DORBSLAM_DEPS=...`), system OpenCV/Eigen (same versions the tree was built
with), `pybind11`.

## Usage

```python
import pyorbslam3 as pos

slam = pos.Slam(
    vocab_file="vocabulary/ORBvoc.txt",
    settings_file="configs/tumvi/mono_inertial.yaml",
    sensor=pos.Sensor.IMU_MONOCULAR,   # or MONOCULAR / STEREO / IMU_STEREO / RGBD / IMU_RGBD
    use_viewer=False,
)

result = slam.track_monocular(gray_image, t, imu=imu_chunk)
# result.state               -> TrackingState (OK, NOT_INITIALIZED, RECENTLY_LOST, ...)
# result.ok                  -> state == OK
# result.pose_valid          -> pose usable (OK or RECENTLY_LOST)
# result.pose_wc / .pose_cw  -> 4x4 numpy (None if not pose_valid)
# result.num_tracked_points  -> map points tracked in this frame

slam.imu_initialized()       # inertial modes: has the IMU been initialized yet?
slam.tracked_map_points()    # (N,3) world positions seen in last frame
slam.shutdown()              # stop threads; then save_trajectory_euroc(...) etc.
```

`Slam` is also a context manager (`with pos.Slam(...) as slam:`).

### Conventions (important!)

* **Timestamps are in seconds** (float64). TUM-VI/EuRoC csv files are in
  nanoseconds — multiply by `1e-9`.
* **IMU chunk per frame**: an `(N, 7)` array with rows
  `[t, gx, gy, gz, ax, ay, az]` (seconds, rad/s, m/s²) containing **all IMU
  samples with `t_prev_frame < t <= t_frame`**. This is exactly the column
  order of a EuRoC/TUM-VI `imu0/data.csv` row and of a ROS `sensor_msgs/Imu`
  read as (stamp, angular_velocity, linear_acceleration).
* Images: `uint8` numpy arrays, `HxW` grayscale or `HxWx3` color; depth is
  `HxW` float32.
* Poses: `pose_cw` maps world → camera (ORB-SLAM3's native `Tcw`);
  `pose_wc` is its inverse (camera position/orientation in world frame — what
  you want for odometry).
* In inertial modes the world frame is **gravity-aligned after IMU
  initialization** (z up) and scale is metric.

### Inertial-mode behavior to expect

* Mono-inertial needs ~10–15 s of motion before `imu_initialized()` turns
  true; until then the state stays `NOT_INITIALIZED` and poses are invalid.
  This is normal ORB-SLAM3 behavior, not a binding bug.
* Passing an empty IMU chunk in an inertial mode emits a Python warning;
  passing IMU data in a non-inertial mode raises.
* IMU timestamps must be non-decreasing and not (much) newer than the frame
  timestamp — violations raise with a message naming the expected layout.

## Testing on the TUM-VI room1 dataset

```bash
python3 examples/run_tumvi.py --mode mono         # non-inertial
python3 examples/run_tumvi.py --mode mono-imu     # mono-inertial
python3 examples/run_tumvi.py --mode stereo-imu   # stereo-inertial
python3 examples/run_tumvi.py --mode stereo       # stereo
```

The runner prints the fraction of frames tracked `OK`, when the IMU
initialized, saves EuRoC-format trajectories to `results/`, and computes an
ATE RMSE against the mocap ground truth (sim(3)-aligned; for inertial runs the
reported `scale` should be ≈ 1.0, which proves metric scale was recovered from
the IMU).

Add `--viewer` for the Pangolin GUI, `--max-frames N` for a quick run.

## Files

```
src/orbslam3_pybind.cpp   C++ bindings (single file)
pyorbslam3/               Python package (+ built _core*.so)
configs/tumvi/            ORB-SLAM3 settings for TUM-VI 512x512
vocabulary/ORBvoc.txt     ORB vocabulary (extracted)
examples/run_tumvi.py     dataset runner + ATE evaluation
```
