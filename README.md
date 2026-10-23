# pyorbslam3

Python bindings for [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) built
for production use: every sensor mode (monocular, stereo, RGB-D, each with or
without an IMU), explicit tracking state on every frame, the GIL released while
ORB-SLAM3 works, and errors raised as Python exceptions. A self-contained Docker
image is provided, and the bindings are benchmarked on TUM-VI, EuRoC, KITTI and
TUM RGB-D ([BENCHMARKS.md](BENCHMARKS.md)).

They are the SLAM core of the [`orbulation`](https://github.com/Vulcan758/orbulation)
ROS 2 package, which runs them live on a ZED 2i and an Intel RealSense D435i.

## What the bindings offer

* **All six ORB-SLAM3 sensor modes** — `MONOCULAR`, `STEREO`, `RGBD` and their
  inertial variants `IMU_MONOCULAR`, `IMU_STEREO`, `IMU_RGBD`.
* **A `TrackResult` per frame, not just a matrix.** ORB-SLAM3 returns an identity
  pose whenever it is not initialized or has lost tracking. Each track call here
  returns the tracking state (`NOT_INITIALIZED`, `OK`, `RECENTLY_LOST`, `LOST`,
  …), a `pose_valid` flag, the camera pose both ways (`pose_wc`, `pose_cw`) and
  the number of tracked map points, so a caller can decide what to publish.
* **Inertial modes that are easy to feed correctly.** IMU samples go in as an
  `(N, 7)` array in the natural `[t, gx, gy, gz, ax, ay, az]` order — the same
  column order as a EuRoC/TUM-VI `imu0/data.csv` row or a ROS `sensor_msgs/Imu`.
  Timestamps are validated, and `imu_initialized()` /
  `time_from_imu_init()` tell you when metric scale and gravity are available.
* **Thread-friendly.** The GIL is released inside the constructor, every track
  call and `shutdown()`, so image and IMU callbacks keep running in other
  Python threads (this is what makes a ROS 2 node with a multi-threaded executor
  work).
* **Exceptions instead of process exits.** Bad paths, a track call that does not
  match the sensor mode, malformed IMU input or empty images raise with a
  message naming the problem; the Python process stays alive.
* **Map introspection and output.** Tracked map points and keypoints of the
  last frame, `map_changed()` for loop-closure events, localization-only mode,
  reset, and trajectory export in TUM, EuRoC and KITTI formats.
* **Known upstream crashes are handled.** `System::GetTimeFromIMUInit()` reads an
  uninitialized pointer until the first keyframe exists; the binding guards the
  call. The Docker build also carries a small defensive patch for a race in
  `LocalMapping::KeyFrameCulling()` that could segfault long inertial runs.

## Quick start with Docker

```bash
docker build -t pyorbslam3 .            # builds Pangolin + ORB-SLAM3 + bindings (30–60 min)
docker run --rm -it -v /path/to/datasets:/data pyorbslam3 bash
# inside the container:
python3 examples/benchmark.py --dataset tumvi --root /data/dataset-room1_512_16 --mode stereo-imu
```

The image is Ubuntu 22.04 with Python 3.10 and contains the bindings, the ORB
vocabulary (`$PYORBSLAM3_VOCAB`), the dataset configs and the example scripts;
it weighs about 1 GB. `JOBS` (default 3) caps build parallelism because a few
ORB-SLAM3 translation units need about 2 GB of RAM each to compile: use
`--build-arg JOBS=2` on an 8 GB machine, more on a large one.

To use your own code against the image, mount it and import `pyorbslam3`:

```bash
docker run --rm -it -v $PWD:/work -w /work pyorbslam3 python3 my_script.py
```

Publishing the image (replace `<user>`):

```bash
docker tag pyorbslam3:latest <user>/pyorbslam3:0.1.0
docker tag pyorbslam3:latest <user>/pyorbslam3:latest
docker push <user>/pyorbslam3:0.1.0 && docker push <user>/pyorbslam3:latest
```

## Building natively

Requirements: Ubuntu 22.04 (or similar), CMake ≥ 3.16, a C++17 compiler, OpenCV
≥ 4.4, Eigen 3, Boost.Serialization, OpenSSL, Pangolin 0.9 (static, built with
`CMAKE_POSITION_INDEPENDENT_CODE=ON`), `pybind11` (`pip install pybind11`).

1. Build ORB-SLAM3 as static `-fPIC` libraries. The `Dockerfile` shows the
   exact recipe: upstream commit `4452a3c4` plus the two patches in
   `docker/patches/`, built with `cmake -S ORB_SLAM3 -B ORB_SLAM3/build`.
2. Build and install the bindings:

```bash
./build.sh -DORBSLAM3_ROOT=/path/to/ORB_SLAM3   # drops _core*.so into pyorbslam3/
pip install -e .
curl -L https://github.com/UZ-SLAMLab/ORB_SLAM3/raw/master/Vocabulary/ORBvoc.txt.tar.gz \
  | tar -xz -C vocabulary/                      # 139 MB, not committed
```

`ORBSLAM3_ROOT` defaults to `../ORB_SLAM3` if that tree has been built. The
CMake file also still accepts `-DORBSLAM_DEPS=<_deps dir>` to link against an
ORB-SLAM-Python style build tree.

## Usage

```python
import pyorbslam3 as pos

slam = pos.Slam(
    vocab_file="vocabulary/ORBvoc.txt",
    settings_file="configs/euroc/stereo_inertial.yaml",
    sensor=pos.Sensor.IMU_STEREO,      # MONOCULAR / STEREO / RGBD / IMU_MONOCULAR / IMU_STEREO / IMU_RGBD
    use_viewer=False,
)

result = slam.track_stereo(left, right, t, imu=imu_chunk)   # or track_monocular / track_rgbd
result.state               # TrackingState.OK, NOT_INITIALIZED, RECENTLY_LOST, LOST, ...
result.ok                  # state == OK
result.pose_valid          # pose usable (OK or RECENTLY_LOST)
result.pose_wc             # 4x4 numpy, camera in world (None if not pose_valid)
result.pose_cw             # 4x4 numpy, world in camera (ORB-SLAM3's Tcw)
result.num_tracked_points  # map points tracked in this frame

slam.imu_initialized()     # inertial modes: metric scale + gravity available?
slam.tracked_map_points()  # (N, 3) world points seen in the last frame
slam.tracked_keypoints()   # (N, 2) undistorted pixel coordinates
slam.map_changed()         # loop closure / global BA since the last call?

slam.shutdown()            # stop threads, then:
slam.save_trajectory_euroc("traj.txt")   # also *_tum / *_kitti, and keyframe variants
```

`Slam` is also a context manager (`with pos.Slam(...) as slam:`).

### Conventions

* **Timestamps are seconds** (float64). EuRoC/TUM-VI csv files are in
  nanoseconds — multiply by `1e-9`.
* **IMU chunk per frame:** `(N, 7)` rows `[t, gx, gy, gz, ax, ay, az]` in
  seconds, rad/s and m/s², containing every sample with
  `t_prev_frame < t <= t_frame`. Lists of `ImuMeasurement` tuples are accepted too.
* **Images:** `uint8` arrays, `HxW` grayscale or `HxWx3` colour. Depth for RGB-D
  is `HxW` and scaled by `RGBD.DepthMapFactor` from the settings file (so the
  raw 16-bit TUM depth images cast to float32 work as-is).
* **Frames:** `pose_wc` is what you publish as odometry. In inertial modes the
  world frame becomes gravity-aligned (z up) and metric once the IMU initializes.

### What to expect from inertial modes

* IMU initialization needs motion. Mono-inertial typically takes 8–15 s of
  handheld motion; stereo-inertial a few seconds. A static camera never
  initializes ("not enough acceleration") and periodically resets the map —
  that is ORB-SLAM3 behaviour, not a fault.
* Until then the state is `NOT_INITIALIZED` and `pose_valid` is false.
* An empty IMU chunk in an inertial mode emits a warning; IMU data in a
  non-inertial mode raises.

## Benchmarks

See [BENCHMARKS.md](BENCHMARKS.md) for results on TUM-VI, EuRoC, KITTI raw and
TUM RGB-D, with frame-tracking rates and ATE against ground truth. Reproduce
any row with `examples/benchmark.py`:

```bash
python3 examples/benchmark.py --dataset euroc --root /data/V1_01_easy --mode mono-imu
python3 examples/benchmark.py --dataset kitti --root /data/2011_09_26/2011_09_26_drive_0036_sync --mode stereo
python3 examples/benchmark.py --dataset tum_rgbd --root /data/rgbd_dataset_freiburg1_desk --mode rgbd
python3 examples/make_report.py -o BENCHMARKS.md --header docs/benchmarks_header.md --footer docs/benchmarks_notes.md
```

Every run appends a JSON record to `results/benchmarks.jsonl` and saves the
trajectory, so results stay reproducible. ORB-SLAM3 is multi-threaded and not
deterministic; expect a few percent of variation between runs.

## Real-camera status

Through the `orbulation` ROS 2 package:

| Camera | Mode | Status |
|---|---|---|
| ZED 2i (SDK-less driver) | stereo | live, 100 % frames OK |
| ZED 2i | stereo-inertial | validated on a 90 s handheld recording replayed through the live node (1706/1707 frames OK, IMU init 2.7 s) |
| RealSense D435i | stereo (infrared pair) | live, stationary on a desk: state OK, ~300 points, 24 fps |
| RealSense D435i | stereo-inertial | **pending** — configured, needs a handheld test with motion |

## Files

```
src/orbslam3_pybind.cpp    C++ bindings (single file)
pyorbslam3/                Python package (+ built _core*.so)
configs/{tumvi,euroc,kitti,tum_rgbd}/   ORB-SLAM3 settings per dataset and mode
examples/benchmark.py      multi-dataset runner + ATE evaluation
examples/make_report.py    renders results/benchmarks.jsonl to Markdown
examples/run_tumvi.py      original TUM-VI runner
docker/patches/            ORB-SLAM3 patches applied in the Docker build
Dockerfile                 self-contained image
```

## Acknowledgements

ORB-SLAM3 is by Campos, Elvira, Rodríguez, Montiel and Tardós (GPLv3). The CMake
modernization patch comes from the PRBonn ORB-SLAM-Python project (MIT).
