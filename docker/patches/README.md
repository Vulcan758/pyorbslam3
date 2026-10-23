# ORB-SLAM3 patches applied in the Docker build

Applied on top of upstream `UZ-SLAMLab/ORB_SLAM3` at commit
`4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`, in order:

1. `0001-cmake-static-pic-modern-targets.patch` — CMake modernization: static
   `-fPIC` libraries, DBoW2/g2o/Sophus built via `add_subdirectory`, an
   exported `ORB_SLAM3::ORB_SLAM3` target, and small header fixes for current
   toolchains. Taken from the MIT-licensed PRBonn `ORB-SLAM-Python` project
   (`3rdparty/ORB_SLAM3/cmakefix.patch`), with thanks.
2. `0002-localmapping-keyframeculling-guards.patch` — defensive null / `isBad()` /
   keypoint-index bounds guards in `LocalMapping::KeyFrameCulling()`, and null
   checks on IMU preintegration pointers before `MergePrevious()`. Works around
   an upstream race that intermittently segfaulted the inertial pipeline.
