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
3. `0003-framedrawer-fisheye-stereo-bounds.patch` — `FrameDrawer::Update()`
   indexed the left-only keypoint vector with left + right indices for
   fisheye (`KannalaBrandt8`) stereo, reading past its end on every frame even
   without the viewer. Intermittent segfault in stereo / stereo-inertial on
   TUM-VI; pinhole stereo is unaffected.
4. `0004-localmapping-fisheye-uright-bounds.patch` — `LocalMapping::CreateNewMapPoints()`
   read `mvuRight[idx]` for right-camera match indices, past the end of the
   left-sized array of fisheye stereo frames. Intermittent LocalMapping segfault
   in stereo-inertial on TUM-VI. The value is unused when there is a second
   camera, so out-of-range reads now return -1 with no behaviour change.
5. `0005-localmapping-keyframeculling-fisheye-right-index.patch`. Applies after 0002.
   - **Bug 1:** `LocalMapping::KeyFrameCulling()` indexed `mvKeysRight` with the
     combined keypoint index `i` instead of `i - NLeft`.
   - **Bug 2:** it read `mvDepth[i]`, which for fisheye stereo frames only holds
     the left keypoints.
   - **Symptom:** an intermittent LocalMapping segfault in stereo-inertial on
     TUM-VI room1, about 1 in 9 runs.
   - **Fix:** right-camera points now count as having no depth (skipped, like
     upstream's other non-stereo points), and the right octave index is corrected.
