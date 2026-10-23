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
