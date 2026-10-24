"""pyorbslam3 — Python bindings for ORB-SLAM3 with inertial and non-inertial modes.

Quick start::

    import pyorbslam3 as pos

    slam = pos.Slam(
        vocab_file=pos.vocabulary_path(),   # downloads ORBvoc.txt once
        settings_file="configs/tumvi/mono_inertial.yaml",
        sensor=pos.Sensor.IMU_MONOCULAR,
    )
    for image, t, imu_chunk in my_data_source():
        # imu_chunk: (N, 7) array, rows [t, gx, gy, gz, ax, ay, az]
        result = slam.track_monocular(image, t, imu=imu_chunk)
        if result.pose_valid:
            do_something(result.pose_wc)
    slam.shutdown()
"""

from ._core import Sensor, TrackingState, TrackResult
from .slam import Slam, ImuMeasurement
from .vocab import vocabulary_path

__all__ = ["Slam", "Sensor", "TrackingState", "TrackResult", "ImuMeasurement", "vocabulary_path"]
