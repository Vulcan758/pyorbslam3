"""High-level wrapper around the pyorbslam3._core bindings."""

from __future__ import annotations

import os
from typing import NamedTuple, Optional, Sequence, Union

import numpy as np

from . import _core
from ._core import Sensor, TrackingState, TrackResult


class ImuMeasurement(NamedTuple):
    """One IMU sample. Timestamp in seconds, gyro in rad/s, accel in m/s^2."""

    timestamp: float
    gx: float
    gy: float
    gz: float
    ax: float
    ay: float
    az: float


ImuInput = Union[np.ndarray, Sequence[ImuMeasurement], Sequence[Sequence[float]], None]

_INERTIAL = {Sensor.IMU_MONOCULAR, Sensor.IMU_STEREO, Sensor.IMU_RGBD}


def _as_imu_array(imu: ImuInput) -> np.ndarray:
    """Normalize IMU input to an (N, 7) float64 array [t, gx, gy, gz, ax, ay, az]."""
    if imu is None:
        return np.empty((0, 7), dtype=np.float64)
    arr = np.asarray(imu, dtype=np.float64)
    if arr.size == 0:
        return np.empty((0, 7), dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] != 7:
        raise ValueError(
            "imu must be an (N, 7) array-like with rows "
            "[t, gx, gy, gz, ax, ay, az] (seconds, rad/s, m/s^2); "
            f"got shape {arr.shape}"
        )
    return np.ascontiguousarray(arr)


class Slam:
    """ORB-SLAM3 system.

    Parameters
    ----------
    vocab_file:
        Path to ORBvoc.txt (loading takes tens of seconds).
    settings_file:
        ORB-SLAM3 OpenCV-YAML settings file (camera calibration, ORB params,
        and IMU noise/extrinsics for the IMU_* sensors).
    sensor:
        One of :class:`Sensor`. IMU_* modes require IMU measurements to be
        passed to every track call.
    use_viewer:
        Launch the Pangolin visualization (requires a display).
    """

    def __init__(
        self,
        vocab_file: Union[str, os.PathLike],
        settings_file: Union[str, os.PathLike],
        sensor: Sensor,
        use_viewer: bool = False,
    ):
        vocab_file = os.fspath(vocab_file)
        settings_file = os.fspath(settings_file)
        if not os.path.isfile(vocab_file):
            raise FileNotFoundError(f"Vocabulary file not found: {vocab_file}")
        if not os.path.isfile(settings_file):
            raise FileNotFoundError(f"Settings file not found: {settings_file}")
        self._slam = _core.Slam(vocab_file, settings_file, sensor, use_viewer)
        self._sensor = sensor
        self._scale = float(self._slam.image_scale())

    # -- properties --------------------------------------------------------

    @property
    def sensor(self) -> Sensor:
        return self._sensor

    @property
    def inertial(self) -> bool:
        return self._sensor in _INERTIAL

    # -- tracking ----------------------------------------------------------

    def _maybe_rescale(self, image: np.ndarray) -> np.ndarray:
        if self._scale == 1.0:
            return image
        import cv2

        h, w = image.shape[:2]
        return cv2.resize(image, (int(w * self._scale), int(h * self._scale)))

    def track_monocular(
        self, image: np.ndarray, timestamp: float, imu: ImuInput = None
    ) -> TrackResult:
        """Track one monocular frame.

        ``imu``: all IMU samples with timestamps in (previous frame time,
        this frame time], as (N, 7) rows [t, gx, gy, gz, ax, ay, az].
        Required for Sensor.IMU_MONOCULAR, must be omitted otherwise.
        """
        return self._slam.track_monocular(
            self._maybe_rescale(image), float(timestamp), _as_imu_array(imu)
        )

    def track_stereo(
        self,
        left: np.ndarray,
        right: np.ndarray,
        timestamp: float,
        imu: ImuInput = None,
    ) -> TrackResult:
        """Track one synchronized stereo pair (see track_monocular for imu)."""
        return self._slam.track_stereo(
            self._maybe_rescale(left),
            self._maybe_rescale(right),
            float(timestamp),
            _as_imu_array(imu),
        )

    def track_rgbd(
        self,
        image: np.ndarray,
        depth: np.ndarray,
        timestamp: float,
        imu: ImuInput = None,
    ) -> TrackResult:
        """Track one RGB-D frame; depth is float32 (or castable) HxW."""
        return self._slam.track_rgbd(
            self._maybe_rescale(image),
            self._maybe_rescale(depth),
            float(timestamp),
            _as_imu_array(imu),
        )

    # -- state -------------------------------------------------------------

    def tracking_state(self) -> TrackingState:
        return self._slam.tracking_state()

    def is_lost(self) -> bool:
        return self._slam.is_lost()

    def map_changed(self) -> bool:
        """True if a big map change (loop closure / global BA) happened since last call."""
        return self._slam.map_changed()

    def time_from_imu_init(self) -> float:
        """Seconds since IMU initialization; <= 0 while not yet initialized."""
        return self._slam.time_from_imu_init()

    def imu_initialized(self) -> bool:
        return self.inertial and self._slam.time_from_imu_init() > 0.0

    def tracked_map_points(self) -> np.ndarray:
        """(N, 3) float32 world positions of map points seen in the last frame."""
        return self._slam.tracked_map_points()

    def tracked_keypoints(self) -> np.ndarray:
        """(N, 2) float32 undistorted pixel coordinates of last frame's keypoints."""
        return self._slam.tracked_keypoints()

    # -- control -----------------------------------------------------------

    def reset(self) -> None:
        self._slam.reset()

    def reset_active_map(self) -> None:
        self._slam.reset_active_map()

    def activate_localization_mode(self) -> None:
        """Stop mapping; track against the existing map only."""
        self._slam.activate_localization_mode()

    def deactivate_localization_mode(self) -> None:
        self._slam.deactivate_localization_mode()

    # -- saving ------------------------------------------------------------

    def save_trajectory_tum(self, filename: str) -> None:
        self._slam.save_trajectory_tum(filename)

    def save_keyframe_trajectory_tum(self, filename: str) -> None:
        self._slam.save_keyframe_trajectory_tum(filename)

    def save_trajectory_euroc(self, filename: str) -> None:
        self._slam.save_trajectory_euroc(filename)

    def save_keyframe_trajectory_euroc(self, filename: str) -> None:
        self._slam.save_keyframe_trajectory_euroc(filename)

    def save_trajectory_kitti(self, filename: str) -> None:
        self._slam.save_trajectory_kitti(filename)

    # -- lifecycle ---------------------------------------------------------

    def shutdown(self) -> None:
        """Stop all ORB-SLAM3 threads. Call before saving trajectories at exit."""
        self._slam.shutdown()

    def is_shutdown(self) -> bool:
        return self._slam.is_shutdown()

    def __enter__(self) -> "Slam":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.shutdown()
