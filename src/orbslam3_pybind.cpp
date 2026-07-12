// pyorbslam3 — pybind11 bindings for ORB-SLAM3
//
// Design goals (vs. the PRBonn ORB-SLAM-Python bindings previously used here):
//  * Working inertial AND non-inertial modes with a clearly documented IMU
//    convention: rows of [t, gx, gy, gz, ax, ay, az]  (rad/s, m/s^2, seconds),
//    i.e. the exact column order of EuRoC / TUM-VI imu0/data.csv.
//  * The GIL is released during every long-running call (constructor/vocab
//    load, Track*, Shutdown) so Python threads — e.g. ROS2 executors — keep
//    spinning while the C++ side works.
//  * Track calls return a TrackResult carrying the tracking state; ORB-SLAM3
//    returns an identity pose when it is lost, which is indistinguishable from
//    a real pose without the state.
//  * Every ORB-SLAM3 code path that would call exit(-1) (bad settings path,
//    bad vocabulary path, track method not matching the configured sensor) is
//    guarded here and raises a Python exception instead of killing the
//    process.

#include <System.h>
#include <MapPoint.h>
#include <ImuTypes.h>

#include <opencv2/core.hpp>
#include <opencv2/core/persistence.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <Eigen/Core>
#include <sophus/se3.hpp>

#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace py = pybind11;
using namespace py::literals;

namespace {

using ImageArray = py::array_t<uint8_t, py::array::c_style | py::array::forcecast>;
using DepthArray = py::array_t<float, py::array::c_style | py::array::forcecast>;
using ImuArray = py::array_t<double, py::array::c_style | py::array::forcecast>;

enum class TrackingState : int {
    SYSTEM_NOT_READY = -1,
    NO_IMAGES_YET = 0,
    NOT_INITIALIZED = 1,
    OK = 2,
    RECENTLY_LOST = 3,
    LOST = 4,
    OK_KLT = 5,
};

struct TrackResult {
    Eigen::Matrix4d Tcw = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
    int state = 0;
    bool pose_valid = false;
    int num_tracked_points = 0;

    py::object pose_cw() const {
        if (!pose_valid) return py::none();
        return py::cast(Tcw);
    }
    py::object pose_wc() const {
        if (!pose_valid) return py::none();
        return py::cast(Twc);
    }
};

// Wraps (no copy) a HxW / HxWx{1,3,4} uint8 numpy array as a cv::Mat.
// The caller must keep the py::array alive for the duration of use.
cv::Mat wrap_image(const ImageArray &a, const char *what) {
    if (a.ndim() == 2) {
        return cv::Mat(static_cast<int>(a.shape(0)), static_cast<int>(a.shape(1)),
                       CV_8UC1, const_cast<uint8_t *>(a.data()));
    }
    if (a.ndim() == 3) {
        const int ch = static_cast<int>(a.shape(2));
        if (ch == 1 || ch == 3 || ch == 4) {
            return cv::Mat(static_cast<int>(a.shape(0)), static_cast<int>(a.shape(1)),
                           CV_MAKETYPE(CV_8U, ch), const_cast<uint8_t *>(a.data()));
        }
    }
    std::ostringstream oss;
    oss << what << " must be a HxW (grayscale) or HxWx{1,3,4} (color) uint8 array, got ndim="
        << a.ndim();
    throw std::invalid_argument(oss.str());
}

cv::Mat wrap_depth(const DepthArray &a) {
    if (a.ndim() != 2) {
        throw std::invalid_argument("depth must be a HxW float32 array");
    }
    return cv::Mat(static_cast<int>(a.shape(0)), static_cast<int>(a.shape(1)), CV_32FC1,
                   const_cast<float *>(a.data()));
}

// IMU rows: [t, gx, gy, gz, ax, ay, az] — t in seconds, gyro rad/s, accel m/s^2.
std::vector<ORB_SLAM3::IMU::Point> to_imu_points(const ImuArray &a, double frame_timestamp) {
    std::vector<ORB_SLAM3::IMU::Point> out;
    if (a.size() == 0) return out;
    if (a.ndim() != 2 || a.shape(1) != 7) {
        throw std::invalid_argument(
            "imu must be an (N, 7) array with rows [t, gx, gy, gz, ax, ay, az] "
            "(seconds, rad/s, m/s^2)");
    }
    auto r = a.unchecked<2>();
    out.reserve(static_cast<size_t>(r.shape(0)));
    double prev_t = -std::numeric_limits<double>::infinity();
    for (py::ssize_t i = 0; i < r.shape(0); ++i) {
        const double t = r(i, 0);
        if (t < prev_t) {
            std::ostringstream oss;
            oss << "imu timestamps must be non-decreasing (row " << i << ": " << t
                << " < previous " << prev_t << "). Column order is [t, gx, gy, gz, ax, ay, az].";
            throw std::invalid_argument(oss.str());
        }
        prev_t = t;
        // IMU::Point ctor is (acc, gyro, t)
        out.emplace_back(static_cast<float>(r(i, 4)), static_cast<float>(r(i, 5)),
                         static_cast<float>(r(i, 6)), static_cast<float>(r(i, 1)),
                         static_cast<float>(r(i, 2)), static_cast<float>(r(i, 3)), t);
    }
    if (!out.empty() && out.back().t > frame_timestamp + 1e-3) {
        std::ostringstream oss;
        oss << "last imu timestamp (" << out.back().t << ") is after the frame timestamp ("
            << frame_timestamp << "); pass only measurements up to the current frame. "
            << "If the gap is huge, check that both are in *seconds*.";
        throw std::invalid_argument(oss.str());
    }
    return out;
}

class Slam {
public:
    Slam(const std::string &vocab_file, const std::string &settings_file,
         ORB_SLAM3::System::eSensor sensor, bool use_viewer) : sensor_(sensor) {
        // Pre-flight checks: ORB-SLAM3 calls exit(-1) on these, which would
        // kill the whole Python process.
        {
            std::ifstream v(vocab_file);
            if (!v.good()) {
                throw std::invalid_argument("Vocabulary file not found: " + vocab_file);
            }
        }
        try {
            cv::FileStorage fs(settings_file, cv::FileStorage::READ);
            if (!fs.isOpened()) throw cv::Exception();
        } catch (const cv::Exception &) {
            throw std::invalid_argument(
                "Settings file missing or not valid OpenCV YAML (needs '%YAML:1.0' header): " +
                settings_file);
        }
        py::gil_scoped_release release;  // vocabulary load takes tens of seconds
        system_ = std::make_unique<ORB_SLAM3::System>(vocab_file, settings_file, sensor,
                                                      use_viewer);
    }

    ~Slam() {
        try {
            shutdown();
        } catch (...) {
        }
    }

    bool inertial() const {
        return sensor_ == ORB_SLAM3::System::IMU_MONOCULAR ||
               sensor_ == ORB_SLAM3::System::IMU_STEREO ||
               sensor_ == ORB_SLAM3::System::IMU_RGBD;
    }

    ORB_SLAM3::System::eSensor sensor() const { return sensor_; }

    TrackResult track_monocular(const ImageArray &image, double timestamp, const ImuArray &imu) {
        require_sensor("track_monocular", {ORB_SLAM3::System::MONOCULAR,
                                           ORB_SLAM3::System::IMU_MONOCULAR});
        cv::Mat im = wrap_image(image, "image");
        auto imu_points = prepare_imu(imu, timestamp);
        Sophus::SE3f Tcw;
        {
            py::gil_scoped_release release;
            Tcw = system_->TrackMonocular(im, timestamp, imu_points);
        }
        return make_result(Tcw);
    }

    TrackResult track_stereo(const ImageArray &left, const ImageArray &right, double timestamp,
                             const ImuArray &imu) {
        require_sensor("track_stereo",
                       {ORB_SLAM3::System::STEREO, ORB_SLAM3::System::IMU_STEREO});
        cv::Mat imL = wrap_image(left, "left image");
        cv::Mat imR = wrap_image(right, "right image");
        auto imu_points = prepare_imu(imu, timestamp);
        Sophus::SE3f Tcw;
        {
            py::gil_scoped_release release;
            Tcw = system_->TrackStereo(imL, imR, timestamp, imu_points);
        }
        return make_result(Tcw);
    }

    TrackResult track_rgbd(const ImageArray &image, const DepthArray &depth, double timestamp,
                           const ImuArray &imu) {
        require_sensor("track_rgbd", {ORB_SLAM3::System::RGBD, ORB_SLAM3::System::IMU_RGBD});
        cv::Mat im = wrap_image(image, "image");
        cv::Mat dm = wrap_depth(depth);
        auto imu_points = prepare_imu(imu, timestamp);
        Sophus::SE3f Tcw;
        {
            py::gil_scoped_release release;
            Tcw = system_->TrackRGBD(im, dm, timestamp, imu_points);
        }
        return make_result(Tcw);
    }

    int tracking_state() const { return system_->GetTrackingState(); }
    bool is_lost() { return system_->isLost(); }
    bool map_changed() { return system_->MapChanged(); }

    // Upstream bug: LocalMapping::mpCurrentKeyFrame is never initialized in
    // its constructor, and System::GetTimeFromIMUInit() dereferences it via
    // GetCurrKFTime(). Calling it before the mapper has processed a keyframe
    // segfaults. Only forward the call once tracking has been OK on two
    // separate track calls — by then (>= one full frame period later) the
    // mapper has consumed the first keyframe and the pointer is valid.
    double time_from_imu_init() {
        if (!imu_time_safe_) return 0.0;
        return system_->GetTimeFromIMUInit();
    }
    float image_scale() { return system_->GetImageScale(); }

    void reset() {
        seen_ok_ = imu_time_safe_ = false;
        system_->Reset();
    }
    void reset_active_map() {
        seen_ok_ = imu_time_safe_ = false;
        system_->ResetActiveMap();
    }
    void activate_localization_mode() { system_->ActivateLocalizationMode(); }
    void deactivate_localization_mode() { system_->DeactivateLocalizationMode(); }

    // World positions (Nx3 float32) of the map points seen in the last frame.
    py::array_t<float> tracked_map_points() {
        std::vector<Eigen::Vector3f> pts;
        {
            auto mps = system_->GetTrackedMapPoints();
            pts.reserve(mps.size());
            for (auto *mp : mps) {
                if (mp && !mp->isBad()) pts.push_back(mp->GetWorldPos());
            }
        }
        py::array_t<float> out({static_cast<py::ssize_t>(pts.size()), py::ssize_t(3)});
        auto w = out.mutable_unchecked<2>();
        for (size_t i = 0; i < pts.size(); ++i) {
            w(i, 0) = pts[i].x();
            w(i, 1) = pts[i].y();
            w(i, 2) = pts[i].z();
        }
        return out;
    }

    // Undistorted pixel coordinates (Nx2 float32) of last frame's keypoints.
    py::array_t<float> tracked_keypoints() {
        auto kps = system_->GetTrackedKeyPointsUn();
        py::array_t<float> out({static_cast<py::ssize_t>(kps.size()), py::ssize_t(2)});
        auto w = out.mutable_unchecked<2>();
        for (size_t i = 0; i < kps.size(); ++i) {
            w(i, 0) = kps[i].pt.x;
            w(i, 1) = kps[i].pt.y;
        }
        return out;
    }

    void save_trajectory_tum(const std::string &f) { system_->SaveTrajectoryTUM(f); }
    void save_keyframe_trajectory_tum(const std::string &f) {
        system_->SaveKeyFrameTrajectoryTUM(f);
    }
    void save_trajectory_euroc(const std::string &f) { system_->SaveTrajectoryEuRoC(f); }
    void save_keyframe_trajectory_euroc(const std::string &f) {
        system_->SaveKeyFrameTrajectoryEuRoC(f);
    }
    void save_trajectory_kitti(const std::string &f) { system_->SaveTrajectoryKITTI(f); }

    bool is_shutdown() { return !system_ || system_->isShutDown(); }

    void shutdown() {
        if (system_ && !system_->isShutDown()) {
            py::gil_scoped_release release;
            system_->Shutdown();
        }
    }

private:
    void require_sensor(const char *method,
                        std::initializer_list<ORB_SLAM3::System::eSensor> allowed) const {
        for (auto s : allowed) {
            if (sensor_ == s) return;
        }
        std::ostringstream oss;
        oss << method << "() does not match the sensor this system was created with; "
            << "use the track method for your Sensor mode.";
        throw std::runtime_error(oss.str());
    }

    std::vector<ORB_SLAM3::IMU::Point> prepare_imu(const ImuArray &imu, double timestamp) const {
        auto pts = to_imu_points(imu, timestamp);
        if (inertial() && pts.empty()) {
            // Not fatal (e.g. very first frame), but almost always a bug in
            // the caller's data pipeline — surface it.
            py::module_::import("warnings")
                .attr("warn")(
                    "inertial sensor mode but no IMU measurements were passed for this frame");
        }
        if (!inertial() && !pts.empty()) {
            throw std::invalid_argument(
                "IMU measurements passed but the system is in a non-inertial mode; create the "
                "system with an IMU_* sensor to use them");
        }
        return pts;
    }

    TrackResult make_result(const Sophus::SE3f &Tcw) {
        TrackResult r;
        r.state = system_->GetTrackingState();
        if (r.state == static_cast<int>(TrackingState::OK)) {
            if (seen_ok_) imu_time_safe_ = true;
            seen_ok_ = true;
        }
        r.pose_valid = (r.state == static_cast<int>(TrackingState::OK) ||
                        r.state == static_cast<int>(TrackingState::RECENTLY_LOST));
        r.Tcw = Tcw.matrix().cast<double>();
        r.Twc = Tcw.inverse().matrix().cast<double>();
        auto mps = system_->GetTrackedMapPoints();
        int n = 0;
        for (auto *mp : mps) {
            if (mp && !mp->isBad()) ++n;
        }
        r.num_tracked_points = n;
        return r;
    }

    ORB_SLAM3::System::eSensor sensor_;
    std::unique_ptr<ORB_SLAM3::System> system_;
    bool seen_ok_ = false;
    bool imu_time_safe_ = false;
};

}  // namespace

PYBIND11_MODULE(_core, m) {
    m.doc() = "Low-level ORB-SLAM3 bindings. Prefer the pyorbslam3.Slam wrapper.";

    py::enum_<ORB_SLAM3::System::eSensor>(m, "Sensor")
        .value("MONOCULAR", ORB_SLAM3::System::MONOCULAR)
        .value("STEREO", ORB_SLAM3::System::STEREO)
        .value("RGBD", ORB_SLAM3::System::RGBD)
        .value("IMU_MONOCULAR", ORB_SLAM3::System::IMU_MONOCULAR)
        .value("IMU_STEREO", ORB_SLAM3::System::IMU_STEREO)
        .value("IMU_RGBD", ORB_SLAM3::System::IMU_RGBD);

    py::enum_<TrackingState>(m, "TrackingState")
        .value("SYSTEM_NOT_READY", TrackingState::SYSTEM_NOT_READY)
        .value("NO_IMAGES_YET", TrackingState::NO_IMAGES_YET)
        .value("NOT_INITIALIZED", TrackingState::NOT_INITIALIZED)
        .value("OK", TrackingState::OK)
        .value("RECENTLY_LOST", TrackingState::RECENTLY_LOST)
        .value("LOST", TrackingState::LOST)
        .value("OK_KLT", TrackingState::OK_KLT);

    py::class_<TrackResult>(m, "TrackResult")
        .def_property_readonly("state",
                               [](const TrackResult &r) {
                                   return static_cast<TrackingState>(r.state);
                               })
        .def_property_readonly("ok",
                               [](const TrackResult &r) {
                                   return r.state == static_cast<int>(TrackingState::OK);
                               })
        .def_property_readonly("pose_valid", [](const TrackResult &r) { return r.pose_valid; })
        .def_property_readonly("pose_cw", &TrackResult::pose_cw,
                               "4x4 camera-from-world transform (None unless OK/RECENTLY_LOST)")
        .def_property_readonly("pose_wc", &TrackResult::pose_wc,
                               "4x4 world-from-camera transform (None unless OK/RECENTLY_LOST)")
        .def_readonly("num_tracked_points", &TrackResult::num_tracked_points)
        .def("__repr__", [](const TrackResult &r) {
            std::ostringstream oss;
            oss << "TrackResult(state=" << r.state << ", pose_valid=" << (r.pose_valid ? "True" : "False")
                << ", num_tracked_points=" << r.num_tracked_points << ")";
            return oss.str();
        });

    py::class_<Slam>(m, "Slam")
        .def(py::init<const std::string &, const std::string &, ORB_SLAM3::System::eSensor,
                      bool>(),
             "vocab_file"_a, "settings_file"_a, "sensor"_a, "use_viewer"_a = false)
        .def("track_monocular", &Slam::track_monocular, "image"_a, "timestamp"_a,
             "imu"_a = ImuArray())
        .def("track_stereo", &Slam::track_stereo, "left"_a, "right"_a, "timestamp"_a,
             "imu"_a = ImuArray())
        .def("track_rgbd", &Slam::track_rgbd, "image"_a, "depth"_a, "timestamp"_a,
             "imu"_a = ImuArray())
        .def_property_readonly("sensor", &Slam::sensor)
        .def_property_readonly("inertial", &Slam::inertial)
        .def("tracking_state",
             [](const Slam &s) { return static_cast<TrackingState>(s.tracking_state()); })
        .def("is_lost", &Slam::is_lost)
        .def("map_changed", &Slam::map_changed)
        .def("time_from_imu_init", &Slam::time_from_imu_init,
             "Seconds since the IMU was initialized (<= 0 before initialization)")
        .def("image_scale", &Slam::image_scale)
        .def("reset", &Slam::reset)
        .def("reset_active_map", &Slam::reset_active_map)
        .def("activate_localization_mode", &Slam::activate_localization_mode)
        .def("deactivate_localization_mode", &Slam::deactivate_localization_mode)
        .def("tracked_map_points", &Slam::tracked_map_points)
        .def("tracked_keypoints", &Slam::tracked_keypoints)
        .def("save_trajectory_tum", &Slam::save_trajectory_tum, "filename"_a)
        .def("save_keyframe_trajectory_tum", &Slam::save_keyframe_trajectory_tum, "filename"_a)
        .def("save_trajectory_euroc", &Slam::save_trajectory_euroc, "filename"_a)
        .def("save_keyframe_trajectory_euroc", &Slam::save_keyframe_trajectory_euroc,
             "filename"_a)
        .def("save_trajectory_kitti", &Slam::save_trajectory_kitti, "filename"_a)
        .def("is_shutdown", &Slam::is_shutdown)
        .def("shutdown", &Slam::shutdown);
}
