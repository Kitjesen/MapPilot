#include "slam.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <utility>

namespace nb = nanobind;
using namespace nb::literals;

namespace lingtu::slam {
namespace {

constexpr const char* kBindingSchema = "lingtu.slam.native.v2";

bool isNone(PyObject* value) {
  return value == nullptr || value == Py_None;
}

nb::object dictGet(const nb::dict& dict, const char* key) {
  PyObject* value = PyDict_GetItemString(dict.ptr(), key);
  if (value == nullptr) {
    return nb::none();
  }
  return nb::borrow<nb::object>(value);
}

double objectNumber(PyObject* value, double fallback = 0.0) {
  if (isNone(value)) {
    return fallback;
  }
  try {
    return nb::cast<double>(nb::borrow<nb::object>(value));
  } catch (const std::exception&) {
    PyErr_Clear();
    return fallback;
  }
}

std::string objectString(PyObject* value, std::string fallback = "") {
  if (isNone(value)) {
    return fallback;
  }
  try {
    return nb::cast<std::string>(nb::borrow<nb::object>(value));
  } catch (const std::exception&) {
    PyErr_Clear();
    return fallback;
  }
}

bool objectBool(PyObject* value, bool fallback = false) {
  if (isNone(value)) {
    return fallback;
  }
  const int truth = PyObject_IsTrue(value);
  if (truth < 0) {
    PyErr_Clear();
    return fallback;
  }
  return truth == 1;
}

double dictNumber(const nb::dict& dict, const char* key, double fallback = 0.0) {
  return objectNumber(dictGet(dict, key).ptr(), fallback);
}

std::string dictString(
    const nb::dict& dict,
    const char* key,
    std::string fallback = "") {
  return objectString(dictGet(dict, key).ptr(), std::move(fallback));
}

bool dictBool(const nb::dict& dict, const char* key, bool fallback = false) {
  return objectBool(dictGet(dict, key).ptr(), fallback);
}

Py_ssize_t objectLength(PyObject* value) {
  if (isNone(value)) {
    return -1;
  }
  const Py_ssize_t length = PyObject_Length(value);
  if (length < 0) {
    PyErr_Clear();
    return -1;
  }
  return length;
}

nb::object sequenceItem(PyObject* sequence, Py_ssize_t index) {
  if (isNone(sequence)) {
    return nb::none();
  }
  PyObject* item = PySequence_GetItem(sequence, index);
  if (item == nullptr) {
    PyErr_Clear();
    return nb::none();
  }
  return nb::steal<nb::object>(item);
}

double sequenceNumber(PyObject* sequence, Py_ssize_t index, double fallback = 0.0) {
  nb::object item = sequenceItem(sequence, index);
  return objectNumber(item.ptr(), fallback);
}

std::uint8_t sequenceByte(PyObject* sequence, Py_ssize_t index, std::uint8_t fallback = 0) {
  const double value = sequenceNumber(sequence, index, static_cast<double>(fallback));
  if (value < 0.0) {
    return 0;
  }
  if (value > 255.0) {
    return 255;
  }
  return static_cast<std::uint8_t>(value);
}

PyObject* dictItem(PyObject* dict, const char* key) {
  if (dict == nullptr || !PyDict_Check(dict)) {
    return nullptr;
  }
  return PyDict_GetItemString(dict, key);
}

double dictItemNumber(PyObject* dict, const char* key, double fallback = 0.0) {
  return objectNumber(dictItem(dict, key), fallback);
}

void readVector3(PyObject* value, double& x, double& y, double& z) {
  if (isNone(value)) {
    return;
  }
  if (PyDict_Check(value)) {
    x = dictItemNumber(value, "x", x);
    y = dictItemNumber(value, "y", y);
    z = dictItemNumber(value, "z", z);
    return;
  }
  x = sequenceNumber(value, 0, x);
  y = sequenceNumber(value, 1, y);
  z = sequenceNumber(value, 2, z);
}

Pose3d poseFromDict(const nb::dict& raw) {
  PyObject* pose_obj = raw.ptr();
  if (PyObject* nested = dictItem(pose_obj, "pose"); PyDict_Check(nested)) {
    pose_obj = nested;
  }

  Pose3d pose;
  if (PyObject* position = dictItem(pose_obj, "position")) {
    pose.x = dictItemNumber(position, "x", pose.x);
    pose.y = dictItemNumber(position, "y", pose.y);
    pose.z = dictItemNumber(position, "z", pose.z);
  } else {
    pose.x = dictItemNumber(pose_obj, "x", pose.x);
    pose.y = dictItemNumber(pose_obj, "y", pose.y);
    pose.z = dictItemNumber(pose_obj, "z", pose.z);
  }

  if (PyObject* orientation = dictItem(pose_obj, "orientation")) {
    pose.qx = dictItemNumber(orientation, "x", pose.qx);
    pose.qy = dictItemNumber(orientation, "y", pose.qy);
    pose.qz = dictItemNumber(orientation, "z", pose.qz);
    pose.qw = dictItemNumber(orientation, "w", pose.qw);
  } else {
    pose.qx = dictItemNumber(pose_obj, "qx", pose.qx);
    pose.qy = dictItemNumber(pose_obj, "qy", pose.qy);
    pose.qz = dictItemNumber(pose_obj, "qz", pose.qz);
    pose.qw = dictItemNumber(pose_obj, "qw", pose.qw);
  }
  return pose;
}

ImuSample imuFromDict(const nb::dict& raw) {
  ImuSample sample;
  sample.stamp_s = dictNumber(raw, "stamp_s", dictNumber(raw, "ts", 0.0));

  if (nb::object orientation = dictGet(raw, "orientation"); !orientation.is_none()) {
    sample.qx = dictItemNumber(orientation.ptr(), "x", sample.qx);
    sample.qy = dictItemNumber(orientation.ptr(), "y", sample.qy);
    sample.qz = dictItemNumber(orientation.ptr(), "z", sample.qz);
    sample.qw = dictItemNumber(orientation.ptr(), "w", sample.qw);
  }

  double gx = sample.gx;
  double gy = sample.gy;
  double gz = sample.gz;
  readVector3(dictGet(raw, "angular_velocity").ptr(), gx, gy, gz);
  sample.gx = gx;
  sample.gy = gy;
  sample.gz = gz;

  double ax = sample.ax;
  double ay = sample.ay;
  double az = sample.az;
  readVector3(dictGet(raw, "linear_acceleration").ptr(), ax, ay, az);
  sample.ax = ax;
  sample.ay = ay;
  sample.az = az;
  return sample;
}

void appendColumnarPoints(const nb::dict& raw, LidarFrame& frame) {
  nb::object x = dictGet(raw, "x");
  nb::object y = dictGet(raw, "y");
  nb::object z = dictGet(raw, "z");
  if (x.is_none() || y.is_none() || z.is_none()) {
    return;
  }

  Py_ssize_t count = objectLength(x.ptr());
  const auto y_count = objectLength(y.ptr());
  const auto z_count = objectLength(z.ptr());
  if (count < 0 || y_count < 0 || z_count < 0) {
    return;
  }
  count = std::min(count, std::min(y_count, z_count));
  const auto requested = static_cast<Py_ssize_t>(dictNumber(raw, "num_points", count));
  if (requested >= 0) {
    count = std::min(count, requested);
  }

  nb::object intensity = dictGet(raw, "intensity");
  nb::object offsets = dictGet(raw, "offset_time_ns");
  nb::object line = dictGet(raw, "line");
  nb::object tag = dictGet(raw, "tag");

  frame.points.reserve(static_cast<std::size_t>(count));
  for (Py_ssize_t i = 0; i < count; ++i) {
    PointXYZIT point;
    point.x = static_cast<float>(sequenceNumber(x.ptr(), i, 0.0));
    point.y = static_cast<float>(sequenceNumber(y.ptr(), i, 0.0));
    point.z = static_cast<float>(sequenceNumber(z.ptr(), i, 0.0));
    point.intensity = static_cast<float>(sequenceNumber(intensity.ptr(), i, 0.0));
    point.offset_time_ns =
        static_cast<std::int64_t>(sequenceNumber(offsets.ptr(), i, 0.0));
    point.line = sequenceByte(line.ptr(), i, 0);
    point.tag = sequenceByte(tag.ptr(), i, 0);
    frame.points.push_back(point);
  }
}

void appendRowPoints(const nb::dict& raw, LidarFrame& frame) {
  nb::object points = dictGet(raw, "points");
  const Py_ssize_t count = objectLength(points.ptr());
  if (count < 0) {
    return;
  }

  const auto requested = static_cast<Py_ssize_t>(dictNumber(raw, "num_points", count));
  const Py_ssize_t limit = requested >= 0 ? std::min(count, requested) : count;
  frame.points.reserve(static_cast<std::size_t>(limit));
  for (Py_ssize_t i = 0; i < limit; ++i) {
    nb::object row = sequenceItem(points.ptr(), i);
    if (row.is_none()) {
      continue;
    }
    PointXYZIT point;
    if (PyDict_Check(row.ptr())) {
      point.x = static_cast<float>(dictItemNumber(row.ptr(), "x", 0.0));
      point.y = static_cast<float>(dictItemNumber(row.ptr(), "y", 0.0));
      point.z = static_cast<float>(dictItemNumber(row.ptr(), "z", 0.0));
      point.intensity = static_cast<float>(dictItemNumber(row.ptr(), "intensity", 0.0));
      point.offset_time_ns =
          static_cast<std::int64_t>(dictItemNumber(row.ptr(), "offset_time_ns", 0.0));
      point.line = static_cast<std::uint8_t>(dictItemNumber(row.ptr(), "line", 0.0));
      point.tag = static_cast<std::uint8_t>(dictItemNumber(row.ptr(), "tag", 0.0));
    } else {
      point.x = static_cast<float>(sequenceNumber(row.ptr(), 0, 0.0));
      point.y = static_cast<float>(sequenceNumber(row.ptr(), 1, 0.0));
      point.z = static_cast<float>(sequenceNumber(row.ptr(), 2, 0.0));
      point.intensity = static_cast<float>(sequenceNumber(row.ptr(), 3, 0.0));
    }
    frame.points.push_back(point);
  }
}

LidarFrame lidarFromDict(const nb::dict& raw) {
  LidarFrame frame;
  frame.stamp_s = dictNumber(raw, "stamp_s", dictNumber(raw, "ts", 0.0));
  frame.frame_id = dictString(raw, "frame_id", "lidar");
  appendColumnarPoints(raw, frame);
  if (frame.points.empty()) {
    appendRowPoints(raw, frame);
  }
  return frame;
}

GnssSample gnssFromDict(const nb::dict& raw) {
  GnssSample sample;
  sample.stamp_s = dictNumber(raw, "stamp_s", dictNumber(raw, "ts", 0.0));
  sample.fix_type = dictString(raw, "fix_type", sample.fix_type);
  readVector3(dictGet(raw, "position_enu").ptr(), sample.east, sample.north, sample.up);
  readVector3(dictGet(raw, "velocity_enu").ptr(), sample.ve, sample.vn, sample.vu);
  readVector3(dictGet(raw, "cov_diag").ptr(), sample.cov_e, sample.cov_n, sample.cov_u);
  return sample;
}

OdomSample odomFromDict(const nb::dict& raw) {
  OdomSample sample;
  sample.stamp_s = dictNumber(raw, "stamp_s", dictNumber(raw, "ts", 0.0));
  sample.odom_body = poseFromDict(raw);
  if (PyObject* velocity = dictItem(raw.ptr(), "linear_velocity")) {
    readVector3(velocity, sample.vx, sample.vy, sample.vz);
    sample.has_velocity = true;
  } else if (PyObject* velocity_odom = dictItem(raw.ptr(), "velocity")) {
    readVector3(velocity_odom, sample.vx, sample.vy, sample.vz);
    sample.has_velocity = true;
  }
  return sample;
}

std::filesystem::path mapPcdPath(const std::string& path) {
  std::filesystem::path p(path);
  if (p.extension() == ".pcd") {
    return p;
  }
  return p / "map.pcd";
}

nb::dict statusDict(const Status& status) {
  nb::dict result;
  result["ok"_s] = status.ok;
  result["message"_s] = status.message;
  return result;
}

nb::dict statusDictWithMapArtifacts(const Status& status, const std::string& path) {
  nb::dict result = statusDict(status);
  const std::filesystem::path pcd = mapPcdPath(path);
  result["map_pcd"_s] = pcd.string();
  result["poses_txt"_s] = (pcd.parent_path() / "poses.txt").string();
  result["patches_dir"_s] = (pcd.parent_path() / "patches").string();
  return result;
}

nb::dict poseDict(const Pose3d& pose) {
  nb::dict position;
  position["x"_s] = pose.x;
  position["y"_s] = pose.y;
  position["z"_s] = pose.z;

  nb::dict orientation;
  orientation["x"_s] = pose.qx;
  orientation["y"_s] = pose.qy;
  orientation["z"_s] = pose.qz;
  orientation["w"_s] = pose.qw;

  nb::dict out;
  out["position"_s] = position;
  out["orientation"_s] = orientation;
  return out;
}

nb::dict odomDict(
    const Pose3d& pose,
    double stamp_s,
    const std::string& frame_id,
    const std::string& child_frame_id) {
  nb::dict out;
  out["pose"_s] = poseDict(pose);
  out["stamp_s"_s] = stamp_s;
  out["frame_id"_s] = frame_id;
  out["child_frame_id"_s] = child_frame_id;
  return out;
}

nb::dict cloudDict(const Cloud& cloud) {
  nb::list points;
  for (const auto& point : cloud.points) {
    nb::list row;
    row.append(point.x);
    row.append(point.y);
    row.append(point.z);
    row.append(point.intensity);
    points.append(row);
  }

  nb::dict out;
  out["stamp_s"_s] = cloud.stamp_s;
  out["frame_id"_s] = cloud.frame_id;
  out["num_points"_s] = static_cast<int>(cloud.points.size());
  out["points"_s] = points;
  return out;
}

nb::dict transformDict(const Transform3d& tf, double stamp_s) {
  nb::dict translation;
  translation["x"_s] = tf.pose.x;
  translation["y"_s] = tf.pose.y;
  translation["z"_s] = tf.pose.z;

  nb::dict rotation;
  rotation["x"_s] = tf.pose.qx;
  rotation["y"_s] = tf.pose.qy;
  rotation["z"_s] = tf.pose.qz;
  rotation["w"_s] = tf.pose.qw;

  nb::dict out;
  out["frame_id"_s] = tf.frame_id;
  out["child_frame_id"_s] = tf.child_frame_id;
  out["translation"_s] = translation;
  out["rotation"_s] = rotation;
  out["stamp_s"_s] = stamp_s;
  return out;
}

nb::dict gnssHealthDict(const GnssFusionHealth& health) {
  nb::dict out;
  out["enabled"_s] = health.enabled;
  out["alignment_locked"_s] = health.alignment_locked;
  out["last_fix_type"_s] = health.last_fix_type;
  out["last_gnss_age_s"_s] = health.last_gnss_age_s;
  out["last_residual_m"_s] = health.last_residual_m;
  out["relock_count"_s] = health.relock_count;
  return out;
}

nb::dict fastLioLidarUpdateDict(
    const FastLioLidarUpdateDiagnostics& diagnostics) {
  nb::dict candidate;
  candidate["translation_m"_s] = diagnostics.candidate_translation_m;
  candidate["rotation_rad"_s] = diagnostics.candidate_rotation_rad;
  candidate["velocity_mps"_s] = diagnostics.candidate_velocity_mps;
  candidate["velocity_delta_mps"_s] =
      diagnostics.candidate_velocity_delta_mps;

  nb::dict thresholds;
  thresholds["max_translation_m"_s] = diagnostics.max_update_translation_m;
  thresholds["max_rotation_rad"_s] = diagnostics.max_update_rotation_rad;
  thresholds["max_velocity_mps"_s] = diagnostics.max_update_velocity_mps;
  thresholds["max_velocity_delta_mps"_s] =
      diagnostics.max_update_velocity_delta_mps;

  nb::dict information_ldlt;
  information_ldlt["evaluated"_s] = diagnostics.information_ldlt_evaluated;
  information_ldlt["decomposition_success"_s] =
      diagnostics.information_ldlt_decomposition_success;
  information_ldlt["positive"_s] = diagnostics.information_ldlt_positive;

  nb::dict candidate_covariance;
  candidate_covariance["evaluated"_s] =
      diagnostics.candidate_covariance_evaluated;
  candidate_covariance["finite"_s] =
      diagnostics.candidate_covariance_finite;
  candidate_covariance["positive_diagonal"_s] =
      diagnostics.candidate_covariance_positive_diagonal;

  nb::dict posterior_covariance;
  posterior_covariance["evaluated"_s] =
      diagnostics.posterior_covariance_evaluated;
  posterior_covariance["finite"_s] =
      diagnostics.posterior_covariance_finite;
  posterior_covariance["positive_diagonal"_s] =
      diagnostics.posterior_covariance_positive_diagonal;

  nb::dict out;
  out["attempted"_s] = diagnostics.attempted;
  out["accepted"_s] = diagnostics.accepted;
  out["attempt_sequence"_s] = diagnostics.attempt_sequence;
  out["rejection_reason"_s] = diagnostics.rejection_reason;
  out["previous_rejection_reason"_s] =
      diagnostics.previous_rejection_reason;
  out["consecutive_rejections"_s] = diagnostics.consecutive_rejections;
  out["downsampled_points"_s] = diagnostics.downsampled_points;
  out["effective_points"_s] = diagnostics.effective_points;
  out["candidate"_s] = candidate;
  out["thresholds"_s] = thresholds;
  out["information_ldlt"_s] = information_ldlt;
  out["candidate_covariance"_s] = candidate_covariance;
  out["posterior_covariance"_s] = posterior_covariance;
  return out;
}

std::string normalizedBackend(std::string backend) {
  std::transform(backend.begin(), backend.end(), backend.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  if (backend == "fastlio" || backend == "fastlio2" || backend == "localizer") {
    return "fastlio2";
  }
  if (backend == "pointlio") {
    return "pointlio";
  }
  return backend.empty() ? "fastlio2" : backend;
}

std::unique_ptr<ISlamBackend> createBackend(const std::string& backend) {
  const std::string normalized = normalizedBackend(backend);
  if (normalized == "pointlio") {
    return makePointLioBackend();
  }
  if (normalized == "fastlio2") {
    return makeFastLioBackend();
  }
  return makeContractBackend(normalized);
}

class SlamRunner {
 public:
  SlamRunner(std::string backend, std::string mode, std::string map_path)
      : backend_name_(normalizedBackend(std::move(backend))),
        mode_(toString(modeFromString(mode))),
        map_path_(std::move(map_path)),
        backend_(createBackend(backend_name_)) {}

  nb::dict configure(const nb::dict& raw) {
    SlamConfig config;
    config.backend = dictString(raw, "backend", backend_name_);
    config.config_path = dictString(raw, "config_path", config.config_path);
    config.map_frame = dictString(raw, "map_frame", config.map_frame);
    config.odom_frame = dictString(raw, "odom_frame", config.odom_frame);
    config.body_frame = dictString(raw, "body_frame", config.body_frame);
    config.publish_state_estimation_at_scan =
        dictBool(raw, "publish_state_estimation_at_scan", false);
    mode_ = dictString(raw, "mode", mode_);
    map_path_ = dictString(raw, "map_path", map_path_);
    config_ = config;

    Status status;
    {
      nb::gil_scoped_release release;
      status = backend_->configure(config_);
    }
    return statusDict(status);
  }

  nb::dict setMode(const std::string& mode, const std::string& map_path) {
    mode_ = toString(modeFromString(mode));
    if (!map_path.empty()) {
      map_path_ = map_path;
    }
    Status status;
    {
      nb::gil_scoped_release release;
      status = backend_->setMode(modeFromString(mode_), map_path_);
    }
    nb::dict result = statusDict(status);
    result["mode"_s] = mode_;
    return result;
  }

  nb::dict feedImu(const nb::dict& raw) {
    const ImuSample sample = imuFromDict(raw);
    Status status;
    {
      nb::gil_scoped_release release;
      status = backend_->feedImu(sample);
    }
    return statusDict(status);
  }

  nb::dict feedLidar(const nb::dict& raw) {
    const LidarFrame frame = lidarFromDict(raw);
    Status status;
    {
      nb::gil_scoped_release release;
      status = backend_->feedLidar(frame);
    }
    return statusDict(status);
  }

  nb::dict feedGnss(const nb::dict& raw) {
    const GnssSample sample = gnssFromDict(raw);
    Status status;
    {
      nb::gil_scoped_release release;
      status = backend_->feedGnss(sample);
    }
    return statusDict(status);
  }

  nb::dict feedVisualOdom(const nb::dict& raw) {
    const OdomSample sample = odomFromDict(raw);
    Status status;
    {
      nb::gil_scoped_release release;
      status = backend_->feedVisualOdom(sample);
    }
    return statusDict(status);
  }

  nb::dict setInitialPose(const nb::dict& raw) {
    const Pose3d pose = poseFromDict(raw);
    Status status;
    {
      nb::gil_scoped_release release;
      status = backend_->setInitialPose(pose);
    }
    return statusDict(status);
  }

  nb::dict relocalize(nb::object guess) {
    std::optional<Pose3d> pose;
    if (!guess.is_none()) {
      pose = poseFromDict(nb::cast<nb::dict>(guess));
    }
    Status status;
    {
      nb::gil_scoped_release release;
      status = backend_->relocalize(pose);
    }
    return statusDict(status);
  }

  nb::dict tick() {
    Status status;
    {
      nb::gil_scoped_release release;
      status = backend_->tick();
    }
    return statusDict(status);
  }

  nb::dict saveMap(const std::string& path) {
    Status status;
    {
      nb::gil_scoped_release release;
      status = backend_->saveMap(path);
    }
    return statusDictWithMapArtifacts(status, path);
  }

  nb::dict loadMap(const std::string& path) {
    Status status;
    {
      nb::gil_scoped_release release;
      status = backend_->loadMap(path);
    }
    return statusDictWithMapArtifacts(status, path);
  }

  nb::dict outputs() const {
    SlamOutputs out;
    {
      nb::gil_scoped_release release;
      out = backend_->outputs();
    }

    nb::dict result;
    result["state"_s] = toString(out.state);
    result["stamp_s"_s] = out.stamp_s;
    result["confidence"_s] = out.confidence;
    result["reason"_s] = out.reason;
    result["odometry_odom_body"_s] =
        out.odometry_odom_body.has_value()
            ? nb::cast(odomDict(*out.odometry_odom_body, out.stamp_s, config_.odom_frame, config_.body_frame))
            : nb::none();
    result["state_estimation_at_scan"_s] =
        out.state_estimation_at_scan.has_value()
            ? nb::cast(odomDict(*out.state_estimation_at_scan, out.stamp_s, config_.odom_frame, config_.body_frame))
            : nb::none();
    result["registered_cloud_body"_s] =
        out.registered_cloud_body.has_value() ? nb::cast(cloudDict(*out.registered_cloud_body)) : nb::none();
    result["observation_sequence"_s] = out.observation_sequence;
    result["source_epoch"_s] = out.source_epoch;
    result["map_cloud_map"_s] =
        out.map_cloud_map.has_value() ? nb::cast(cloudDict(*out.map_cloud_map)) : nb::none();
    result["saved_map_cloud_map"_s] =
        out.saved_map_cloud_map.has_value() ? nb::cast(cloudDict(*out.saved_map_cloud_map)) : nb::none();
    result["map_odom_tf"_s] =
        out.map_odom_tf.has_value() ? nb::cast(transformDict(*out.map_odom_tf, out.stamp_s)) : nb::none();
    result["alive"_s] = out.alive;
    result["map_loaded"_s] = out.map_loaded;
    result["map_frame_jump"_s] = out.map_frame_jump;
    result["saved_map_points"_s] = out.saved_map_points;
    result["localization_quality"_s] = out.localization_quality;
    result["gnss_fusion_health"_s] = gnssHealthDict(out.gnss_fusion_health);
    result["scene_mode"_s] = out.scene_mode;
    result["scan_start_s"_s] = out.scan_start_s;
    result["scan_end_s"_s] = out.scan_end_s;
    result["last_imu_s"_s] = out.last_imu_s;
    result["imu_batch"_s] = out.imu_batch;
    result["sync_wait_count"_s] = out.sync_wait_count;
    result["imu_rollback_count"_s] = out.imu_rollback_count;
    result["lidar_rollback_count"_s] = out.lidar_rollback_count;
    result["imu_buffer"_s] = out.imu_buffer;
    result["lidar_buffer"_s] = out.lidar_buffer;
    result["dropped_lidar_frames"_s] = out.dropped_lidar_frames;
    result["dropped_imu_frames"_s] = out.dropped_imu_frames;
    result["odom_prior_enabled"_s] = out.odom_prior_enabled;
    result["odom_prior_active"_s] = out.odom_prior_active;
    result["odom_prior_age_s"_s] = out.odom_prior_age_s;
    result["odom_prior_error_xy_m"_s] = out.odom_prior_error_xy_m;
    result["odom_prior_map_points"_s] = out.odom_prior_map_points;
    result["fastlio_lidar_update"_s] =
        fastLioLidarUpdateDict(out.fastlio_lidar_update);
    nb::dict map_optimization;
    map_optimization["status"_s] = out.map_optimization_status;
    map_optimization["backend"_s] = out.map_optimization_backend;
    map_optimization["refine_backend"_s] = out.map_optimization_refine_backend;
    map_optimization["enabled"_s] = out.map_optimization_enabled;
    map_optimization["loop_closure_enabled"_s] =
        out.map_optimization_loop_closure_enabled;
    map_optimization["loop_closure_applied"_s] =
        out.map_optimization_loop_closure_applied;
    map_optimization["refine_enabled"_s] =
        out.map_optimization_refine_enabled;
    map_optimization["refine_applied"_s] =
        out.map_optimization_refine_applied;
    map_optimization["hba_refine_enabled"_s] =
        out.map_optimization_hba_refine_enabled;
    map_optimization["hba_refine_applied"_s] =
        out.map_optimization_hba_refine_applied;
    map_optimization["patch_count"_s] = out.map_optimization_patch_count;
    map_optimization["pose_count"_s] = out.map_optimization_pose_count;
    map_optimization["optimized_pose_count"_s] =
        out.map_optimization_optimized_pose_count;
    map_optimization["loop_count"_s] = out.map_optimization_loop_count;
    map_optimization["raw_map_points"_s] = out.map_optimization_raw_map_points;
    map_optimization["optimized_map_points"_s] =
        out.map_optimization_optimized_map_points;
    map_optimization["loop_closure_error_m"_s] =
        out.map_optimization_loop_error_m;
    result["map_optimization"_s] = map_optimization;
    result["map_optimization_status"_s] = out.map_optimization_status;
    result["map_optimization_loop_count"_s] = out.map_optimization_loop_count;
    result["map_optimization_optimized_pose_count"_s] =
        out.map_optimization_optimized_pose_count;
    result["relocalization_supported"_s] = out.relocalization_supported;
    result["saved_map_relocalization_supported"_s] =
        out.saved_map_relocalization_supported;
    result["relocalization_state"_s] = out.relocalization_state;
    result["last_relocalization_message"_s] = out.last_relocalization_message;
    result["relocalization_quality"_s] = out.relocalization_quality;
    result["relocalization_map_body"_s] =
        out.relocalization_map_body.has_value() ? nb::cast(poseDict(*out.relocalization_map_body)) : nb::none();
    result["relocalization_refine_backend"_s] = out.relocalization_refine_backend;
    result["relocalization_refine_iterations"_s] = out.relocalization_refine_iterations;
    result["relocalization_refine_inliers"_s] = out.relocalization_refine_inliers;
    result["relocalization_min_inliers"_s] = out.relocalization_min_inliers;
    result["relocalization_min_evaluated_points"_s] =
        out.relocalization_min_evaluated_points;
    result["relocalization_refine_input_points"_s] =
        out.relocalization_refine_input_points;
    result["relocalization_refine_evaluated_points"_s] =
        out.relocalization_refine_evaluated_points;
    result["relocalization_refine_support_ratio"_s] =
        out.relocalization_refine_support_ratio;
    result["relocalization_refine_overlap_inlier_ratio"_s] =
        out.relocalization_refine_overlap_inlier_ratio;
    result["relocalization_refine_converged"_s] = out.relocalization_refine_converged;
    result["relocalization_refine_pos_cov_trace"_s] = out.relocalization_refine_pos_cov_trace;
    return result;
  }

  nb::dict reset() {
    Status status;
    {
      nb::gil_scoped_release release;
      status = backend_->reset();
    }
    return statusDict(status);
  }

 private:
  std::string backend_name_;
  std::string mode_;
  std::string map_path_;
  SlamConfig config_;
  std::unique_ptr<ISlamBackend> backend_;
};

}  // namespace

NB_MODULE(_native, m) {
  m.doc() = "LingTu native SLAM runtime binding";
  m.attr("NATIVE_SLAM_BINDING_SCHEMA") = kBindingSchema;
  m.attr("BUILD_SCHEMA") = kBindingSchema;
  nb::class_<SlamRunner>(m, "SlamRunner")
      .def(nb::init<std::string, std::string, std::string>(),
           "backend"_a = "fastlio2",
           "mode"_a = "mapping",
           "map_path"_a = "")
      .def("configure", &SlamRunner::configure)
      .def("setMode", &SlamRunner::setMode)
      .def("feedImu", &SlamRunner::feedImu)
      .def("feedLidar", &SlamRunner::feedLidar)
      .def("feedGnss", &SlamRunner::feedGnss)
      .def("feedVisualOdom", &SlamRunner::feedVisualOdom)
      .def("setInitialPose", &SlamRunner::setInitialPose)
      .def("relocalize", &SlamRunner::relocalize, "guess"_a = nb::none())
      .def("tick", &SlamRunner::tick)
      .def("saveMap", &SlamRunner::saveMap)
      .def("loadMap", &SlamRunner::loadMap)
      .def("outputs", &SlamRunner::outputs)
      .def("reset", &SlamRunner::reset);
}

}  // namespace lingtu::slam
