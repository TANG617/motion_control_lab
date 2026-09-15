#include "elbow_reference.hpp"
#include <motion_control_lab/execution_request.hpp>
#include <set>
#ifdef MCL_HAS_HARP
#include <motion_control_core/posture_reference/predictor.hpp>
#endif
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <json/json.h>
#include <pthread.h>
#include <sstream>
#include <stdexcept>
#include <unistd.h>

namespace motion_control_lab::hierarchical_kinematics_step {
struct ElbowReference::HarpState {
#ifdef MCL_HAS_HARP
  motion_control::core::posture_reference::Predictor predictor;
#endif
  std::ofstream windows;
};

namespace {
ElbowCircle circle(const ElbowGeometry &g, const Eigen::Vector3d &s,
              const Eigen::Vector3d &w) {
  const double distance = (w - s).norm();
  if (!s.allFinite() || !w.allFinite() || distance <= 1e-9 ||
      distance >= g.upper_length + g.lower_length - 1e-9 ||
      distance <= std::abs(g.upper_length - g.lower_length) + 1e-9)
    throw std::runtime_error(
        "elbow geometry: degenerate or unreachable shoulder-wrist circle");
  const Eigen::Vector3d n = (w - s) / distance;
  Eigen::Vector3d axis(-1, 0, 0);
  Eigen::Vector3d u = axis - axis.dot(n) * n;
  if (u.norm() < 1e-6) {
    axis = Eigen::Vector3d(0, 1, 0);
    u = axis - axis.dot(n) * n;
  }
  u.normalize();
  const double along = (g.upper_length * g.upper_length -
                        g.lower_length * g.lower_length + distance * distance) /
                       (2 * distance);
  const double r2 = g.upper_length * g.upper_length - along * along;
  if (!(r2 > 1e-18))
    throw std::runtime_error("elbow geometry: zero circle radius");
  return {s + along * n, u, n.cross(u), std::sqrt(r2)};
}
std::array<double, 16> matrix(const Eigen::Isometry3d &pose) {
  std::array<double, 16> result{};
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      result[r * 4 + c] = pose.matrix()(r, c);
  return result;
}
Json::Value parse(const std::string &text) {
  Json::CharReaderBuilder b;
  Json::Value result;
  std::string error;
  std::istringstream input(text);
  if (!Json::parseFromStream(b, input, &result, &error))
    throw std::runtime_error("elbow JSON: " + error);
  return result;
}
std::string encode(const Json::Value &value) {
  Json::StreamWriterBuilder b;
  b["indentation"] = "";
  return Json::writeString(b, value) + "\n";
}
ElbowPrediction prediction(const Json::Value &j) {
  for (auto key : {"generation", "sequence", "sample_time_s", "cos", "sin",
                   "inference_ms"})
    if (!j.isMember(key) || !j[key].isNumeric())
      throw std::runtime_error(std::string("elbow response missing numeric ") +
                               key);
  ElbowPrediction p{
      j["generation"].asUInt64(),    j["sequence"].asUInt64(),
      j["sample_time_s"].asDouble(), j["cos"].asDouble(),
      j["sin"].asDouble(),           j["inference_ms"].asDouble()};
  const double norm = std::hypot(p.cosine, p.sine);
  if (!std::isfinite(norm) || norm < 1e-9 || !std::isfinite(p.sample_time_s) ||
      !std::isfinite(p.inference_ms) || p.inference_ms < 0)
    throw std::runtime_error(
        "elbow response contains an invalid arm angle or timing");
  p.cosine /= norm;
  p.sine /= norm;
  return p;
}
Json::Value json(const ElbowConsumption &v) {
  Json::Value j;
  const auto &p = v.prediction;
  j["generation"] = Json::UInt64(p.generation);
  j["sequence"] = Json::UInt64(p.sequence);
  j["sample_time_s"] = p.sample_time_s;
  j["cos"] = p.cosine;
  j["sin"] = p.sine;
  j["inference_ms"] = p.inference_ms;
  j["target_revision"] = Json::UInt64(v.target_revision);
  j["mirror_tcp_input"] = v.mirror_tcp_input;
  j["source_index"] = Json::UInt64(v.source_index);
  for (double x : v.left_goal)
    j["left_goal"].append(x);
  for (double x : v.right_goal)
    j["right_goal"].append(x);
  j["consume_time_s"] = v.time_s;
  j["age_ms"] = (v.time_s - p.sample_time_s) * 1000;
  for (int i = 0; i < 3; ++i) {
    j["target_elbow"].append(v.target[i]);
    j["raw_elbow"].append(v.raw[i]);
    j["executed_elbow"].append(v.executed[i]);
  }
#define FIELD(name) j[#name] = v.name
  FIELD(raw_position_error_m);
  FIELD(raw_orientation_error_rad);
  FIELD(executed_position_error_m);
  FIELD(executed_orientation_error_rad);
  FIELD(hard_violation);
  FIELD(selected_shared_hard_violation);
  FIELD(primary_position_drift);
  FIELD(primary_orientation_drift);
  FIELD(scale_drift);
  FIELD(secondary_executed);
  FIELD(secondary_succeeded);
  FIELD(selected_priority);
#undef FIELD
  return j;
}
} // namespace
ElbowGeometry ElbowGeometry::fromFk(const Eigen::Isometry3d &s,
                                    const Eigen::Isometry3d &e,
                                    const Eigen::Isometry3d &w,
                                    const Eigen::Isometry3d &ee) {
  ElbowGeometry g{(e.translation() - s.translation()).norm(),
                  (w.translation() - e.translation()).norm(),
                  ee.inverse() * w.translation()};
  (void)circle(g, s.translation(), w.translation());
  return g;
}
ElbowCircle ElbowGeometry::referenceCircle(const Eigen::Vector3d &s,
                                          const Eigen::Isometry3d &ee) const {
  return circle(*this, s, ee * wrist_in_ee);
}
Eigen::Vector3d ElbowGeometry::target(const Eigen::Vector3d &s,
                                      const Eigen::Isometry3d &ee, double c,
                                      double sn) const {
  const double norm = std::hypot(c, sn);
  if (!std::isfinite(norm) || norm < 1e-9)
    throw std::runtime_error("elbow geometry: invalid arm angle");
  const auto k = referenceCircle(s, ee);
  return k.center + k.radius * (c * k.u + sn * k.v) / norm;
}
Eigen::Vector2d ElbowGeometry::angle(const Eigen::Vector3d &s,
                                     const Eigen::Vector3d &w,
                                     const Eigen::Vector3d &e) const {
  const auto k = circle(*this, s, w);
  const Eigen::Vector3d d = e - k.center;
  Eigen::Vector2d result(d.dot(k.u), d.dot(k.v));
  if (!result.allFinite() || result.norm() < 1e-9)
    throw std::runtime_error("elbow geometry: elbow has no swivel direction");
  return result.normalized();
}
ElbowReference::ElbowReference(ElbowReferenceOptions options,
                               double red_rate_hz)
    : options_(std::move(options)),
      sample_period_s_(1 / options_.sample_rate_hz),
      records_(
          std::make_unique<std::array<ElbowConsumption, record_capacity>>()) {
  if (options_.sample_rate_hz > red_rate_hz)
    throw std::runtime_error("elbow sample rate exceeds Red rate");
}
ElbowReference::~ElbowReference() {
  stopping_ = true;
  if (worker_.joinable())
    worker_.join();
}
void ElbowReference::initialize(const Eigen::Isometry3d &ee,
                                std::uint64_t generation) {
  if (worker_.joinable())
    throw std::runtime_error("ElbowReference initialization requires a new run instance");
  window_.generation = generation;
  window_.sequence = 0;
  window_.sample_time_s = 0;
  window_.poses.fill(matrix(ee));
  next_sample_s_ = sample_period_s_;
  if (!options_.record_path.empty() && !options_.recorded_path.empty() &&
      std::filesystem::weakly_canonical(options_.record_path) ==
          std::filesystem::weakly_canonical(options_.recorded_path))
    throw std::runtime_error(
        "elbow recording output must differ from its input");
  if (!options_.record_path.empty()) {
    const auto parent =
        std::filesystem::path(options_.record_path).parent_path();
    if (!parent.empty())
      std::filesystem::create_directories(parent);
    stream_.exceptions(std::ios::badbit | std::ios::failbit);
    stream_.open(options_.record_path);
  }
  if (options_.source == "recorded") {
    std::ifstream file(options_.recorded_path);
    if (!file)
      throw std::runtime_error("cannot read elbow references: " +
                               options_.recorded_path);
    std::string line;
    double previous = -1;
    while (std::getline(file, line)) {
      auto j = parse(line);
      ElbowConsumption v;
      v.mirror_tcp_input = j.get("mirror_tcp_input", false).asBool();
      v.prediction = prediction(j);
      if (!j["consume_time_s"].isNumeric())
        throw std::runtime_error(
            "recorded elbow reference lacks consume_time_s");
      v.time_s = j["consume_time_s"].asDouble();
      v.target_revision = j["target_revision"].asUInt64();
      v.source_index = j["source_index"].asUInt64();
      if (j["left_goal"].size() != 7 || j["right_goal"].size() != 7)
        throw std::runtime_error(
            "recorded reference lacks consumed Cartesian goals");
      for (int axis = 0; axis < 7; ++axis) {
        if (!j["left_goal"][axis].isNumeric() ||
            !j["right_goal"][axis].isNumeric())
          throw std::runtime_error("recorded Cartesian goal must be numeric");
        v.left_goal[axis] = j["left_goal"][axis].asDouble();
        v.right_goal[axis] = j["right_goal"][axis].asDouble();
        if (!std::isfinite(v.left_goal[axis]) ||
            !std::isfinite(v.right_goal[axis]))
          throw std::runtime_error("recorded Cartesian goal must be finite");
      }
      if (!std::isfinite(v.time_s) || v.time_s < previous ||
          v.prediction.generation != generation)
        throw std::runtime_error(
            "recorded elbow reference has invalid time order or generation");
      for (const auto *goal : {&v.left_goal, &v.right_goal}) {
        double norm = 0;
        for (int axis = 3; axis < 7; ++axis)
          norm += (*goal)[axis] * (*goal)[axis];
        if (std::abs(norm - 1.0) > 1e-5)
          throw std::runtime_error(
              "recorded Cartesian goal requires a unit quaternion");
      }
      previous = v.time_s;
      recorded_.push_back(v);
    }
    if (recorded_.empty() || recorded_.front().time_s > 1e-9)
      throw std::runtime_error(
          "recorded elbow references require an initial sample at time zero");
    latest_ = recorded_.front().prediction;
  } else if (options_.source == "harp") {
#ifdef MCL_HAS_HARP
    harp_ = std::make_unique<HarpState>();
    auto status = harp_->predictor.initialize({options_.harp_model_directory, options_.harp_device});
    if (!status.ok()) throw std::runtime_error(status.message);
    const auto &manifest = harp_->predictor.manifest();
    if (manifest.sample_hz != options_.sample_rate_hz)
      throw std::runtime_error("HARP manifest sampling rate does not match application");
    if (!options_.record_path.empty()) {
      harp_->windows.exceptions(std::ios::badbit | std::ios::failbit);
      harp_->windows.open(options_.record_path + ".windows.jsonl");
      std::ofstream provenance(options_.record_path + ".model.json");
      provenance.exceptions(std::ios::badbit | std::ios::failbit);
      provenance << manifest.json << '\n';
    }
#else
    throw std::runtime_error("HARP requires MCL_BUILD_HARP=ON");
#endif
  } else {
    throw std::runtime_error("ElbowReference requires recorded or harp; manual is owned by the control loop");
  }
  auto ready = worker_ready_.get_future();
  worker_ = std::thread([this] { work(); });
  ready.get();
}
void ElbowReference::checkError() const {
  if (failed_.load(std::memory_order_acquire))
    throw std::runtime_error(error_.data());
}
ElbowPrediction ElbowReference::consume(double time) {
  checkError();
  if (options_.source == "recorded") {
    if (time > recorded_.back().time_s + 1e-8)
      throw std::runtime_error("recorded elbow references exhausted");
    while (recorded_index_ + 1 < recorded_.size() &&
           recorded_[recorded_index_ + 1].time_s <= time + 1e-9)
      ++recorded_index_;
    latest_ = recorded_[recorded_index_].prediction;
  } else {
    responses_.readLatest(latest_);
  }
  if (latest_.generation != window_.generation)
    throw std::runtime_error("elbow reference belongs to a different run generation");
  if (time - latest_.sample_time_s > options_.maximum_age_ms / 1000 + 1e-9)
    throw std::runtime_error(
        "elbow reference exceeded maximum active-time age");
  if (latest_.sample_time_s > time + 1e-9)
    throw std::runtime_error("elbow reference is from the future");
  return latest_;
}
void ElbowReference::sampleAccepted(double time, const Eigen::Isometry3d &ee) {
  if (options_.source != "harp" || time + 1e-9 < next_sample_s_)
    return;
  if (time > next_sample_s_ + 1e-7)
    throw std::runtime_error(
        "elbow reference sampling missed an active-time sample");
  std::move(window_.poses.begin() + 1, window_.poses.end(),
            window_.poses.begin());
  window_.poses.back() = matrix(ee);
  window_.sample_time_s = next_sample_s_;
  ++window_.sequence;
  next_sample_s_ = (window_.sequence + 1) * sample_period_s_;
  requests_.publish(window_);
}
void ElbowReference::record(const ElbowConsumption &v) {
  checkError();
  if (options_.record_path.empty())
    return;
  const auto w = write_.load(std::memory_order_relaxed);
  const auto next = (w + 1) % record_capacity;
  if (next == read_.load(std::memory_order_acquire))
    throw std::runtime_error("elbow reference recording queue overflow");
  (*records_)[w] = v;
  write_.store(next, std::memory_order_release);
}
void ElbowReference::flushRecords() {
  auto r = read_.load(std::memory_order_relaxed);
  while (r != write_.load(std::memory_order_acquire)) {
    stream_ << encode(json((*records_)[r]));
    r = (r + 1) % record_capacity;
    read_.store(r, std::memory_order_release);
  }
}
ElbowPrediction ElbowReference::inferHarp(const ElbowWindow &w) {
#ifdef MCL_HAS_HARP
  motion_control::core::posture_reference::PoseWindow input;
  constexpr std::array<int,9> indices{3,7,11,0,1,4,5,8,9};
  for (std::size_t t=0; t<30; ++t)
    for (std::size_t i=0; i<9; ++i)
      input.values[t*9+i] = static_cast<float>(w.poses[t][indices[i]]);
  motion_control::core::posture_reference::PosturePrediction output;
  auto status = harp_->predictor.predict(input, output);
  if (!status.ok()) throw std::runtime_error(status.message);
  ElbowPrediction result{w.generation,w.sequence,w.sample_time_s,
    output.arm_angle[0],output.arm_angle[1],output.inference_ms};
  if (harp_->windows.is_open()) {
    Json::Value j;
    j["generation"]=Json::UInt64(w.generation); j["sequence"]=Json::UInt64(w.sequence);
    j["sample_time_s"]=w.sample_time_s; j["cos"]=result.cosine; j["sin"]=result.sine;
    j["inference_ms"]=result.inference_ms;
    for (float x:input.values) j["window"].append(x);
    harp_->windows << encode(j);
  }
  return result;
#else
  (void)w;
  throw std::runtime_error("HARP requires MCL_BUILD_HARP=ON");
#endif
}
void ElbowReference::work() {
  bool ready = false;
  try {
    sched_param parameter{};
    int rc = pthread_setschedparam(pthread_self(), SCHED_OTHER, &parameter);
    if (rc)
      throw std::runtime_error("cannot set elbow I/O worker to SCHED_OTHER: " +
                               std::string(std::strerror(rc)));
    if (options_.source == "harp") {
      // Warm the exact inference thread before the caller starts periodic work.
      for (int i=0; i<5; ++i) latest_ = inferHarp(window_);
      if (!options_.record_path.empty()) {
        std::ifstream maps("/proc/self/maps");std::string line;std::set<std::string> paths;
        while(std::getline(maps,line)){const auto pos=line.find('/');if(pos!=std::string::npos){auto path=line.substr(pos);if(path.find(".so")!=std::string::npos&&std::filesystem::is_regular_file(path))paths.insert(path);}}
        Json::Value provenance;provenance["schema_version"]="harp_loaded_runtime.v1";provenance["worker_policy"]="SCHED_OTHER";
        for(const auto &path:paths){Json::Value library;library["path"]=path;library["sha256"]=sha256_file(path);provenance["libraries"].append(library);}
        execution::writeJson(options_.record_path+".runtime.json",provenance);
      }
    }
    worker_ready_.set_value();
    ready = true;
    while (!stopping_) {
      flushRecords();
      ElbowWindow request;
      if ((options_.source == "harp") && requests_.readLatest(request))
        responses_.publish(inferHarp(request));
      else
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    flushRecords();
    if (stream_.is_open())
      stream_.flush();
    if (harp_ && harp_->windows.is_open()) harp_->windows.flush();
  } catch (const std::exception &e) {
    // Propagate the original worker failure to the existing Red exception path.
    std::strncpy(error_.data(), e.what(), error_.size() - 1);
    failed_.store(true, std::memory_order_release);
    if (!ready) worker_ready_.set_exception(std::current_exception());
  }
}
void ElbowReference::finish() {
  stopping_ = true;
  if (worker_.joinable())
    worker_.join();
}
} // namespace motion_control_lab::hierarchical_kinematics_step
