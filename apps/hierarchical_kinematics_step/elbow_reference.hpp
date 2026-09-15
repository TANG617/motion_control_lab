#pragma once

#include "components/scheduler/latest_value_mailbox.hpp"
#include "options.hpp"
#include <array>
#include <atomic>
#include <fstream>
#include <future>
#include <memory>
#include <thread>

namespace motion_control_lab::hierarchical_kinematics_step {

struct ElbowCircle {
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  Eigen::Vector3d u{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v{Eigen::Vector3d::Zero()};
  double radius{0};
};

struct ElbowGeometry {
  double upper_length{}, lower_length{};
  Eigen::Vector3d wrist_in_ee{Eigen::Vector3d::Zero()};
  ElbowCircle referenceCircle(const Eigen::Vector3d &shoulder,
                             const Eigen::Isometry3d &ee) const;
  static ElbowGeometry fromFk(const Eigen::Isometry3d &shoulder,
                              const Eigen::Isometry3d &elbow,
                              const Eigen::Isometry3d &wrist,
                              const Eigen::Isometry3d &ee);
  Eigen::Vector3d target(const Eigen::Vector3d &shoulder,
                         const Eigen::Isometry3d &ee, double cosine,
                         double sine) const;
  Eigen::Vector2d angle(const Eigen::Vector3d &shoulder,
                        const Eigen::Vector3d &wrist,
                        const Eigen::Vector3d &elbow) const;
};

// Row-major homogeneous transforms; the full uniformly sampled history travels
// across the mailbox even when the worker skips intermediate requests.
struct ElbowWindow {
  std::uint64_t generation{2}, sequence{0};
  double sample_time_s{0};
  std::array<std::array<double, 16>, 30> poses{};
};
struct ElbowPrediction {
  std::uint64_t generation{2}, sequence{0};
  double sample_time_s{0}, cosine{1}, sine{0}, inference_ms{0};
};
struct ElbowConsumption {
  bool mirror_tcp_input{false};
  double time_s{0};
  std::uint64_t target_revision{0}, source_index{0};
  std::array<double, 7> left_goal{}, right_goal{};
  ElbowPrediction prediction;
  std::array<double, 3> target{}, raw{}, executed{};
  double raw_position_error_m{}, raw_orientation_error_rad{};
  double executed_position_error_m{}, executed_orientation_error_rad{};
  double selected_shared_hard_violation{};
  double hard_violation{}, primary_position_drift{},
      primary_orientation_drift{}, scale_drift{};
  bool secondary_executed{}, secondary_succeeded{};
  int selected_priority{-1};
};

class ElbowReference {
public:
  explicit ElbowReference(ElbowReferenceOptions options, double red_rate_hz);
  ~ElbowReference();
  ElbowReference(const ElbowReference &) = delete;
  ElbowReference &operator=(const ElbowReference &) = delete;
  // All file access and native predictor initialization finish before periodic workers start.
  void initialize(const Eigen::Isometry3d &initial_ee,
                  std::uint64_t generation = 2);
  ElbowPrediction consume(double active_time_s);
  void sampleAccepted(double active_time_s, const Eigen::Isometry3d &ee);
  void record(const ElbowConsumption &value);
  void finish();
  std::string failureDetail() const {
    return failed_.load(std::memory_order_acquire) ? error_.data() : "";
  }
  double recordedEndTime() const { return recorded_.back().time_s; }
  const ElbowConsumption &recordedConsumption() const {
    return recorded_[recorded_index_];
  }

private:
  struct HarpState;
  std::unique_ptr<HarpState> harp_;
  std::promise<void> worker_ready_;
  ElbowPrediction inferHarp(const ElbowWindow &window);
  void work();
  void flushRecords();
  void checkError() const;
  ElbowReferenceOptions options_;
  double sample_period_s_;
  double next_sample_s_{};
  ElbowWindow window_;
  ElbowPrediction latest_;
  motion_control_lab::LatestValueMailbox<ElbowWindow> requests_{ElbowWindow{}};
  motion_control_lab::LatestValueMailbox<ElbowPrediction> responses_{
      ElbowPrediction{}};
  static constexpr std::size_t record_capacity = 8192;
  std::unique_ptr<std::array<ElbowConsumption, record_capacity>> records_;
  std::atomic<std::size_t> read_{0}, write_{0};
  std::vector<ElbowConsumption> recorded_;
  std::size_t recorded_index_{};
  std::ofstream stream_;
  std::thread worker_;
  std::atomic_bool stopping_{false}, failed_{false};
  std::array<char, 1024> error_{};
};
} // namespace motion_control_lab::hierarchical_kinematics_step
