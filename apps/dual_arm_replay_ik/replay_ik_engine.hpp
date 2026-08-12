#pragma once

#include <Eigen/Geometry>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "apps/dual_arm_replay_ik/replay_support.hpp"

namespace motion_control_lab::replay
{

struct R1ReplayIkContract
{
  std::string base_frame;
  std::string left_end_effector;
  std::string right_end_effector;
  Eigen::Isometry3d left_tcp_offset{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d right_tcp_offset{Eigen::Isometry3d::Identity()};
  std::vector<std::string> joint_names;
  std::vector<double> fallback_initial_positions;
};

const R1ReplayIkContract & r1ReplayIkContract();

struct ReplayIkVisualizationSample
{
  std::uint64_t sequence{};
  std::int64_t sample_time_ns{};
  std::string left_target_frame_id;
  Eigen::Isometry3d left_input_target{Eigen::Isometry3d::Identity()};
  std::string right_target_frame_id;
  Eigen::Isometry3d right_input_target{Eigen::Isometry3d::Identity()};
  std::string forward_kinematics_frame_id;
  Eigen::Isometry3d left_end_effector_fk{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d right_end_effector_fk{Eigen::Isometry3d::Identity()};
  std::vector<std::string> joint_names;
  std::vector<double> positions;
  std::vector<double> velocities;
  std::string status;
  bool paused{};
  bool solve_accepted{};
  double solve_time_ms{};
};

using ReplayIkVisualizationCallback = std::function<void(const ReplayIkVisualizationSample &)>;

struct ReplayIkExecutionConfig
{
  bool stop_on_first_error{};
  std::uint64_t first_visualization_sequence{};
  std::function<void()> before_replay;
  ReplayIkVisualizationCallback initial_frame_gate;
  ReplayIkVisualizationCallback visualization_callback;
};

struct ReplayIkCaseResult
{
  LoadedReplay loaded;
  std::vector<double> initial_positions;
  std::size_t frames_planned{};
  std::size_t frames_attempted{};
  std::size_t accepted_solves{};
  std::size_t rejected_solves{};
  std::size_t deadline_misses{};
  std::optional<std::uint64_t> first_failure_frame;
  std::string first_failure_code;
  std::string first_failure_message;
  std::uint64_t next_visualization_sequence{};
  std::string trace_csv;

  bool completed() const;
};

std::string replayIkTraceHeader();

ReplayIkCaseResult executeReplayIkCase(
  const ReplayOptions & options, const ReplayIkExecutionConfig & execution_config = {});

}  // namespace motion_control_lab::replay
