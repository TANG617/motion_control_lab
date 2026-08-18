#pragma once

#include <json/json.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace motion_control_lab::baseline
{

struct SourceDigest
{
  std::string path;
  std::string sha256;
};

struct TaskSpec
{
  std::string name;
  std::string type;
  std::string priority;
  double weight{};
  std::vector<std::string> joints;
};

struct ConstraintSpec
{
  std::string name;
  std::string type;
  std::string priority;
  double weight{};
  std::vector<std::string> joints;
};

struct ProductionStaticConfig
{
  std::string schema_version;
  std::string source_revision;
  std::vector<SourceDigest> source_digests;

  std::string base_frame;
  std::string left_end_effector_frame;
  std::string right_end_effector_frame;
  std::vector<double> left_tcp_offset_xyz;
  std::vector<double> right_tcp_offset_xyz;

  std::vector<std::string> joint_names;
  std::vector<std::string> active_joint_names;
  std::vector<std::string> masked_joint_names;
  std::vector<double> initial_positions;
  std::vector<double> lower_limits;
  std::vector<double> upper_limits;
  std::vector<double> velocity_limits;
  std::vector<double> posture_joint_weights;
  double posture_profile_default_weight{};

  bool use_sparsity{};
  bool rewrite_equalities{};
  double problem_regularization{};
  double solver_dt{};
  double control_rate_hz{};
  double control_dt_s{};
  double soft_limit_margin{};
  int maximum_iterations{};
  double soft_solve_time_budget_ms{};
  double position_tolerance_m{};
  double orientation_tolerance_rad{};
  double minimum_position_improvement_m{};
  double minimum_orientation_improvement_rad{};

  std::vector<TaskSpec> tasks;
  std::vector<ConstraintSpec> constraints;
  std::vector<std::string> disabled_features;
};

const ProductionStaticConfig & productionStaticConfig();

std::size_t jointIndex(const ProductionStaticConfig & config, const std::string & joint_name);
bool isActiveJoint(const ProductionStaticConfig & config, const std::string & joint_name);
double limitedLower(const ProductionStaticConfig & config, std::size_t joint_index);
double limitedUpper(const ProductionStaticConfig & config, std::size_t joint_index);

const TaskSpec & taskSpec(const ProductionStaticConfig & config, const std::string & name);
const ConstraintSpec & constraintSpec(const ProductionStaticConfig & config,
                                      const std::string & name);

Json::Value productionStaticConfigJson();
std::string productionStaticConfigJsonText();

} // namespace motion_control_lab::baseline
