#include "e01_build_config.hpp"

#include "motion_control_lab/run_artifacts.hpp"
#include "motion_control_lab/sha256.hpp"
#include "placo/kinematics/kinematics_solver.h"
#include "placo/model/robot_wrapper.h"

#include <Eigen/Core>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

namespace
{
namespace fs = std::filesystem;
using motion_control_lab::RunArtifacts;
using motion_control_lab::RunMetadata;

struct Options
{
  fs::path output_root =
    std::string(motion_control_lab::e01::build_config::kDefaultOutputRoot);
  std::optional<std::string> run_id;
};

struct SolveResult
{
  bool converged = false;
  int iterations = 0;
  double initial_error_m = std::numeric_limits<double>::quiet_NaN();
  double final_error_m = std::numeric_limits<double>::quiet_NaN();
  double mean_solve_time_us = 0.0;
  double max_solve_time_us = 0.0;
  int joint_limit_violation_count = 0;
  std::string trace_csv;
};

void print_usage(const char* executable)
{
  std::cout << "Usage: " << executable
            << " [--output-root PATH] [--run-id ID]\n"
               "\n"
               "Runs E01 with the pinned definition and writes an append-only run bundle.\n";
}

Options parse_options(int argc, char** argv)
{
  Options options;
  for (int index = 1; index < argc; ++index)
  {
    const std::string argument = argv[index];
    if (argument == "--help" || argument == "-h")
    {
      print_usage(argv[0]);
      std::exit(0);
    }
    if (argument == "--output-root" || argument == "--run-id")
    {
      if (index + 1 >= argc)
      {
        throw std::invalid_argument("Missing value for " + argument);
      }
      const std::string value = argv[++index];
      if (argument == "--output-root")
      {
        options.output_root = value;
      }
      else
      {
        options.run_id = value;
      }
      continue;
    }
    throw std::invalid_argument("Unknown argument: " + argument);
  }
  return options;
}

std::string read_text_file(const fs::path& path)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream)
  {
    throw std::runtime_error("Unable to read file: " + path.string());
  }
  std::ostringstream contents;
  contents << stream.rdbuf();
  return contents.str();
}

void validate_definition(const Json::Value& definition)
{
  if (!definition.isObject() || definition["schema_version"].asString() != "experiment.v1" ||
      definition["experiment_id"].asString() != "E01")
  {
    throw std::runtime_error("Definition is not an E01 experiment.v1 declaration");
  }
  if (!definition["controlled_factors"].isObject() || !definition["inputs"].isArray() ||
      definition["inputs"].size() != 1 || !definition["arms"].isArray() ||
      definition["arms"].size() != 1)
  {
    throw std::runtime_error("E01 definition must declare one input and one arm");
  }

  const auto& factors = definition["controlled_factors"];
  if (!factors["target_position_m"].isArray() || factors["target_position_m"].size() != 3 ||
      !factors["initial_joints_rad"].isObject() ||
      factors["maximum_iterations"].asInt() <= 0 ||
      factors["position_tolerance_m"].asDouble() <= 0.0)
  {
    throw std::runtime_error("E01 controlled factors are incomplete or invalid");
  }
}

Eigen::Vector3d target_from_definition(const Json::Value& definition)
{
  const auto& target = definition["controlled_factors"]["target_position_m"];
  return {target[0].asDouble(), target[1].asDouble(), target[2].asDouble()};
}

int count_joint_limit_violations(placo::model::RobotWrapper& robot)
{
  int violations = 0;
  for (const std::string joint : {"joint1", "joint2"})
  {
    const auto [lower, upper] = robot.get_joint_limits(joint);
    const auto value = robot.get_joint(joint);
    if (value < lower - 1e-12 || value > upper + 1e-12)
    {
      ++violations;
    }
  }
  return violations;
}

SolveResult run_solver(const Json::Value& definition, const fs::path& model_path)
{
  const auto& factors = definition["controlled_factors"];
  const Eigen::Vector3d target = target_from_definition(definition);
  const int maximum_iterations = factors["maximum_iterations"].asInt();
  const double tolerance_m = factors["position_tolerance_m"].asDouble();
  const double regularization_weight = factors["regularization_weight"].asDouble();
  const std::string frame = factors["end_effector_frame"].asString();

  placo::model::RobotWrapper robot(
    model_path.string(),
    placo::model::RobotWrapper::IGNORE_COLLISIONS);
  robot.set_joint("joint1", factors["initial_joints_rad"]["joint1"].asDouble());
  robot.set_joint("joint2", factors["initial_joints_rad"]["joint2"].asDouble());
  robot.update_kinematics();

  placo::kinematics::KinematicsSolver solver(robot);
  solver.mask_fbase(factors["floating_base_masked"].asBool());
  solver.enable_joint_limits(factors["joint_limits_enabled"].asBool());
  auto& position_task = solver.add_position_task(frame, target);
  position_task.configure("tool_position", "soft", 1.0);
  solver.add_regularization_task(regularization_weight);

  SolveResult result;
  const auto initial_position = robot.get_T_world_frame(frame).translation();
  result.initial_error_m = (target - initial_position).norm();

  std::ostringstream trace;
  trace << std::setprecision(17);
  trace << "iteration,target_x_m,target_y_m,target_z_m,achieved_x_m,achieved_y_m,"
           "achieved_z_m,position_error_m,solve_time_us,joint1_rad,joint2_rad\n";
  trace << 0 << ',' << target.x() << ',' << target.y() << ',' << target.z() << ','
        << initial_position.x() << ',' << initial_position.y() << ',' << initial_position.z() << ','
        << result.initial_error_m << ',' << 0.0 << ',' << robot.get_joint("joint1") << ','
        << robot.get_joint("joint2") << '\n';

  double total_solve_time_us = 0.0;
  double current_error_m = result.initial_error_m;
  result.joint_limit_violation_count += count_joint_limit_violations(robot);

  for (int iteration = 1;
       iteration <= maximum_iterations && current_error_m > tolerance_m;
       ++iteration)
  {
    const auto start = std::chrono::steady_clock::now();
    const Eigen::VectorXd delta = solver.solve(true);
    const auto stop = std::chrono::steady_clock::now();
    const double solve_time_us =
      std::chrono::duration<double, std::micro>(stop - start).count();

    if (!delta.allFinite() || !robot.state.q.allFinite())
    {
      throw std::runtime_error("PlaCo returned a non-finite solver output");
    }

    robot.update_kinematics();
    const auto achieved = robot.get_T_world_frame(frame).translation();
    current_error_m = (target - achieved).norm();
    if (!std::isfinite(current_error_m))
    {
      throw std::runtime_error("PlaCo produced a non-finite task error");
    }

    total_solve_time_us += solve_time_us;
    result.max_solve_time_us = std::max(result.max_solve_time_us, solve_time_us);
    result.iterations = iteration;
    result.joint_limit_violation_count += count_joint_limit_violations(robot);

    trace << iteration << ',' << target.x() << ',' << target.y() << ',' << target.z() << ','
          << achieved.x() << ',' << achieved.y() << ',' << achieved.z() << ',' << current_error_m
          << ',' << solve_time_us << ',' << robot.get_joint("joint1") << ','
          << robot.get_joint("joint2") << '\n';
  }

  result.final_error_m = current_error_m;
  result.converged = current_error_m <= tolerance_m;
  if (result.iterations > 0)
  {
    result.mean_solve_time_us = total_solve_time_us / result.iterations;
  }
  result.trace_csv = trace.str();
  return result;
}

std::string make_metrics_csv(const std::string& run_id, const SolveResult& result)
{
  std::ostringstream metrics;
  metrics << std::setprecision(17);
  metrics << "source_id,input_id,arm_id,window_id,metric_id,metric_version,value,unit,"
             "direction,role,status,sample_count,notes\n";

  const auto row = [&](const std::string& metric_id,
                       double value,
                       const std::string& unit,
                       const std::string& direction,
                       const std::string& role,
                       int sample_count,
                       const std::string& notes) {
    metrics << run_id << ",synthetic_two_link,placo_v0_9_23,full_solve," << metric_id
            << ",v1," << value << ',' << unit << ',' << direction << ',' << role
            << ",available," << sample_count << ',' << notes << '\n';
  };

  row("final_position_error_m", result.final_error_m, "m", "lower", "primary", 1, "");
  row("iteration_count", result.iterations, "count", "lower", "secondary", 1, "");
  row(
    "joint_limit_violation_count",
    result.joint_limit_violation_count,
    "count",
    "lower",
    "guardrail",
    result.iterations + 1,
    "");
  row("initial_position_error_m", result.initial_error_m, "m", "none", "diagnostic", 1, "");
  row(
    "mean_solve_time_us",
    result.mean_solve_time_us,
    "us",
    "none",
    "diagnostic",
    result.iterations,
    "smoke-test timing only");
  row(
    "max_solve_time_us",
    result.max_solve_time_us,
    "us",
    "none",
    "diagnostic",
    result.iterations,
    "smoke-test timing only");
  return metrics.str();
}

Json::Value make_arm_status(const SolveResult& result, double tolerance_m)
{
  Json::Value status(Json::objectValue);
  status["schema_version"] = "arm_status.v1";
  status["status"] = result.converged ? "completed" : "failed";
  status["iterations"] = result.iterations;
  status["initial_position_error_m"] = result.initial_error_m;
  status["final_position_error_m"] = result.final_error_m;
  status["position_tolerance_m"] = tolerance_m;
  status["joint_limit_violation_count"] = result.joint_limit_violation_count;
  return status;
}

std::string make_report(const SolveResult& result, double tolerance_m)
{
  std::ostringstream report;
  report << std::setprecision(8);
  report << "# E01 PlaCo C++ smoke-test report\n\n"
         << "- Status: " << (result.converged ? "PASS" : "FAIL") << "\n"
         << "- Initial position error: " << result.initial_error_m << " m\n"
         << "- Final position error: " << result.final_error_m << " m\n"
         << "- Declared tolerance: " << tolerance_m << " m\n"
         << "- Solver iterations: " << result.iterations << "\n"
         << "- Joint-limit violations: " << result.joint_limit_violation_count << "\n"
         << "- Mean solve time: " << result.mean_solve_time_us << " us\n"
         << "- Maximum solve time: " << result.max_solve_time_us << " us\n\n"
         << "Timing values only prove that instrumentation works. They are not a performance "
            "qualification result.\n";
  return report.str();
}
}  // namespace

int main(int argc, char** argv)
{
  std::unique_ptr<RunArtifacts> artifacts;
  try
  {
    const Options options = parse_options(argc, argv);
    const fs::path definition_path =
      std::string(motion_control_lab::e01::build_config::kDefinitionPath);
    const fs::path model_path = std::string(motion_control_lab::e01::build_config::kModelPath);

    const std::string definition_sha256 = motion_control_lab::sha256_file(definition_path);
    const std::string model_sha256 = motion_control_lab::sha256_file(model_path);
    if (definition_sha256 != motion_control_lab::e01::build_config::kDefinitionSha256 ||
        model_sha256 != motion_control_lab::e01::build_config::kModelSha256)
    {
      throw std::runtime_error(
        "E01 definition or model changed after configuration; rerun CMake before executing");
    }

    const Json::Value definition = motion_control_lab::load_json_file(definition_path);
    validate_definition(definition);

    const std::string run_id =
      options.run_id.value_or(motion_control_lab::make_run_id(definition_sha256));
    RunMetadata metadata;
    metadata.run_id = run_id;
    metadata.experiment_id = definition["experiment_id"].asString();
    metadata.arm_id = definition["arms"][0]["arm_id"].asString();
    metadata.input_id = definition["inputs"][0]["input_id"].asString();
    metadata.definition_locator = "experiments/E01_placo_smoke/definition.json";
    metadata.definition_sha256 = definition_sha256;
    metadata.resolved_definition = definition;
    metadata.input_locator = "experiments/E01_placo_smoke/model/two_link.urdf";
    metadata.input_sha256 = model_sha256;
    metadata.source_revision =
      std::string(motion_control_lab::e01::build_config::kSourceRevision);
    metadata.source_dirty = motion_control_lab::e01::build_config::kSourceDirty;
    metadata.placo_revision =
      std::string(motion_control_lab::e01::build_config::kPlacoRevision);
    metadata.dependencies["placo"] = metadata.placo_revision;
    metadata.runtime = std::string(motion_control_lab::e01::build_config::kRuntime);

    artifacts = std::make_unique<RunArtifacts>(options.output_root, std::move(metadata));
    artifacts->write_text("definition/resolved.json", motion_control_lab::json_to_string(definition));
    artifacts->write_text(
      "inputs/synthetic_two_link/canonical_copy.urdf",
      read_text_file(model_path));

    Json::Value input_metadata(Json::objectValue);
    input_metadata["schema_version"] = "input_metadata.v1";
    input_metadata["input_id"] = "synthetic_two_link";
    input_metadata["source_locator"] = "experiments/E01_placo_smoke/model/two_link.urdf";
    input_metadata["sha256"] = model_sha256;
    input_metadata["kind"] = "synthetic_smoke_fixture";
    artifacts->write_text(
      "inputs/synthetic_two_link/metadata.json",
      motion_control_lab::json_to_string(input_metadata));

    const SolveResult result = run_solver(definition, model_path);
    const double tolerance_m =
      definition["controlled_factors"]["position_tolerance_m"].asDouble();

    artifacts->write_text(
      "arms/placo_v0_9_23/synthetic_two_link/trace.csv",
      result.trace_csv);
    artifacts->write_text(
      "arms/placo_v0_9_23/synthetic_two_link/status.json",
      motion_control_lab::json_to_string(make_arm_status(result, tolerance_m)));
    artifacts->write_text("evaluation/metrics.csv", make_metrics_csv(run_id, result));
    artifacts->write_text("evaluation/report.md", make_report(result, tolerance_m));

    if (!result.converged)
    {
      const std::string message = "Final position error exceeds the declared tolerance";
      artifacts->finalize_failed(message);
      std::cerr << "E01 failed: " << message << "\n"
                << "Run: " << artifacts->run_directory() << '\n';
      return 2;
    }
    if (result.joint_limit_violation_count != 0)
    {
      const std::string message = "Joint-limit guardrail failed";
      artifacts->finalize_failed(message);
      std::cerr << "E01 failed: " << message << "\n"
                << "Run: " << artifacts->run_directory() << '\n';
      return 2;
    }

    artifacts->finalize_completed();
    std::cout << "E01 completed\n"
              << "Run: " << artifacts->run_directory() << '\n'
              << "Final position error: " << std::setprecision(8) << result.final_error_m << " m\n";
    return 0;
  }
  catch (const std::exception& error)
  {
    if (artifacts)
    {
      try
      {
        Json::Value failure(Json::objectValue);
        failure["schema_version"] = "failure.v1";
        failure["message"] = error.what();
        artifacts->write_text(
          "evaluation/failure.json",
          motion_control_lab::json_to_string(failure));
        artifacts->finalize_failed(error.what());
      }
      catch (const std::exception&)
      {
      }
    }
    std::cerr << "E01 infrastructure failure: " << error.what() << '\n';
    return 1;
  }
}
