#include "e04_build_config.hpp"

#include "mcl_opensot_bridge.h"
#include "motion_control_lab/run_artifacts.hpp"
#include "motion_control_lab/sha256.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
namespace fs = std::filesystem;
using motion_control_lab::RunArtifacts;
using motion_control_lab::RunMetadata;

constexpr char kArmId[] = "opensot_v4_0_1_ros2_osqp";
constexpr char kInputId[] = "psi_r1_cos";
constexpr char kTracePath[] =
  "arms/opensot_v4_0_1_ros2_osqp/psi_r1_cos/trace.csv";
constexpr char kStatusPath[] =
  "arms/opensot_v4_0_1_ros2_osqp/psi_r1_cos/status.json";

struct Options
{
  fs::path output_root =
    std::string(motion_control_lab::e04::build_config::kDefaultOutputRoot);
  std::optional<std::string> run_id;
};

void print_usage(const char* executable)
{
  std::cout << "Usage: " << executable
            << " [--output-root PATH] [--run-id ID]\n\n"
               "Runs E04 through the isolated OpenSoT bridge and writes an append-only run bundle.\n";
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
      definition["experiment_id"].asString() != "E04")
  {
    throw std::runtime_error("Definition is not an E04 experiment.v1 declaration");
  }
  if (!definition["controlled_factors"].isObject() || !definition["inputs"].isArray() ||
      definition["inputs"].size() != 1 || !definition["arms"].isArray() ||
      definition["arms"].size() != 1)
  {
    throw std::runtime_error("E04 definition must declare one input and one arm");
  }

  const auto& factors = definition["controlled_factors"];
  if (!factors["controlled_joint_names"].isArray() ||
      factors["controlled_joint_names"].empty() ||
      !factors["initial_joints_rad"].isObject() ||
      !factors["goal_joints_rad"].isObject() ||
      factors["maximum_iterations"].asInt() <= 0 ||
      factors["position_tolerance_m"].asDouble() <= 0.0 ||
      factors["cartesian_lambda"].asDouble() <= 0.0 ||
      factors["postural_lambda"].asDouble() < 0.0 ||
      factors["regularization"].asDouble() <= 0.0 ||
      factors["velocity_limit_period_s"].asDouble() <= 0.0)
  {
    throw std::runtime_error("E04 controlled factors are incomplete or invalid");
  }
  for (const auto& joint : factors["controlled_joint_names"])
  {
    const std::string name = joint.asString();
    if (name.empty() || !factors["initial_joints_rad"].isMember(name) ||
        !factors["goal_joints_rad"].isMember(name))
    {
      throw std::runtime_error("E04 joint maps do not cover every controlled joint");
    }
  }
}

std::vector<std::string> controlled_joint_names(const Json::Value& definition)
{
  std::vector<std::string> names;
  for (const auto& joint : definition["controlled_factors"]["controlled_joint_names"])
  {
    names.push_back(joint.asString());
  }
  return names;
}

class TraceCollector
{
public:
  explicit TraceCollector(std::vector<std::string> joints) : joints_(std::move(joints))
  {
    trace_ << std::setprecision(17);
    trace_ << "iteration,target_x_m,target_y_m,target_z_m,achieved_x_m,achieved_y_m,"
              "achieved_z_m,position_error_m,solve_time_us";
    for (const auto& joint : joints_)
    {
      trace_ << ',' << joint << "_rad";
    }
    trace_ << '\n';
  }

  static void callback(void* user_data, const mcl_opensot_trace_sample* sample) noexcept
  {
    auto& collector = *static_cast<TraceCollector*>(user_data);
    try
    {
      collector.append(sample);
    }
    catch (const std::exception& error)
    {
      collector.callback_error_ = error.what();
    }
    catch (...)
    {
      collector.callback_error_ = "Unknown E04 trace callback failure";
    }
  }

  const std::string& callback_error() const
  {
    return callback_error_;
  }

  bool has_samples() const
  {
    return sample_count_ > 0;
  }

  std::string str() const
  {
    return trace_.str();
  }

private:
  void append(const mcl_opensot_trace_sample* sample)
  {
    if (sample == nullptr || sample->joint_positions == nullptr ||
        sample->joint_count != joints_.size())
    {
      throw std::runtime_error("OpenSoT bridge returned an invalid trace sample");
    }
    trace_ << sample->iteration;
    for (double value : sample->target_position_m)
    {
      trace_ << ',' << value;
    }
    for (double value : sample->achieved_position_m)
    {
      trace_ << ',' << value;
    }
    trace_ << ',' << sample->position_error_m << ',' << sample->solve_time_us;
    for (size_t index = 0; index < sample->joint_count; ++index)
    {
      trace_ << ',' << sample->joint_positions[index];
    }
    trace_ << '\n';
    ++sample_count_;
  }

  std::vector<std::string> joints_;
  std::ostringstream trace_;
  std::string callback_error_;
  size_t sample_count_ = 0;
};

std::string make_metrics_csv(const std::string& run_id, const mcl_opensot_result& result)
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
                       uint32_t sample_count,
                       const std::string& notes) {
    metrics << run_id << ',' << kInputId << ',' << kArmId << ",full_solve," << metric_id
            << ",v1," << value << ',' << unit << ',' << direction << ',' << role
            << ",available," << sample_count << ',' << notes << '\n';
  };
  row("final_position_error_m", result.final_position_error_m, "m", "lower", "primary", 1, "");
  row("iteration_count", result.iterations, "count", "lower", "secondary", 1, "");
  row(
    "joint_limit_violation_count",
    result.joint_limit_violation_count,
    "count",
    "lower",
    "guardrail",
    result.iterations + 1,
    "");
  row("initial_position_error_m", result.initial_position_error_m, "m", "none", "diagnostic", 1, "");
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

Json::Value make_arm_status(const mcl_opensot_result& result, double tolerance_m)
{
  Json::Value status(Json::objectValue);
  status["schema_version"] = "arm_status.v1";
  status["status"] = result.converged != 0 ? "completed" : "failed";
  status["iterations"] = result.iterations;
  status["initial_position_error_m"] = result.initial_position_error_m;
  status["final_position_error_m"] = result.final_position_error_m;
  status["position_tolerance_m"] = tolerance_m;
  status["joint_limit_violation_count"] = result.joint_limit_violation_count;
  return status;
}

std::string make_report(const mcl_opensot_result& result, double tolerance_m)
{
  std::ostringstream report;
  report << std::setprecision(8);
  report << "# E04 OpenSoT ROS2 smoke-test report\n\n"
         << "- Status: " << (result.converged != 0 ? "PASS" : "FAIL") << "\n"
         << "- Initial position error: " << result.initial_position_error_m << " m\n"
         << "- Final position error: " << result.final_position_error_m << " m\n"
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
  std::unique_ptr<TraceCollector> trace;
  bool trace_written = false;
  try
  {
    const Options options = parse_options(argc, argv);
    const fs::path definition_path =
      std::string(motion_control_lab::e04::build_config::kDefinitionPath);
    const fs::path model_path = std::string(motion_control_lab::e04::build_config::kModelPath);

    const std::string definition_sha256 = motion_control_lab::sha256_file(definition_path);
    const std::string model_sha256 = motion_control_lab::sha256_file(model_path);
    if (definition_sha256 != motion_control_lab::e04::build_config::kDefinitionSha256 ||
        model_sha256 != motion_control_lab::e04::build_config::kModelSha256)
    {
      throw std::runtime_error(
        "E04 definition or model changed after configuration; rerun CMake before executing");
    }
    if (std::string(mcl_opensot_upstream_revision()) !=
        motion_control_lab::e04::build_config::kOpenSoTUpstreamRevision)
    {
      throw std::runtime_error("E04 bridge revision does not match the configured OpenSoT source");
    }

    const Json::Value definition = motion_control_lab::load_json_file(definition_path);
    validate_definition(definition);
    const auto& factors = definition["controlled_factors"];
    const std::vector<std::string> joints = controlled_joint_names(definition);
    std::vector<const char*> joint_names;
    std::vector<double> initial_positions;
    std::vector<double> goal_positions;
    joint_names.reserve(joints.size());
    initial_positions.reserve(joints.size());
    goal_positions.reserve(joints.size());
    for (const auto& joint : joints)
    {
      joint_names.push_back(joint.c_str());
      initial_positions.push_back(factors["initial_joints_rad"][joint].asDouble());
      goal_positions.push_back(factors["goal_joints_rad"][joint].asDouble());
    }

    const std::string run_id =
      options.run_id.value_or(motion_control_lab::make_run_id(definition_sha256));
    RunMetadata metadata;
    metadata.run_id = run_id;
    metadata.experiment_id = definition["experiment_id"].asString();
    metadata.arm_id = definition["arms"][0]["arm_id"].asString();
    metadata.input_id = definition["inputs"][0]["input_id"].asString();
    metadata.definition_locator = "experiments/E04_opensot_smoke/definition.json";
    metadata.definition_sha256 = definition_sha256;
    metadata.resolved_definition = definition;
    metadata.input_locator = model_path.string();
    metadata.input_sha256 = model_sha256;
    metadata.source_revision =
      std::string(motion_control_lab::e04::build_config::kSourceRevision);
    metadata.source_dirty = motion_control_lab::e04::build_config::kSourceDirty;
    metadata.dependencies["opensot"] =
      std::string(motion_control_lab::e04::build_config::kOpenSoTRevision);
    metadata.runtime = std::string(motion_control_lab::e04::build_config::kRuntime);

    artifacts = std::make_unique<RunArtifacts>(options.output_root, std::move(metadata));
    artifacts->write_text("definition/resolved.json", motion_control_lab::json_to_string(definition));
    artifacts->write_text("inputs/psi_r1_cos/canonical_copy.urdf", read_text_file(model_path));

    Json::Value input_metadata(Json::objectValue);
    input_metadata["schema_version"] = "input_metadata.v1";
    input_metadata["input_id"] = kInputId;
    input_metadata["source_locator"] = model_path.string();
    input_metadata["sha256"] = model_sha256;
    input_metadata["kind"] = "robot_urdf";
    artifacts->write_text(
      "inputs/psi_r1_cos/metadata.json",
      motion_control_lab::json_to_string(input_metadata));

    const std::string base_frame = factors["base_frame"].asString();
    const std::string end_effector_frame = factors["end_effector_frame"].asString();
    mcl_opensot_request request{};
    request.urdf_path = model_path.c_str();
    request.base_frame = base_frame.c_str();
    request.end_effector_frame = end_effector_frame.c_str();
    request.joint_names = joint_names.data();
    request.initial_positions = initial_positions.data();
    request.goal_positions = goal_positions.data();
    request.joint_count = joints.size();
    request.maximum_iterations = factors["maximum_iterations"].asUInt();
    request.position_tolerance_m = factors["position_tolerance_m"].asDouble();
    request.cartesian_lambda = factors["cartesian_lambda"].asDouble();
    request.postural_lambda = factors["postural_lambda"].asDouble();
    request.regularization = factors["regularization"].asDouble();
    request.velocity_limit_period_s = factors["velocity_limit_period_s"].asDouble();

    trace = std::make_unique<TraceCollector>(joints);
    mcl_opensot_result result{};
    char bridge_error[1024]{};
    const int bridge_status = mcl_opensot_solve_position_ik(
      &request,
      &TraceCollector::callback,
      trace.get(),
      &result,
      bridge_error,
      sizeof(bridge_error));
    if (!trace->callback_error().empty())
    {
      throw std::runtime_error(trace->callback_error());
    }
    if (bridge_status != 0)
    {
      throw std::runtime_error(
        std::string("OpenSoT bridge failed: ") +
        (bridge_error[0] == '\0' ? "unknown error" : bridge_error));
    }

    artifacts->write_text(kTracePath, trace->str());
    trace_written = true;
    artifacts->write_text(
      kStatusPath,
      motion_control_lab::json_to_string(
        make_arm_status(result, request.position_tolerance_m)));
    artifacts->write_text("evaluation/metrics.csv", make_metrics_csv(run_id, result));
    artifacts->write_text(
      "evaluation/report.md",
      make_report(result, request.position_tolerance_m));

    if (result.converged == 0)
    {
      const std::string message = "Final position error exceeds the declared tolerance";
      artifacts->finalize_failed(message);
      std::cerr << "E04 failed: " << message << '\n';
      return 2;
    }
    if (result.joint_limit_violation_count != 0)
    {
      const std::string message = "Joint-limit guardrail failed";
      artifacts->finalize_failed(message);
      std::cerr << "E04 failed: " << message << '\n';
      return 2;
    }

    artifacts->finalize_completed();
    std::cout << "E04 completed\n"
              << "Run: " << artifacts->run_directory() << '\n'
              << "Final position error: " << std::setprecision(8)
              << result.final_position_error_m << " m\n";
    return 0;
  }
  catch (const std::exception& error)
  {
    if (artifacts)
    {
      try
      {
        if (trace && trace->has_samples() && !trace_written)
        {
          artifacts->write_text(kTracePath, trace->str());
        }
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
    std::cerr << "E04 infrastructure failure: " << error.what() << '\n';
    return 1;
  }
}
