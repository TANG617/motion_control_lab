#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include <motion_control_core/motion_control_core.hpp>

#include "execution.hpp"
#include "loop.hpp"
#include "options.hpp"
#include "planning.hpp"
#include "solver.hpp"
#include <optional>

namespace {

namespace app = motion_control_lab::hierarchical_kinematics_step;
namespace mcc = motion_control::core;

constexpr const char *kProgramId = "mcl_hierarchical_kinematics_step";

int run(int argc, char **argv, std::string &normal_exit_detail) {
  for (int i = 1; i < argc; ++i) {
    const std::string a(argv[i]);
    if (a.rfind("--pim-", 0) == 0 || a == "--elbow-socket")
      throw std::runtime_error(
          "PiM Python/socket inference was removed; use --elbow-reference harp "
          "--harp-model <installed model>");
  }
  if (argc == 2 && std::string(argv[1]) == "--describe-capabilities") {
    std::cout << app::executionCapabilities() << '\n';
    return 0;
  }
  std::optional<motion_control_lab::execution::Request> request;
  std::string batch_input, batch_output;
  Json::Value batch_settings;
  std::vector<char *> filtered;
  for (int i = 0; i < argc; ++i) {
    const std::string argument(argv[i]);
    if (argument == "--batch-input" || argument == "--batch-output" ||
        argument == "--batch-settings") {
      if (++i >= argc)
        throw std::runtime_error("missing batch argument");
      if (argument == "--batch-input")
        batch_input = argv[i];
      else if (argument == "--batch-output")
        batch_output = argv[i];
      else
        batch_settings = motion_control_lab::execution::readJson(argv[i]);
    } else
      filtered.push_back(argv[i]);
  }
  const bool request_mode =
      motion_control_lab::execution::isRequest(argc, argv);
  if (request_mode)
    motion_control_lab::execution::requestDumpOnly(argc, argv);
  if (request_mode)
    request = motion_control_lab::execution::readRequest(argv[2], kProgramId);
  auto options = request_mode
                     ? app::requestOptions(*request)
                     : app::parseOptions(filtered.size(), filtered.data());
  if (request_mode)
    options.dump_resolved_options =
        motion_control_lab::execution::requestDumpOnly(argc, argv);
  if (request && !batch_input.empty())
    throw std::runtime_error("request forbids batch CLI overrides");
  if (options.dump_resolved_options) {
    std::cout << app::resolvedOptionsJson(options);
    return EXIT_SUCCESS;
  }
  if (request) {
    Json::CharReaderBuilder reader;
    std::istringstream encoded(app::resolvedOptionsJson(options));
    Json::Value resolved;
    std::string errors;
    if (!Json::parseFromStream(reader, encoded, &resolved, &errors))
      throw std::runtime_error(errors);
    motion_control_lab::execution::beginOutput(*request, resolved,
                                               app::executionCapabilities());
    app::prepareRequestReplay(*request, options);
  } else if (!batch_input.empty()) {
    if (batch_output.empty() || std::filesystem::exists(batch_output))
      throw std::runtime_error("batch output must be new");
    std::filesystem::create_directories(batch_output);
    std::ofstream resolved(std::filesystem::path(batch_output) /
                           "resolved_options.json");
    resolved << app::resolvedOptionsJson(options);
  }
  // runLoop takes ownership of Options; retain a stable value while handing
  // the same resolved RobotOptions through the composition root.
  const auto robot = options.interactive.robot;

  const auto model = app::loadRobotModel(robot, options);
  const auto collision_model = app::loadCollisionModel(model, options);
  const auto joint_limits =
      app::makeJointTargetLimits(robot, options.interactive.robot.joint_stream);
  const auto active_joint_names =
      app::activeJointNames(robot, options.interactive.robot);
  const auto active_joint_full_indices =
      app::activeJointFullIndices(robot, options.interactive.robot);

  app::SolverHandles handles;
  app::SolverRuntime runtime;
  app::configureSolver(runtime, handles, model, active_joint_names,
                       collision_model, robot, options);
  app::enableNativeJournal(runtime, options.raw_journal_path,
                           options.observation_mode);
  if (request || !batch_input.empty())
    app::writeModelMapping(options, model,
                           request ? request->output
                                   : std::filesystem::path(batch_output));
  if (request && !options.replay) {
    auto settings = request->document["execution"];
    settings["observation"] = request->document["observation"];
    settings["structure"] = request->document["execution_structure"];
    return app::runBatch(options, request->input, settings, request->output,
                         runtime, handles);
  }
  if (!batch_input.empty()) {
    if (options.profile != app::Profile::Hierarchical)
      throw std::runtime_error("batch driver requires hierarchical profile");
    return app::runBatch(options,
                         motion_control_lab::execution::readJson(batch_input),
                         batch_settings, batch_output, runtime, handles);
  }
  const auto capabilities = app::profileCapabilities(options);
  std::unique_ptr<mcc::CartesianPlanner> cartesian_planner;
  std::unique_ptr<mcc::JointPlanner> joint_planner;
  if (capabilities.cartesian_planning) {
    cartesian_planner = std::make_unique<mcc::CartesianPlanner>();
  }
  if (capabilities.joint_otg) {
    joint_planner = std::make_unique<mcc::JointPlanner>(
        app::makeJointPlannerConfig(options.planning));
  }

  std::unique_ptr<app::CenterOfMassVisualization> com_visualization;
  if (options.interactive.show_com && options.interactive.visualization.enabled) {
    com_visualization = app::makeCenterOfMassVisualization(model, robot);
  }

  const int result =
      app::runLoop(std::move(options), robot, runtime, handles,
                   cartesian_planner.get(), joint_planner.get(), com_visualization.get(), joint_limits,
                   active_joint_full_indices, normal_exit_detail);
  if (request)
    app::writeReplaySummary(request->output, result);
  return result;
}

} // namespace

int main(int argc, char **argv) {
  try {
    std::string normal_exit_detail;
    const int exit_code = run(argc, argv, normal_exit_detail);
    std::cerr << kProgramId << ": exited normally";
    if (!normal_exit_detail.empty()) {
      std::cerr << ' ' << normal_exit_detail;
    }
    std::cerr << '\n';
    return exit_code;
  } catch (const std::exception &error) {
    std::cerr << kProgramId << ": " << error.what() << '\n';
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << kProgramId << ": non-standard exception\n";
    return EXIT_FAILURE;
  }
}
