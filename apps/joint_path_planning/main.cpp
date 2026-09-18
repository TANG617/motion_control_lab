#include "loop.hpp"
#include "motion_control_lab/sha256.hpp"
#include <iostream>
#include <fstream>
#include <motion_control_viz/fanout_render_sink.hpp>
#include <motion_control_viz/foxglove_mcap_sink.hpp>
#include <motion_control_viz/foxglove_websocket_sink.hpp>
#include <motion_control_viz/null_render_sink.hpp>
namespace app = motion_control_lab::joint_path_planning;
int main(int argc, char **argv) {
  // Fatal boundary only: unwind terminal/worker RAII, expose the original
  // error, exit.
  std::filesystem::path fatal_log;
  try {
    auto o = app::parseOptions(argc, argv);
    if (o.help) {
      std::cout
          << "mcl_joint_path_planning --config FILE --urdf FILE --output "
             "NEW_DIR\n"
             "[--planning-mode whole-body|single-arm] [--side left|right] "
             "[--goal JSON] [--headless] [--realtime] "
             "[--no-viz] [--no-record]\n"
             "[--host HOST] [--port PORT] [--budget SECONDS] "
             "[--simplification-budget SECONDS] [--seed INTEGER]\n"
             "[--timing-mode straight-through|stop-at-waypoints|smooth]\n"
             "[--smooth-validation full|none]\n"
             "[--smoothing-budget SECONDS] [--smoothing-revolute-deviation "
             "RAD]\n"
             "--describe-capabilities | --request FILE "
             "[--dump-resolved-options]\n"
             "Keyboard: w/s a/d q/e translate, arrows arm/step, n i/u rotate, "
             "m step, r reset target, p demo target\n"
             "Enter plan+execute, Space pause, c cancel plan, 1-7 pages, ? "
             "help, x/Esc exit\n";
      return 0;
    }
    if (o.describe) {
      std::cout
          << R"({"app_id":"mcl_joint_path_planning","execution_structure":"target-ik-joint-path-timing-kinematic-execution","sources":["keyboard","request"],"robot":"R1","hardware":false,"planning_modes":["whole-body","single-arm"],"default_planning_mode":"whole-body","timing_modes":["stop-at-waypoints","straight-through","smooth"],"default_timing_mode":"straight-through","smooth_validation_modes":["full","none"],"default_smooth_validation":"full","request_schema":"joint_path_planning.request.v1"})"
          << '\n';
      return 0;
    }
    if (o.dump) {
      std::cout << app::resolved(o) << '\n';
      return 0;
    }
    if (!std::filesystem::create_directories(o.output))
      throw std::runtime_error("output directory already exists: " +
                               o.output.string());
    fatal_log = o.output / "native.log";
    auto config = app::resolved(o);
    config["urdf_sha256"] = motion_control_lab::sha256_file(o.urdf);
    config["config_sha256"] = motion_control_lab::sha256_file(o.config);
    app::writeJson(o.output / "resolved.json", config);
    app::mcc::RobotModelDescription description;
    description.urdf_path = o.urdf.string();
    description.kinematics_reference_frame =
        motion_control_lab::r1RobotConfig().base_frame;
    std::shared_ptr<const app::mcc::RobotModel> model;
    app::require(app::mcc::RobotModel::load(description, model));
    app::Solver solver(model, app::initialState(*model, o),
                       o.planning_mode == "whole-body");
    app::Planning planning(model, o);
    namespace v = motion_control::viz;
    std::vector<std::unique_ptr<v::RenderSink>> outputs;
    if (o.viz)
      outputs.push_back(std::make_unique<v::FoxgloveWebSocketSink>(
          v::FoxgloveWebSocketSinkOptions{
              "mcl_joint_path_planning", o.host,
              static_cast<std::uint16_t>(o.port),
              [refresh = o.scene_refresh] { ++*refresh; }}));
    if (o.record)
      outputs.push_back(std::make_unique<v::FoxgloveMcapSink>(
          v::FoxgloveMcapSinkOptions{o.output / "visualization.mcap"}));
    if (outputs.empty())
      outputs.push_back(std::make_unique<v::NullRenderSink>());
    v::FanoutRenderSink sink(std::move(outputs));
    return app::run(o, model, solver, planning, sink);
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    // SessionLog has restored stderr during unwinding; retain the same fatal error in the run.
    if (!fatal_log.empty())
      std::ofstream(fatal_log, std::ios::app) << e.what() << '\n';
    return 1;
  }
}
