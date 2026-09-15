#include "loop.hpp"
#include <chrono>
#include <iostream>
int main(int argc, char **argv) {
  namespace app = motion_control_lab::planned_kinematics_step;
  namespace execution = motion_control_lab::execution;
  if (argc == 2 && std::string(argv[1]) == "--describe-capabilities") {
    std::cout << app::capabilities() << '\n';
    return 0;
  }
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "mcl_planned_kinematics_step --request FILE "
                 "[--dump-resolved-options]\n"
              << "mcl_planned_kinematics_step --input FILE --input-format "
                 "json|csv|mcap --config FILE --output-dir DIR "
                 "[--dump-resolved-options]\n";
    return 0;
  }
  const auto start = std::chrono::steady_clock::now();
  auto elapsed = [&]() {
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now() - start)
        .count();
  };
  auto options = app::parse(argc, argv);
  const double options_ms = elapsed();
  if (options.dump_only) {
    std::cout << options.config << '\n';
    return 0;
  }
  execution::beginOutput(options.request, options.config, app::capabilities());
  app::write(options.output / "argv.json", options.original_argv);
  const double load_start = elapsed();
  app::prepareInput(options);
  const double input_load_ms = elapsed() - load_start;
  app::write(options.output / "resolved_input.json", options.input);
  const double solver_start = elapsed();
  app::Solver solver(options);
  const double solver_ms = elapsed() - solver_start;
  const double planner_start = elapsed();
  app::Planning planning(options);
  Json::Value timing;
  timing["request_parse_ms"] = options_ms;
  timing["input_load_ms"] = input_load_ms;
  timing["solver_initialization_ms"] = solver_ms;
  timing["planning_initialization_ms"] = elapsed() - planner_start;
  timing["scope"] = "initialization excluded from per-attempt ik_solve_call_ns";
  app::write(options.output / "initialization_timing.json", timing);
  return app::loop(options, solver, planning);
}
