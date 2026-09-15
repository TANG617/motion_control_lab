#pragma once
#include "options.hpp"
#include "solver.hpp"
#include <motion_control_lab/execution_request.hpp>
namespace motion_control_lab::hierarchical_kinematics_step {
Json::Value executionCapabilities();
Json::Value replayReleasePlan(const Options &, std::size_t);
Json::Value replayReleaseCounts(
    const Options &, const motion_control_lab::PeriodicWorkerStatistics &,
    const motion_control_lab::PeriodicWorkerStatistics &, std::size_t,
    std::size_t, std::size_t, std::size_t, bool, bool, bool);

void writeModelMapping(const Options &,
                       const std::shared_ptr<const mcc::RobotModel> &,
                       const std::filesystem::path &);
void writeReplaySummary(const std::filesystem::path &, int);
void enableNativeJournal(SolverRuntime &runtime,
                         const std::filesystem::path &path,
                         const std::string &mode = "full");
void prepareRequestReplay(const execution::Request &request,
                          const Options &options);
Options requestOptions(const execution::Request &request);
int runBatch(const Options &options, const Json::Value &input,
             const Json::Value &settings, const std::filesystem::path &output,
             SolverRuntime &runtime, const SolverHandles &handles);
} // namespace motion_control_lab::hierarchical_kinematics_step
