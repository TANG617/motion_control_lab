#pragma once

#include <Eigen/Geometry>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <motion_control_core/motion_control_core.hpp>
#include <motion_control_viz/render_batch.hpp>

#include "options.hpp"
#include "planning.hpp"
#include "solver.hpp"

namespace motion_control_lab::hierarchical_kinematics_step {

struct Link4TargetSnapshot;

// App-local visualization state; all mutable caches belong to the UI thread.
struct CenterOfMassVisualization {
  motion_control::core::CenterOfMassQuery query;
  SupportVisualizationOptions support;
  std::array<Eigen::Vector3d, 4> support_corners;
  // Only retained when the display reference differs from the support reference.
  std::unique_ptr<motion_control::core::KinematicsSolver> frame_query;
};

std::unique_ptr<CenterOfMassVisualization> makeCenterOfMassVisualization(
    std::shared_ptr<const motion_control::core::RobotModel> model,
    const RobotOptions &robot);

class ReplayPipelineGate {
public:
  void pause() noexcept;
  void resume() noexcept;
  void grantSingleFrame(std::uint64_t red_ticks,
                        std::uint64_t yellow_ticks) noexcept;

  bool acquireRedTick() noexcept;
  bool acquireYellowTick() noexcept;
  bool paused() const noexcept;

private:
  static bool acquireTick(std::atomic<std::uint64_t> &budget) noexcept;

  std::atomic_bool paused_{false};
  std::atomic<std::uint64_t> red_tick_budget_{0U};
  std::atomic<std::uint64_t> yellow_tick_budget_{0U};
};

int runLoop(Options options, const R1RobotConfig &robot, SolverRuntime &solver,
            const SolverHandles &handles,
            motion_control::core::CartesianTrajectoryGenerator *cartesian_generator,
            motion_control::core::JointPtpTrajectoryGenerator *joint_generator,
            CenterOfMassVisualization *com_visualization,
            const JointTargetLimits &joint_limits,
            const std::vector<std::size_t> &active_joint_full_indices,
            std::string &normal_exit_detail);

// Called only on the UI thread, with the committed execution positions.
// A null visualization leaves the batch untouched.
void appendCenterOfMassScene(motion_control::viz::RenderBatch &batch,
                            CenterOfMassVisualization *visualization,
                            const Eigen::VectorXd &executed_positions);

void appendPlanningRequestPoses(motion_control::viz::RenderBatch &frame,
                                const std::string &reference_frame,
                                const Eigen::Isometry3d &left_pose,
                                const Eigen::Isometry3d &right_pose);

void appendOtgExecution(motion_control::viz::RenderBatch &batch,
                        const std::vector<std::string> &joint_names,
                        const std::vector<double> &positions,
                        const std::vector<double> &velocities,
                        const std::string &reference_frame,
                        const Eigen::Isometry3d &left_pose,
                        const Eigen::Isometry3d &right_pose);

void appendNullspaceElbowScene(motion_control::viz::RenderBatch &batch,
                               const std::string &reference_frame,
                               const Link4TargetSnapshot &target,
                               const Eigen::Isometry3d &raw_left_pose,
                               const Eigen::Isometry3d &raw_right_pose,
                               const Eigen::Isometry3d &executed_left_pose,
                               const Eigen::Isometry3d &executed_right_pose);

} // namespace motion_control_lab::hierarchical_kinematics_step
