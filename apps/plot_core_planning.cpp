#include "matplotlibcpp.h"

#include "motion_control_core/motion_control_core.hpp"

#include <Eigen/Geometry>

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

namespace
{

namespace mcc = motion_control::core;
namespace plt = matplotlibcpp;

mcc::Pose makePose(const Eigen::Vector3d & translation, double yaw)
{
  mcc::Pose pose = mcc::Pose::Identity();
  pose.translation() = translation;
  pose.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return pose;
}

mcc::CartesianMoveLineRequest makeCartesianRequest()
{
  mcc::CartesianMoveLineRequest request;
  request.reference_frame_name = "world";
  request.sample_period = 0.01;
  request.path_limits.max_velocity = 0.2;
  request.path_limits.max_acceleration = 0.6;
  request.path_limits.max_jerk = 2.0;

  request.segments.push_back(
    mcc::CartesianMoveLineSegment{
      "left_tcp",
      makePose(Eigen::Vector3d{0.40, 0.20, 0.80}, 0.0),
      mcc::Twist::Zero(),
      mcc::SpatialAcceleration::Zero(),
      makePose(Eigen::Vector3d{0.55, 0.28, 0.86}, 0.45),
      0.5});
  request.segments.push_back(
    mcc::CartesianMoveLineSegment{
      "right_tcp",
      makePose(Eigen::Vector3d{0.40, -0.20, 0.80}, 0.0),
      mcc::Twist::Zero(),
      mcc::SpatialAcceleration::Zero(),
      makePose(Eigen::Vector3d{0.50, -0.30, 0.82}, -0.30),
      0.5});

  return request;
}

mcc::JointTrajectoryRequest makeJointRequest()
{
  mcc::JointTrajectoryRequest request;
  request.joint_names = {
    "left_arm_joint1",
    "left_arm_joint2",
    "left_arm_joint3",
    "left_arm_joint4",
    "left_arm_joint5",
    "left_arm_joint6",
    "left_arm_joint7"};

  request.current.positions = {0.90, -1.38, -1.57, -1.40, -0.45, 0.00, 0.00};
  request.current.velocities.assign(request.joint_names.size(), 0.0);
  request.current.accelerations.assign(request.joint_names.size(), 0.0);

  request.target.positions = {0.75, -1.10, -1.35, -1.15, -0.30, 0.20, -0.10};
  request.target.velocities.assign(request.joint_names.size(), 0.0);
  request.target.accelerations.assign(request.joint_names.size(), 0.0);

  request.limits.position_lower.assign(request.joint_names.size(), -3.14);
  request.limits.position_upper.assign(request.joint_names.size(), 3.14);
  request.limits.max_velocity.assign(request.joint_names.size(), 1.0);
  request.limits.max_acceleration.assign(request.joint_names.size(), 2.0);
  request.limits.max_jerk.assign(request.joint_names.size(), 8.0);
  request.sample_period = 0.01;
  return request;
}

void requireOk(const mcc::Status & status, const std::string & action)
{
  if (!status.ok()) {
    throw std::runtime_error(action + " failed: " + status.message);
  }
}

mcc::CartesianTrajectory sampleCartesianTrajectory()
{
  mcc::CartesianMoveLinePlanner planner;
  mcc::CartesianTrajectory trajectory;
  mcc::PlanningDiagnostics diagnostics;
  requireOk(
    planner.generate(makeCartesianRequest(), trajectory, diagnostics),
    "Cartesian planning");
  return trajectory;
}

mcc::JointTrajectory sampleJointTrajectory()
{
  mcc::JointTrajectoryPlannerConfig config;
  config.algorithm = mcc::JointTrajectoryAlgorithm::JerkLimited;
  config.synchronization = mcc::TrajectorySynchronization::Phase;

  mcc::JointTrajectoryPlanner planner(config);
  mcc::JointTrajectory trajectory;
  mcc::PlanningDiagnostics diagnostics;
  requireOk(
    planner.generate(makeJointRequest(), trajectory, diagnostics),
    "Joint trajectory planning");
  return trajectory;
}

void saveCartesianPlot(
  const mcc::CartesianTrajectory & trajectory,
  const std::filesystem::path & path)
{
  std::map<std::string, std::vector<double>> x_by_frame;
  std::map<std::string, std::vector<double>> y_by_frame;

  for (const auto & sample : trajectory.samples) {
    for (const auto & frame : sample.frames) {
      x_by_frame[frame.frame_name].push_back(frame.pose.translation().x());
      y_by_frame[frame.frame_name].push_back(frame.pose.translation().y());
    }
  }

  plt::figure();
  plt::figure_size(1000, 700);
  for (const auto & entry : x_by_frame) {
    const auto y = y_by_frame.find(entry.first);
    if (y != y_by_frame.end()) {
      plt::named_plot(entry.first, entry.second, y->second);
    }
  }
  plt::title("Cartesian move-line XY path");
  plt::xlabel("x [m]");
  plt::ylabel("y [m]");
  plt::axis("equal");
  plt::grid(true);
  plt::legend();
  plt::tight_layout();
  plt::save(path.string());
  plt::close();
}

void saveJointPlot(
  const mcc::JointTrajectory & trajectory,
  const std::filesystem::path & path)
{
  if (trajectory.samples.empty()) {
    throw std::runtime_error("joint trajectory has no samples");
  }

  std::vector<double> time;
  std::vector<std::vector<double>> positions(trajectory.joint_names.size());
  for (const auto & sample : trajectory.samples) {
    time.push_back(sample.time_from_start);
    for (std::size_t index = 0; index < sample.positions.size(); ++index) {
      if (index < positions.size()) {
        positions[index].push_back(sample.positions[index]);
      }
    }
  }

  plt::figure();
  plt::figure_size(1200, 700);
  for (std::size_t index = 0; index < trajectory.joint_names.size(); ++index) {
    plt::named_plot(trajectory.joint_names[index], time, positions[index]);
  }
  plt::title("Joint trajectory positions");
  plt::xlabel("time from start [s]");
  plt::ylabel("position [rad]");
  plt::grid(true);
  plt::legend();
  plt::tight_layout();
  plt::save(path.string());
  plt::close();
}

std::filesystem::path parseOutputDir(int argc, char ** argv)
{
  std::filesystem::path output_dir{"."};
  for (int i = 1; i < argc; ++i) {
    const std::string arg{argv[i]};
    auto requireValue = [&](const std::string & option) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error(option + " requires a value");
      }
      return argv[++i];
    };

    if (arg == "--help" || arg == "-h") {
      std::cout
        << "Usage:\n"
        << "  " << argv[0] << "\n"
        << "  " << argv[0] << " --output-dir <path>\n";
      std::exit(EXIT_SUCCESS);
    } else if (arg == "--output-dir") {
      output_dir = std::filesystem::path{requireValue(arg)};
    } else {
      throw std::runtime_error("unknown option: " + arg);
    }
  }
  return output_dir;
}

void ensureOutputDir(const std::filesystem::path & output_dir)
{
  std::error_code error;
  std::filesystem::create_directories(output_dir, error);
  if (error) {
    throw std::runtime_error(
      "failed to create output directory " + output_dir.string() + ": " +
      error.message());
  }
  if (!std::filesystem::is_directory(output_dir)) {
    throw std::runtime_error("output path is not a directory: " + output_dir.string());
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto output_dir = parseOutputDir(argc, argv);
    ensureOutputDir(output_dir);

    plt::backend("Agg");

    const auto cartesian_path = output_dir / "cartesian_move_line_planning.png";
    const auto joint_path = output_dir / "joint_trajectory_planning.png";

    saveCartesianPlot(sampleCartesianTrajectory(), cartesian_path);
    saveJointPlot(sampleJointTrajectory(), joint_path);

    std::cout << "Wrote " << std::filesystem::absolute(cartesian_path) << '\n';
    std::cout << "Wrote " << std::filesystem::absolute(joint_path) << '\n';
    return EXIT_SUCCESS;
  } catch (const std::exception & error) {
    std::cerr << "motion_control_lab_plot_core_planning: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
