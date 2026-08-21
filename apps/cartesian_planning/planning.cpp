#include "planning.hpp"

#include "matplotlibcpp.h"

#include <json/json.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace motion_control_lab::cartesian_planning {
namespace {

namespace mcc = motion_control::core;
namespace plt = matplotlibcpp;

constexpr const char *kRequestSchema =
    "motion_control_lab.cartesian_move_line_request.v1";
Eigen::Vector3d parseVector3(const Json::Value &value, const std::string &) {
  return Eigen::Vector3d{value[0].asDouble(), value[1].asDouble(),
                         value[2].asDouble()};
}

mcc::Pose parsePose(const Json::Value &value, const std::string &context) {
  const Eigen::Vector3d position =
      parseVector3(value["position_m"], context + ".position_m");
  const auto &orientation = value["orientation_xyzw"];
  const std::array<double, 4> xyzw{
      orientation[0].asDouble(), orientation[1].asDouble(),
      orientation[2].asDouble(), orientation[3].asDouble()};
  const Eigen::Quaterniond quaternion(xyzw[3], xyzw[0], xyzw[1], xyzw[2]);

  mcc::Pose pose = mcc::Pose::Identity();
  pose.translation() = position;
  pose.linear() = quaternion.toRotationMatrix();
  return pose;
}

mcc::Twist parseTwist(const Json::Value &value, const std::string &context) {
  mcc::Twist result = mcc::Twist::Zero();
  result.head<3>() = parseVector3(value["linear_mps"], context + ".linear_mps");
  result.tail<3>() =
      parseVector3(value["angular_rps"], context + ".angular_rps");
  return result;
}

mcc::SpatialAcceleration parseAcceleration(const Json::Value &value,
                                           const std::string &context) {
  mcc::SpatialAcceleration result = mcc::SpatialAcceleration::Zero();
  result.head<3>() =
      parseVector3(value["linear_mps2"], context + ".linear_mps2");
  result.tail<3>() =
      parseVector3(value["angular_rps2"], context + ".angular_rps2");
  return result;
}

Json::Value loadJson(const std::filesystem::path &path) {
  std::ifstream stream;
  stream.exceptions(std::ios::failbit | std::ios::badbit);
  stream.open(path);
  Json::CharReaderBuilder builder;
  builder["collectComments"] = false;
  builder["allowComments"] = false;
  builder["allowTrailingCommas"] = false;
  builder["strictRoot"] = true;
  builder["failIfExtra"] = true;
  builder["rejectDupKeys"] = true;
  builder["allowSpecialFloats"] = false;
  Json::Value root;
  std::string errors;
  if (!Json::parseFromStream(builder, stream, &root, &errors)) {
    throw std::runtime_error("failed to parse request JSON: " + errors);
  }
  return root;
}

const mcc::CartesianFrameSample &
requireFrame(const mcc::CartesianTrajectorySample &sample,
             const std::string &frame_name) {
  return *std::find_if(sample.frames.begin(), sample.frames.end(),
                       [&](const mcc::CartesianFrameSample &frame) {
                         return frame.frame_name == frame_name;
                       });
}

std::vector<double> sampleTimes(const mcc::CartesianTrajectory &trajectory) {
  std::vector<double> values;
  values.reserve(trajectory.samples.size());
  for (const auto &sample : trajectory.samples) {
    values.push_back(sample.time_from_start);
  }
  return values;
}

std::string csvCell(const std::string &value) {
  if (value.find_first_of(",\"\r\n") == std::string::npos) {
    return value;
  }
  std::string escaped{"\""};
  for (const char character : value) {
    if (character == '\"') {
      escaped += '\"';
    }
    escaped += character;
  }
  escaped += '\"';
  return escaped;
}

} // namespace

mcc::CartesianLineRequest loadRequest(const std::filesystem::path &path) {
  const Json::Value root = loadJson(path);
  if (root["schema_version"].asString() != kRequestSchema) {
    throw std::runtime_error("request.schema_version is unsupported");
  }

  mcc::CartesianLineRequest request;
  request.reference_frame_name = root["reference_frame_name"].asString();
  request.sample_period = root["sample_period_s"].asDouble();

  if (root.isMember("maximum_sample_count")) {
    request.maximum_sample_count =
        static_cast<std::size_t>(root["maximum_sample_count"].asUInt64());
  }

  if (root.isMember("synchronization")) {
    const std::string synchronization = root["synchronization"].asString();
    if (synchronization == "time") {
      request.synchronization = mcc::TrajectorySynchronization::Time;
    } else if (synchronization == "phase") {
      request.synchronization = mcc::TrajectorySynchronization::Phase;
    } else {
      throw std::runtime_error(
          "request.synchronization must be 'time' or 'phase'");
    }
  }

  const auto &limits = root["path_limits"];
  request.path_limits.max_velocity = limits["max_velocity_mps"].asDouble();
  request.path_limits.max_acceleration =
      limits["max_acceleration_mps2"].asDouble();
  request.path_limits.max_jerk = limits["max_jerk_mps3"].asDouble();

  const auto &segments = root["segments"];
  request.segments.reserve(segments.size());
  for (Json::ArrayIndex index = 0; index < segments.size(); ++index) {
    const auto &value = segments[index];
    const std::string context =
        "request.segments[" + std::to_string(index) + "]";
    mcc::CartesianLineSegment segment;
    segment.frame_name = value["frame_name"].asString();
    segment.start_pose =
        parsePose(value["start_pose"], context + ".start_pose");
    segment.target_pose =
        parsePose(value["target_pose"], context + ".target_pose");
    if (value.isMember("current_twist")) {
      segment.current_twist =
          parseTwist(value["current_twist"], context + ".current_twist");
    }
    if (value.isMember("current_acceleration")) {
      segment.current_acceleration = parseAcceleration(
          value["current_acceleration"], context + ".current_acceleration");
    }
    if (value.isMember("equivalent_radius_m")) {
      segment.equivalent_radius_m = value["equivalent_radius_m"].asDouble();
    }
    request.segments.push_back(std::move(segment));
  }
  return request;
}

OutputPaths prepareOutputPaths(const std::filesystem::path &output_dir,
                               bool force) {
  OutputPaths paths{output_dir / "trajectory.csv",
                    output_dir / "cartesian_path_3d.png",
                    output_dir / "cartesian_profiles.png"};
  for (const auto &path :
       {paths.trajectory_csv, paths.path_plot, paths.profiles_plot}) {
    if (!force && std::filesystem::exists(path)) {
      throw std::runtime_error("refusing to overwrite output: " +
                               path.string());
    }
  }
  std::filesystem::create_directories(output_dir);
  return paths;
}

void writeTrajectoryCsv(const std::filesystem::path &path,
                        const mcc::CartesianTrajectory &trajectory) {
  std::ofstream output;
  output.exceptions(std::ios::failbit | std::ios::badbit);
  output.open(path, std::ios::trunc);
  output << std::setprecision(17)
         << "time_from_start,reference_frame_name,frame_name,"
         << "position_x,position_y,position_z,"
         << "orientation_x,orientation_y,orientation_z,orientation_w,"
         << "linear_velocity_x,linear_velocity_y,linear_velocity_z,"
         << "angular_velocity_x,angular_velocity_y,angular_velocity_z,"
         << "linear_acceleration_x,linear_acceleration_y,linear_acceleration_z,"
         << "angular_acceleration_x,angular_acceleration_y,angular_"
            "acceleration_z\n";
  for (const auto &sample : trajectory.samples) {
    for (const auto &frame : sample.frames) {
      Eigen::Quaterniond orientation(frame.pose.linear());
      orientation.normalize();
      output << sample.time_from_start << ','
             << csvCell(frame.reference_frame_name) << ','
             << csvCell(frame.frame_name) << ',' << frame.pose.translation().x()
             << ',' << frame.pose.translation().y() << ','
             << frame.pose.translation().z() << ',' << orientation.x() << ','
             << orientation.y() << ',' << orientation.z() << ','
             << orientation.w() << ',';
      for (Eigen::Index index = 0; index < 6; ++index) {
        output << frame.twist(index) << ',';
      }
      for (Eigen::Index index = 0; index < 6; ++index) {
        output << frame.acceleration(index);
        output << (index == 5 ? '\n' : ',');
      }
    }
  }
}

void renderTrajectoryPlots(const OutputPaths &paths,
                           const mcc::CartesianTrajectory &trajectory) {
  plt::backend("Agg");

  const long path_figure = plt::figure();
  Eigen::Vector3d minimum =
      trajectory.samples.front().frames.front().pose.translation();
  Eigen::Vector3d maximum = minimum;
  for (std::size_t frame_index = 0;
       frame_index < trajectory.samples.front().frames.size(); ++frame_index) {
    const auto &first_frame = trajectory.samples.front().frames[frame_index];
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
    x.reserve(trajectory.samples.size());
    y.reserve(trajectory.samples.size());
    z.reserve(trajectory.samples.size());
    for (const auto &sample : trajectory.samples) {
      const auto &frame = requireFrame(sample, first_frame.frame_name);
      minimum = minimum.cwiseMin(frame.pose.translation());
      maximum = maximum.cwiseMax(frame.pose.translation());
      x.push_back(frame.pose.translation().x());
      y.push_back(frame.pose.translation().y());
      z.push_back(frame.pose.translation().z());
    }
    const std::string color = "C" + std::to_string(frame_index % 10);
    plt::scatter(x, y, z, 4.0,
                 {{"label", first_frame.frame_name}, {"color", color}},
                 path_figure);
    plt::scatter(std::vector<double>{x.front()}, std::vector<double>{y.front()},
                 std::vector<double>{z.front()}, 64.0,
                 {{"marker", "o"}, {"color", color}}, path_figure);
    plt::scatter(std::vector<double>{x.back()}, std::vector<double>{y.back()},
                 std::vector<double>{z.back()}, 81.0,
                 {{"marker", "x"}, {"color", color}}, path_figure);
  }
  Eigen::Vector3d margin = 0.05 * (maximum - minimum);
  for (Eigen::Index index = 0; index < 3; ++index) {
    margin(index) = std::max(margin(index), 0.01);
  }
  const Eigen::Vector3d lower = minimum - margin;
  const Eigen::Vector3d upper = maximum + margin;
  plt::plot3(std::vector<double>{lower.x(), upper.x()},
             std::vector<double>{lower.y(), upper.y()},
             std::vector<double>{lower.z(), upper.z()},
             {{"color", "none"}, {"label", "_bounds"}}, path_figure);
  plt::title("Cartesian MoveLine paths (o=start, x=target)");
  plt::xlabel("x [m]");
  plt::ylabel("y [m]");
  plt::set_zlabel("z [m]");
  plt::legend();
  plt::tight_layout();
  plt::save(paths.path_plot.string());
  plt::close();

  const auto time = sampleTimes(trajectory);
  plt::figure();
  plt::figure_size(1400, 1100);
  for (int component = 0; component < 3; ++component) {
    plt::subplot(3, 2, component + 1);
    for (const auto &first_frame : trajectory.samples.front().frames) {
      std::vector<double> values;
      values.reserve(trajectory.samples.size());
      for (const auto &sample : trajectory.samples) {
        values.push_back(requireFrame(sample, first_frame.frame_name)
                             .pose.translation()(component));
      }
      plt::named_plot(first_frame.frame_name, time, values);
    }
    plt::title(std::string{"position "} + "xyz"[component] + " [m]");
    plt::grid(true);
    plt::legend();
  }

  plt::subplot(3, 2, 4);
  for (const auto &first_frame : trajectory.samples.front().frames) {
    const Eigen::Quaterniond initial(first_frame.pose.linear());
    std::vector<double> values;
    values.reserve(trajectory.samples.size());
    for (const auto &sample : trajectory.samples) {
      const Eigen::Quaterniond current(
          requireFrame(sample, first_frame.frame_name).pose.linear());
      values.push_back(initial.angularDistance(current));
    }
    plt::named_plot(first_frame.frame_name, time, values);
  }
  plt::title("orientation distance from start [rad]");
  plt::grid(true);
  plt::legend();

  plt::subplot(3, 2, 5);
  for (const auto &first_frame : trajectory.samples.front().frames) {
    std::vector<double> linear;
    std::vector<double> angular;
    for (const auto &sample : trajectory.samples) {
      const auto &frame = requireFrame(sample, first_frame.frame_name);
      linear.push_back(frame.twist.head<3>().norm());
      angular.push_back(frame.twist.tail<3>().norm());
    }
    plt::named_plot(first_frame.frame_name + " linear", time, linear);
    plt::named_plot(first_frame.frame_name + " angular", time, angular);
  }
  plt::title("velocity norms [m/s, rad/s]");
  plt::xlabel("time from start [s]");
  plt::grid(true);
  plt::legend();

  plt::subplot(3, 2, 6);
  for (const auto &first_frame : trajectory.samples.front().frames) {
    std::vector<double> linear;
    std::vector<double> angular;
    for (const auto &sample : trajectory.samples) {
      const auto &frame = requireFrame(sample, first_frame.frame_name);
      linear.push_back(frame.acceleration.head<3>().norm());
      angular.push_back(frame.acceleration.tail<3>().norm());
    }
    plt::named_plot(first_frame.frame_name + " linear", time, linear);
    plt::named_plot(first_frame.frame_name + " angular", time, angular);
  }
  plt::title("acceleration norms [m/s^2, rad/s^2]");
  plt::xlabel("time from start [s]");
  plt::grid(true);
  plt::legend();
  plt::tight_layout();
  plt::save(paths.profiles_plot.string());
  plt::close();
}

} // namespace motion_control_lab::cartesian_planning
