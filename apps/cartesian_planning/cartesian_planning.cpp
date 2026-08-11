#include "cartesian_planning.hpp"

#include "matplotlibcpp.h"

#include <motion_control_viz/foxglove_frame_sink.hpp>

#include <json/json.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

namespace motion_control_lab::cartesian_planning
{
namespace
{

namespace mcc = motion_control::core;
namespace mcv = motion_control::viz;
namespace plt = matplotlibcpp;

constexpr const char * kRequestSchema =
  "motion_control_lab.cartesian_move_line_request.v1";
constexpr const char * kSceneChannel = "/mc/cartesian/scene";
constexpr double kQuaternionNormTolerance = 1.0e-9;
constexpr double kAxisLengthM = 0.05;

std::atomic_bool stop_requested{false};

void signalHandler(int)
{
  stop_requested.store(true);
}

std::string joinNames(const std::set<std::string> & names)
{
  std::ostringstream output;
  bool first = true;
  for (const auto & name : names) {
    if (!first) {
      output << ", ";
    }
    output << name;
    first = false;
  }
  return output.str();
}

void validateMembers(
  const Json::Value & value,
  const std::set<std::string> & required,
  const std::set<std::string> & optional,
  const std::string & context)
{
  if (!value.isObject()) {
    throw std::runtime_error(context + " must be an object");
  }
  std::set<std::string> missing;
  for (const auto & name : required) {
    if (!value.isMember(name)) {
      missing.insert(name);
    }
  }
  if (!missing.empty()) {
    throw std::runtime_error(context + " is missing fields: " + joinNames(missing));
  }

  std::set<std::string> allowed = required;
  allowed.insert(optional.begin(), optional.end());
  std::set<std::string> unknown;
  for (const auto & name : value.getMemberNames()) {
    if (allowed.count(name) == 0) {
      unknown.insert(name);
    }
  }
  if (!unknown.empty()) {
    throw std::runtime_error(context + " has unknown fields: " + joinNames(unknown));
  }
}

std::string requireString(
  const Json::Value & value,
  const std::string & field,
  const std::string & context)
{
  const auto & member = value[field];
  if (!member.isString() || member.asString().empty()) {
    throw std::runtime_error(context + "." + field + " must be a non-empty string");
  }
  return member.asString();
}

double requireFiniteNumber(
  const Json::Value & value,
  const std::string & field,
  const std::string & context)
{
  const auto & member = value[field];
  if (!member.isNumeric()) {
    throw std::runtime_error(context + "." + field + " must be a number");
  }
  const double result = member.asDouble();
  if (!std::isfinite(result)) {
    throw std::runtime_error(context + "." + field + " must be finite");
  }
  return result;
}

double requirePositiveNumber(
  const Json::Value & value,
  const std::string & field,
  const std::string & context)
{
  const double result = requireFiniteNumber(value, field, context);
  if (result <= 0.0) {
    throw std::runtime_error(context + "." + field + " must be positive");
  }
  return result;
}

Eigen::Vector3d parseVector3(
  const Json::Value & value,
  const std::string & context)
{
  if (!value.isArray() || value.size() != 3) {
    throw std::runtime_error(context + " must contain exactly three numbers");
  }
  Eigen::Vector3d result;
  for (Json::ArrayIndex index = 0; index < 3; ++index) {
    if (!value[index].isNumeric() || !std::isfinite(value[index].asDouble())) {
      throw std::runtime_error(context + " must contain only finite numbers");
    }
    result(static_cast<Eigen::Index>(index)) = value[index].asDouble();
  }
  return result;
}

mcc::Pose parsePose(const Json::Value & value, const std::string & context)
{
  validateMembers(value, {"position_m", "orientation_xyzw"}, {}, context);
  const Eigen::Vector3d position = parseVector3(value["position_m"], context + ".position_m");
  const auto & orientation = value["orientation_xyzw"];
  if (!orientation.isArray() || orientation.size() != 4) {
    throw std::runtime_error(context + ".orientation_xyzw must contain exactly four numbers");
  }
  std::array<double, 4> xyzw{};
  for (Json::ArrayIndex index = 0; index < 4; ++index) {
    if (!orientation[index].isNumeric() ||
      !std::isfinite(orientation[index].asDouble()))
    {
      throw std::runtime_error(
              context + ".orientation_xyzw must contain only finite numbers");
    }
    xyzw[index] = orientation[index].asDouble();
  }
  const Eigen::Quaterniond quaternion(xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
  if (std::abs(quaternion.norm() - 1.0) > kQuaternionNormTolerance) {
    throw std::runtime_error(
            context + ".orientation_xyzw must be unit length within 1e-9");
  }

  mcc::Pose pose = mcc::Pose::Identity();
  pose.translation() = position;
  pose.linear() = quaternion.toRotationMatrix();
  return pose;
}

mcc::Twist parseTwist(const Json::Value & value, const std::string & context)
{
  validateMembers(value, {"linear_mps", "angular_rps"}, {}, context);
  mcc::Twist result = mcc::Twist::Zero();
  result.head<3>() = parseVector3(value["linear_mps"], context + ".linear_mps");
  result.tail<3>() = parseVector3(value["angular_rps"], context + ".angular_rps");
  return result;
}

mcc::SpatialAcceleration parseAcceleration(
  const Json::Value & value,
  const std::string & context)
{
  validateMembers(value, {"linear_mps2", "angular_rps2"}, {}, context);
  mcc::SpatialAcceleration result = mcc::SpatialAcceleration::Zero();
  result.head<3>() = parseVector3(value["linear_mps2"], context + ".linear_mps2");
  result.tail<3>() = parseVector3(value["angular_rps2"], context + ".angular_rps2");
  return result;
}

Json::Value loadJson(const std::filesystem::path & path)
{
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("failed to open request JSON: " + path.string());
  }
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

double parsePositiveCliDouble(const std::string & option, const std::string & value)
{
  std::size_t consumed = 0;
  const double result = std::stod(value, &consumed);
  if (consumed != value.size()) {
    throw std::runtime_error(option + " must be a number without trailing characters");
  }
  if (!std::isfinite(result) || result <= 0.0) {
    throw std::runtime_error(option + " must be positive and finite");
  }
  return result;
}

double parseNonNegativeCliDouble(const std::string & option, const std::string & value)
{
  std::size_t consumed = 0;
  const double result = std::stod(value, &consumed);
  if (consumed != value.size()) {
    throw std::runtime_error(option + " must be a number without trailing characters");
  }
  if (!std::isfinite(result) || result < 0.0) {
    throw std::runtime_error(option + " must be finite and non-negative");
  }
  return result;
}

std::array<double, 4> paletteColor(std::size_t index)
{
  constexpr std::array<std::array<double, 4>, 8> palette{{
    {{0.121, 0.466, 0.705, 1.0}},
    {{1.000, 0.498, 0.054, 1.0}},
    {{0.172, 0.627, 0.172, 1.0}},
    {{0.839, 0.153, 0.157, 1.0}},
    {{0.580, 0.404, 0.741, 1.0}},
    {{0.549, 0.337, 0.294, 1.0}},
    {{0.890, 0.467, 0.761, 1.0}},
    {{0.498, 0.498, 0.498, 1.0}}}};
  return palette[index % palette.size()];
}

mcv::ColorRgba toColor(const std::array<double, 4> & value)
{
  return {value[0], value[1], value[2], value[3]};
}

std::array<double, 3> toPoint(const Eigen::Vector3d & value)
{
  return {value.x(), value.y(), value.z()};
}

void appendAxisTriad(
  std::vector<mcv::LineStrip3d> & lines,
  const std::string & entity_prefix,
  const std::string & frame_id,
  const mcc::Pose & pose,
  double alpha)
{
  constexpr std::array<std::array<double, 4>, 3> axis_colors{{
    {{1.0, 0.0, 0.0, 1.0}},
    {{0.0, 1.0, 0.0, 1.0}},
    {{0.0, 0.4, 1.0, 1.0}}}};
  const Eigen::Vector3d origin = pose.translation();
  for (std::size_t axis = 0; axis < 3; ++axis) {
    const Eigen::Vector3d end = origin + kAxisLengthM * pose.linear().col(
      static_cast<Eigen::Index>(axis));
    auto color = axis_colors[axis];
    color[3] = alpha;
    lines.push_back(mcv::LineStrip3d{
      entity_prefix + "/" + std::string{"xyz"[axis]},
      kSceneChannel,
      frame_id,
      {toPoint(origin), toPoint(end)},
      toColor(color),
      3.0,
      true});
  }
}

std::uint64_t wallTimeNs()
{
  return static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count());
}

void waitUntil(const std::chrono::steady_clock::time_point & deadline)
{
  constexpr auto poll_interval = std::chrono::milliseconds(20);
  while (!stop_requested.load()) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      return;
    }
    std::this_thread::sleep_until(std::min(deadline, now + poll_interval));
  }
}

const mcc::CartesianFrameSample & requireFrame(
  const mcc::CartesianTrajectorySample & sample,
  const std::string & frame_name)
{
  for (const auto & frame : sample.frames) {
    if (frame.frame_name == frame_name) {
      return frame;
    }
  }
  throw std::runtime_error("trajectory sample is missing frame: " + frame_name);
}

std::vector<double> sampleTimes(const mcc::CartesianTrajectory & trajectory)
{
  std::vector<double> values;
  values.reserve(trajectory.samples.size());
  for (const auto & sample : trajectory.samples) {
    values.push_back(sample.time_from_start);
  }
  return values;
}

std::string csvCell(const std::string & value)
{
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

}  // namespace

void printUsage(const char * program)
{
  std::cout
    << "Usage: " << program << " --request <json> --output-dir <dir> [options]\n\n"
    << "Options:\n"
    << "  --request <path>       Cartesian MoveLine request JSON (required)\n"
    << "  --output-dir <path>    CSV and PNG output directory (required)\n"
    << "  --host <address>       Foxglove bind address (default: 127.0.0.1)\n"
    << "  --port <port>          Foxglove port (default: 8765)\n"
    << "  --playback-rate <x>    Positive playback speed multiplier (default: 1)\n"
    << "  --loop-delay <sec>     Delay between loops (default: 1)\n"
    << "  --once                 Play once and exit\n"
    << "  --no-live              Generate CSV/PNG without Foxglove playback\n"
    << "  --mcap <path>          Record playback to a new MCAP file\n"
    << "  --force                Overwrite existing CSV/PNG outputs\n"
    << "  --help                 Show this help text\n";
}

AppOptions parseAppOptions(int argc, char ** argv)
{
  AppOptions options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument{argv[index]};
    auto requireValue = [&](const std::string & option) -> std::string {
      if (index + 1 >= argc) {
        throw std::runtime_error(option + " requires a value");
      }
      return argv[++index];
    };

    if (argument == "--help" || argument == "-h") {
      printUsage(argv[0]);
      std::exit(EXIT_SUCCESS);
    } else if (argument == "--request") {
      options.request_path = requireValue(argument);
    } else if (argument == "--output-dir") {
      options.output_dir = requireValue(argument);
    } else if (argument == "--host") {
      options.host = requireValue(argument);
      if (options.host.empty()) {
        throw std::runtime_error("--host must not be empty");
      }
    } else if (argument == "--port") {
      const std::string port_text = requireValue(argument);
      std::size_t consumed = 0;
      const long value = std::stol(port_text, &consumed);
      if (consumed != port_text.size()) {
        throw std::runtime_error("--port must be an integer without trailing characters");
      }
      if (value <= 0 || value > 65535) {
        throw std::runtime_error("--port must be in [1, 65535]");
      }
      options.port = static_cast<std::uint16_t>(value);
    } else if (argument == "--playback-rate") {
      options.playback_rate = parsePositiveCliDouble(argument, requireValue(argument));
    } else if (argument == "--loop-delay") {
      options.loop_delay_s = parseNonNegativeCliDouble(argument, requireValue(argument));
    } else if (argument == "--once") {
      options.once = true;
    } else if (argument == "--no-live") {
      options.live = false;
    } else if (argument == "--mcap") {
      options.mcap_path = std::filesystem::path{requireValue(argument)};
    } else if (argument == "--force") {
      options.force = true;
    } else {
      throw std::runtime_error("unknown option: " + argument);
    }
  }

  if (options.request_path.empty()) {
    throw std::runtime_error("--request is required");
  }
  if (options.output_dir.empty()) {
    throw std::runtime_error("--output-dir is required");
  }
  if (!options.live && options.mcap_path.has_value()) {
    throw std::runtime_error("--mcap cannot be used with --no-live");
  }
  return options;
}

mcc::CartesianMoveLineRequest loadRequest(const std::filesystem::path & path)
{
  const Json::Value root = loadJson(path);
  validateMembers(
    root,
    {"schema_version", "reference_frame_name", "sample_period_s", "path_limits", "segments"},
    {"maximum_sample_count", "synchronization"},
    "request");
  if (requireString(root, "schema_version", "request") != kRequestSchema) {
    throw std::runtime_error("request.schema_version is unsupported");
  }

  mcc::CartesianMoveLineRequest request;
  request.reference_frame_name = requireString(root, "reference_frame_name", "request");
  request.sample_period = requirePositiveNumber(root, "sample_period_s", "request");

  if (root.isMember("maximum_sample_count")) {
    const auto & value = root["maximum_sample_count"];
    if (!value.isUInt64() || value.asUInt64() == 0 ||
      value.asUInt64() > std::numeric_limits<std::size_t>::max())
    {
      throw std::runtime_error("request.maximum_sample_count must be a positive size_t");
    }
    request.maximum_sample_count = static_cast<std::size_t>(value.asUInt64());
  }

  if (root.isMember("synchronization")) {
    const std::string synchronization = requireString(root, "synchronization", "request");
    if (synchronization == "time") {
      request.synchronization = mcc::TrajectorySynchronization::Time;
    } else if (synchronization == "phase") {
      request.synchronization = mcc::TrajectorySynchronization::Phase;
    } else {
      throw std::runtime_error("request.synchronization must be 'time' or 'phase'");
    }
  }

  const auto & limits = root["path_limits"];
  validateMembers(
    limits,
    {"max_velocity_mps", "max_acceleration_mps2", "max_jerk_mps3"},
    {},
    "request.path_limits");
  request.path_limits.max_velocity = requirePositiveNumber(
    limits, "max_velocity_mps", "request.path_limits");
  request.path_limits.max_acceleration = requirePositiveNumber(
    limits, "max_acceleration_mps2", "request.path_limits");
  request.path_limits.max_jerk = requirePositiveNumber(
    limits, "max_jerk_mps3", "request.path_limits");

  const auto & segments = root["segments"];
  if (!segments.isArray() || segments.empty()) {
    throw std::runtime_error("request.segments must be a non-empty array");
  }
  std::set<std::string> frame_names;
  request.segments.reserve(segments.size());
  for (Json::ArrayIndex index = 0; index < segments.size(); ++index) {
    const auto & value = segments[index];
    const std::string context = "request.segments[" + std::to_string(index) + "]";
    validateMembers(
      value,
      {"frame_name", "start_pose", "target_pose"},
      {"current_twist", "current_acceleration", "equivalent_radius_m"},
      context);

    mcc::CartesianMoveLineSegment segment;
    segment.frame_name = requireString(value, "frame_name", context);
    if (!frame_names.insert(segment.frame_name).second) {
      throw std::runtime_error("duplicate frame_name: " + segment.frame_name);
    }
    segment.start_pose = parsePose(value["start_pose"], context + ".start_pose");
    segment.target_pose = parsePose(value["target_pose"], context + ".target_pose");
    if (value.isMember("current_twist")) {
      segment.current_twist = parseTwist(value["current_twist"], context + ".current_twist");
    }
    if (value.isMember("current_acceleration")) {
      segment.current_acceleration = parseAcceleration(
        value["current_acceleration"], context + ".current_acceleration");
    }
    if (value.isMember("equivalent_radius_m")) {
      segment.equivalent_radius_m = requirePositiveNumber(
        value, "equivalent_radius_m", context);
    }
    request.segments.push_back(std::move(segment));
  }
  return request;
}

OutputPaths prepareOutputPaths(
  const std::filesystem::path & output_dir,
  bool force)
{
  OutputPaths paths{
    output_dir / "trajectory.csv",
    output_dir / "cartesian_path_3d.png",
    output_dir / "cartesian_profiles.png"};
  for (const auto & path : {paths.trajectory_csv, paths.path_plot, paths.profiles_plot}) {
    if (!force && std::filesystem::exists(path)) {
      throw std::runtime_error("refusing to overwrite output: " + path.string());
    }
  }
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
  return paths;
}

void writeTrajectoryCsv(
  const std::filesystem::path & path,
  const mcc::CartesianTrajectory & trajectory)
{
  std::ofstream output(path, std::ios::trunc);
  if (!output) {
    throw std::runtime_error("failed to open output CSV: " + path.string());
  }
  output << std::setprecision(17)
         << "time_from_start,reference_frame_name,frame_name,"
         << "position_x,position_y,position_z,"
         << "orientation_x,orientation_y,orientation_z,orientation_w,"
         << "linear_velocity_x,linear_velocity_y,linear_velocity_z,"
         << "angular_velocity_x,angular_velocity_y,angular_velocity_z,"
         << "linear_acceleration_x,linear_acceleration_y,linear_acceleration_z,"
         << "angular_acceleration_x,angular_acceleration_y,angular_acceleration_z\n";
  for (const auto & sample : trajectory.samples) {
    for (const auto & frame : sample.frames) {
      Eigen::Quaterniond orientation(frame.pose.linear());
      orientation.normalize();
      output
        << sample.time_from_start << ','
        << csvCell(frame.reference_frame_name) << ','
        << csvCell(frame.frame_name) << ','
        << frame.pose.translation().x() << ','
        << frame.pose.translation().y() << ','
        << frame.pose.translation().z() << ','
        << orientation.x() << ','
        << orientation.y() << ','
        << orientation.z() << ','
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
  if (!output) {
    throw std::runtime_error("failed while writing output CSV: " + path.string());
  }
}

void renderTrajectoryPlots(
  const OutputPaths & paths,
  const mcc::CartesianTrajectory & trajectory)
{
  if (trajectory.samples.empty() || trajectory.samples.front().frames.empty()) {
    throw std::runtime_error("cannot render an empty Cartesian trajectory");
  }
  plt::backend("Agg");

  const long path_figure = plt::figure();
  Eigen::Vector3d minimum = trajectory.samples.front().frames.front().pose.translation();
  Eigen::Vector3d maximum = minimum;
  for (std::size_t frame_index = 0;
    frame_index < trajectory.samples.front().frames.size();
    ++frame_index)
  {
    const auto & first_frame = trajectory.samples.front().frames[frame_index];
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
    x.reserve(trajectory.samples.size());
    y.reserve(trajectory.samples.size());
    z.reserve(trajectory.samples.size());
    for (const auto & sample : trajectory.samples) {
      const auto & frame = requireFrame(sample, first_frame.frame_name);
      minimum = minimum.cwiseMin(frame.pose.translation());
      maximum = maximum.cwiseMax(frame.pose.translation());
      x.push_back(frame.pose.translation().x());
      y.push_back(frame.pose.translation().y());
      z.push_back(frame.pose.translation().z());
    }
    const std::string color = "C" + std::to_string(frame_index % 10);
    plt::scatter(
      x,
      y,
      z,
      4.0,
      {{"label", first_frame.frame_name}, {"color", color}},
      path_figure);
    plt::scatter(
      std::vector<double>{x.front()},
      std::vector<double>{y.front()},
      std::vector<double>{z.front()},
      64.0,
      {{"marker", "o"}, {"color", color}},
      path_figure);
    plt::scatter(
      std::vector<double>{x.back()},
      std::vector<double>{y.back()},
      std::vector<double>{z.back()},
      81.0,
      {{"marker", "x"}, {"color", color}},
      path_figure);
  }
  Eigen::Vector3d margin = 0.05 * (maximum - minimum);
  for (Eigen::Index index = 0; index < 3; ++index) {
    margin(index) = std::max(margin(index), 0.01);
  }
  const Eigen::Vector3d lower = minimum - margin;
  const Eigen::Vector3d upper = maximum + margin;
  plt::plot3(
    std::vector<double>{lower.x(), upper.x()},
    std::vector<double>{lower.y(), upper.y()},
    std::vector<double>{lower.z(), upper.z()},
    {{"color", "none"}, {"label", "_bounds"}},
    path_figure);
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
    for (const auto & first_frame : trajectory.samples.front().frames) {
      std::vector<double> values;
      values.reserve(trajectory.samples.size());
      for (const auto & sample : trajectory.samples) {
        values.push_back(requireFrame(sample, first_frame.frame_name).pose.translation()(component));
      }
      plt::named_plot(first_frame.frame_name, time, values);
    }
    plt::title(std::string{"position "} + "xyz"[component] + " [m]");
    plt::grid(true);
    plt::legend();
  }

  plt::subplot(3, 2, 4);
  for (const auto & first_frame : trajectory.samples.front().frames) {
    const Eigen::Quaterniond initial(first_frame.pose.linear());
    std::vector<double> values;
    values.reserve(trajectory.samples.size());
    for (const auto & sample : trajectory.samples) {
      const Eigen::Quaterniond current(requireFrame(sample, first_frame.frame_name).pose.linear());
      values.push_back(initial.angularDistance(current));
    }
    plt::named_plot(first_frame.frame_name, time, values);
  }
  plt::title("orientation distance from start [rad]");
  plt::grid(true);
  plt::legend();

  plt::subplot(3, 2, 5);
  for (const auto & first_frame : trajectory.samples.front().frames) {
    std::vector<double> linear;
    std::vector<double> angular;
    for (const auto & sample : trajectory.samples) {
      const auto & frame = requireFrame(sample, first_frame.frame_name);
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
  for (const auto & first_frame : trajectory.samples.front().frames) {
    std::vector<double> linear;
    std::vector<double> angular;
    for (const auto & sample : trajectory.samples) {
      const auto & frame = requireFrame(sample, first_frame.frame_name);
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

std::vector<mcv::LineStrip3d> makeStaticScene(
  const mcc::CartesianMoveLineRequest & request)
{
  std::vector<mcv::LineStrip3d> lines;
  for (std::size_t index = 0; index < request.segments.size(); ++index) {
    const auto & segment = request.segments[index];
    const Eigen::Vector3d start = segment.start_pose.translation();
    const Eigen::Vector3d target = segment.target_pose.translation();
    if ((target - start).norm() > 1.0e-12) {
      lines.push_back(mcv::LineStrip3d{
        segment.frame_name + "/path",
        kSceneChannel,
        request.reference_frame_name,
        {toPoint(start), toPoint(target)},
        toColor(paletteColor(index)),
        4.0,
        true});
    }
    appendAxisTriad(
      lines,
      segment.frame_name + "/start",
      request.reference_frame_name,
      segment.start_pose,
      0.55);
    appendAxisTriad(
      lines,
      segment.frame_name + "/target",
      request.reference_frame_name,
      segment.target_pose,
      0.75);
  }
  return lines;
}

mcv::VisualizationFrame makePlaybackFrame(
  const mcc::CartesianTrajectorySample & sample,
  const std::vector<mcv::LineStrip3d> & static_scene,
  bool include_static_scene,
  std::uint64_t sequence,
  std::int64_t sample_time_ns,
  std::uint64_t emit_time_ns)
{
  mcv::VisualizationFrame visualization;
  visualization.run_id = "cartesian-planning";
  visualization.sequence = sequence;
  visualization.sample_time_ns = sample_time_ns;
  visualization.sample_clock = "cartesian_playback";
  visualization.emit_time_ns = emit_time_ns;
  visualization.status = "playing";
  if (include_static_scene) {
    visualization.line_strips = static_scene;
  }
  visualization.poses.reserve(sample.frames.size());
  for (const auto & frame : sample.frames) {
    Eigen::Quaterniond orientation(frame.pose.linear());
    orientation.normalize();
    visualization.poses.push_back(mcv::NamedPose{
      frame.frame_name,
      "/mc/cartesian/pose/" + frame.frame_name,
      frame.reference_frame_name,
      mcv::Pose3d{
        toPoint(frame.pose.translation()),
        {orientation.x(), orientation.y(), orientation.z(), orientation.w()}}});
    appendAxisTriad(
      visualization.line_strips,
      frame.frame_name + "/current",
      frame.reference_frame_name,
      frame.pose,
      1.0);
  }
  return visualization;
}

void playTrajectory(
  const AppOptions & options,
  const mcc::CartesianMoveLineRequest & request,
  const mcc::CartesianTrajectory & trajectory)
{
  if (trajectory.samples.empty()) {
    throw std::runtime_error("cannot play an empty Cartesian trajectory");
  }

  mcv::FoxgloveFrameSinkOptions sink_options;
  sink_options.server_name = "mcl_cartesian_planning";
  sink_options.host = options.host;
  sink_options.port = options.port;
  sink_options.mcap_path = options.mcap_path;
  mcv::FoxgloveFrameSink sink(std::move(sink_options));
  sink.open({"cartesian-planning", "mcl_cartesian_planning"});

  stop_requested.store(false);
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  const auto static_scene = makeStaticScene(request);
  const auto playback_origin = std::chrono::steady_clock::now();
  std::uint64_t sequence = 0;
  try {
    do {
      const auto loop_start = std::chrono::steady_clock::now();
      bool first_sample = true;
      for (const auto & sample : trajectory.samples) {
        const auto offset = std::chrono::duration<double>(
          sample.time_from_start / options.playback_rate);
        waitUntil(loop_start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(offset));
        if (stop_requested.load()) {
          break;
        }
        const auto logical_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now() - playback_origin).count();
        sink.write(makePlaybackFrame(
          sample,
          static_scene,
          first_sample,
          sequence++,
          logical_time,
          wallTimeNs()));
        first_sample = false;
      }
      sink.flush();
      if (options.once || stop_requested.load()) {
        break;
      }
      waitUntil(
        std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(options.loop_delay_s)));
    } while (!stop_requested.load());
    sink.close();
  } catch (...) {
    try {
      sink.close();
    } catch (...) {
    }
    throw;
  }
}

}  // namespace motion_control_lab::cartesian_planning
