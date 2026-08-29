#include "components/robot/r1/r1_robot_config.hpp"

#include <motion_control_core/dynamics/actuation_model.hpp>
#include <motion_control_core/dynamics/workspace.hpp>
#include <motion_control_core/model/types.hpp>
#include <motion_control_sim/mujoco_torque_simulation.hpp>

#include <Eigen/Core>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

namespace mcc = motion_control::core;
namespace mcs = motion_control::sim;

std::string readFile(const std::string &path) {
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("cannot read model file: " + path);
  }
  std::ostringstream output;
  output << stream.rdbuf();
  return output.str();
}

double firstNumber(const std::string &input, const std::regex &pattern,
                   const std::string &label) {
  std::smatch match;
  if (!std::regex_search(input, match, pattern) || match.size() != 2) {
    throw std::runtime_error("missing model limit for " + label);
  }
  return std::stod(match[1].str());
}

void requireNear(double actual, double expected, double tolerance,
                 const std::string &label) {
  if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance) {
    throw std::runtime_error(label +
                             " mismatch: actual=" + std::to_string(actual) +
                             " expected=" + std::to_string(expected));
  }
}

void requireOk(const mcc::Status &status, const std::string &operation) {
  if (!status.ok()) {
    throw std::runtime_error(operation + ": " + status.message);
  }
}

} // namespace

int main(int argc, char **argv) {
  try {
    if (argc != 3) {
      throw std::runtime_error("expected <urdf> <mjcf>");
    }
    const auto &robot = motion_control_lab::r1RobotConfig();
    const std::string urdf = readFile(argv[1]);
    const std::string mjcf = readFile(argv[2]);
    if (robot.joint_names.size() != 20 || robot.effort_limits.size() != 20) {
      throw std::runtime_error(
          "R1 curated actuator profile must contain exactly 20 joints");
    }
    for (std::size_t index = 0; index < robot.joint_names.size(); ++index) {
      const std::string &name = robot.joint_names[index];
      const double expected = robot.effort_limits[index];
      const std::string escaped = std::regex_replace(
          name, std::regex(R"([.^$|()\[\]{}*+?\\])"), R"(\$&)");
      const double urdf_effort = firstNumber(
          urdf,
          std::regex("<joint\\s+name=\"" + escaped +
                         "\"[\\s\\S]*?<limit[\\s\\S]*?effort=\"([0-9eE+.-]+)\"",
                     std::regex::ECMAScript),
          "URDF " + name);
      const double mjcf_joint_effort = firstNumber(
          mjcf,
          std::regex(
              "<joint\\s+name=\"" + escaped +
              "\"[^>]*actuatorfrcrange=\"-[0-9eE+.-]+\\s+([0-9eE+.-]+)\""),
          "MJCF joint " + name);
      const double mjcf_motor_effort = firstNumber(
          mjcf,
          std::regex("<motor[^>]*joint=\"" + escaped +
                     "\"[^>]*ctrlrange=\"-[0-9eE+.-]+\\s+([0-9eE+.-]+)\""),
          "MJCF motor " + name);
      requireNear(urdf_effort, expected, 1.0e-12, "URDF effort " + name);
      requireNear(mjcf_joint_effort, expected, 1.0e-12,
                  "MJCF joint effort " + name);
      requireNear(mjcf_motor_effort, expected, 1.0e-12,
                  "MJCF motor effort " + name);
    }

    mcc::RobotModelDescription model_description;
    model_description.urdf_path = argv[1];
    model_description.kinematics_reference_frame = robot.base_frame;
    model_description.joint_names = robot.joint_names;
    model_description.root_joint = mcc::RootJointType::Fixed;
    std::shared_ptr<const mcc::RobotModel> model;
    requireOk(mcc::RobotModel::load(model_description, model),
              "load Core R1 model");
    mcc::DynamicsWorkspace workspace;
    requireOk(workspace.configure(model), "configure Core dynamics");
    mcc::RobotState state;
    state.joint_positions =
        Eigen::Map<const Eigen::VectorXd>(robot.default_positions.data(), 20);
    state.joint_velocities.setZero(20);
    requireOk(workspace.update(state), "update Core dynamics");
    Eigen::VectorXd core_bias(workspace.velocitySize());
    requireOk(workspace.nonlinearEffects(core_bias), "read Core bias forces");
    mcc::ActuationModelDescription actuation_description;
    for (std::size_t index = 0; index < robot.joint_names.size(); ++index) {
      actuation_description.actuators.push_back(mcc::ScalarActuatorDescription{
          robot.joint_names[index], -robot.effort_limits[index],
          robot.effort_limits[index]});
    }
    std::shared_ptr<const mcc::ActuationModel> actuation;
    requireOk(
        mcc::ActuationModel::create(model, actuation_description, actuation),
        "create Core R1 actuation map");

    mcs::MujocoTorqueSimulation simulation;
    mcs::ModelDescription simulation_description;
    simulation_description.xml_path = argv[2];
    simulation_description.joint_names = robot.joint_names;
    simulation_description.free_joint_name = "floating_base";
    simulation_description.base_weld_name = "fixed_base_weld";
    simulation_description.base_weld_enabled = true;
    simulation_description.contact_dynamics_enabled = false;
    simulation.load(simulation_description);
    mcs::JointState joint_state;
    joint_state.names = robot.joint_names;
    joint_state.positions = robot.default_positions;
    joint_state.velocities.assign(20, 0.0);
    simulation.setState(joint_state);
    const std::vector<double> mujoco_bias = simulation.jointBiasForces();
    double maximum_error = 0.0;
    std::string maximum_error_joint;
    for (std::size_t index = 0; index < mujoco_bias.size(); ++index) {
      const double error =
          std::abs(mujoco_bias[index] -
                   core_bias(actuation->generalizedVelocityIndices()[index]));
      if (error > maximum_error) {
        maximum_error = error;
        maximum_error_joint = robot.joint_names[index];
      }
    }
    if (maximum_error > 2.0e-3) {
      throw std::runtime_error("R1 URDF/MJCF bias-force mismatch at " +
                               maximum_error_joint + ": " +
                               std::to_string(maximum_error));
    }
    std::cout << "max_bias_force_error=" << maximum_error
              << " joint=" << maximum_error_joint << '\n';
    return EXIT_SUCCESS;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
