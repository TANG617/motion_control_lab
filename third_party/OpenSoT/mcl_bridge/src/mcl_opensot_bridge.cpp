#include "mcl_opensot_bridge.h"

#include <OpenSoT/SubTask.h>
#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/VelocityLimits.h>
#include <OpenSoT/solvers/iHQP.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/utils/AutoStack.h>
#include <xbot2_interface/xbotinterface2.h>

#include <Eigen/Core>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iterator>
#include <limits>
#include <list>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
constexpr char kUpstreamRevision[] = "f1d8c733bcff49ed5b213191ac85a0629bb97d43";

void set_error(char* destination, size_t capacity, const std::string& message)
{
  if (destination == nullptr || capacity == 0)
  {
    return;
  }
  const size_t count = std::min(capacity - 1, message.size());
  std::memcpy(destination, message.data(), count);
  destination[count] = '\0';
}

void validate_request(const mcl_opensot_request& request)
{
  if (request.urdf_path == nullptr || request.base_frame == nullptr ||
      request.end_effector_frame == nullptr || request.joint_names == nullptr ||
      request.initial_positions == nullptr || request.goal_positions == nullptr)
  {
    throw std::invalid_argument("OpenSoT request contains a null required field");
  }
  if (request.joint_count == 0 || request.maximum_iterations == 0)
  {
    throw std::invalid_argument("OpenSoT request must contain joints and iterations");
  }
  if (!(request.position_tolerance_m > 0.0) || !(request.cartesian_lambda > 0.0) ||
      !(request.postural_lambda >= 0.0) || !(request.regularization > 0.0) ||
      !(request.velocity_limit_period_s > 0.0))
  {
    throw std::invalid_argument("OpenSoT request contains invalid solver parameters");
  }
  for (size_t index = 0; index < request.joint_count; ++index)
  {
    if (request.joint_names[index] == nullptr || request.joint_names[index][0] == '\0' ||
        !std::isfinite(request.initial_positions[index]) ||
        !std::isfinite(request.goal_positions[index]))
    {
      throw std::invalid_argument("OpenSoT request contains an invalid joint entry");
    }
  }
}

std::string read_text_file(const std::string& path)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream)
  {
    throw std::runtime_error("Unable to read URDF: " + path);
  }
  return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

XBot::JointNameMap make_joint_map(
  const mcl_opensot_request& request,
  const double* positions)
{
  XBot::JointNameMap values;
  for (size_t index = 0; index < request.joint_count; ++index)
  {
    if (!values.emplace(request.joint_names[index], positions[index]).second)
    {
      throw std::invalid_argument(
        std::string("Duplicate joint name in OpenSoT request: ") + request.joint_names[index]);
    }
  }
  return values;
}

uint32_t count_joint_limit_violations(
  const XBot::ModelInterface& model,
  const Eigen::VectorXd& lower,
  const Eigen::VectorXd& upper)
{
  const Eigen::VectorXd position = model.getJointPosition();
  uint32_t violations = 0;
  for (Eigen::Index index = 0; index < position.size(); ++index)
  {
    if (position[index] < lower[index] - 1e-12 || position[index] > upper[index] + 1e-12)
    {
      ++violations;
    }
  }
  return violations;
}

Eigen::Affine3d get_task_pose(
  const XBot::ModelInterface& model,
  const char* distal_frame,
  const char* base_frame)
{
  if (std::strcmp(base_frame, WORLD_FRAME_NAME) == 0)
  {
    return model.getPose(distal_frame);
  }
  return model.getPose(distal_frame, base_frame);
}

void emit_sample(
  const mcl_opensot_request& request,
  const XBot::ModelInterface& model,
  const Eigen::Vector3d& target,
  uint32_t iteration,
  double solve_time_us,
  mcl_opensot_trace_callback callback,
  void* user_data)
{
  if (callback == nullptr)
  {
    return;
  }

  const Eigen::Vector3d achieved =
    get_task_pose(model, request.end_effector_frame, request.base_frame).translation();
  XBot::JointNameMap position_map;
  model.getJointPosition(position_map);
  std::vector<double> positions;
  positions.reserve(request.joint_count);
  for (size_t index = 0; index < request.joint_count; ++index)
  {
    const auto found = position_map.find(request.joint_names[index]);
    if (found == position_map.end())
    {
      throw std::runtime_error(
        std::string("OpenSoT model did not return joint: ") + request.joint_names[index]);
    }
    positions.push_back(found->second);
  }

  mcl_opensot_trace_sample sample{};
  sample.iteration = iteration;
  for (int axis = 0; axis < 3; ++axis)
  {
    sample.target_position_m[axis] = target[axis];
    sample.achieved_position_m[axis] = achieved[axis];
  }
  sample.position_error_m = (target - achieved).norm();
  sample.solve_time_us = solve_time_us;
  sample.joint_positions = positions.data();
  sample.joint_count = positions.size();
  callback(user_data, &sample);
}
}  // namespace

extern "C" int mcl_opensot_solve_position_ik(
  const mcl_opensot_request* request,
  mcl_opensot_trace_callback trace_callback,
  void* trace_user_data,
  mcl_opensot_result* result,
  char* error_message,
  size_t error_message_capacity)
{
  if (error_message != nullptr && error_message_capacity > 0)
  {
    error_message[0] = '\0';
  }
  if (request == nullptr || result == nullptr)
  {
    set_error(error_message, error_message_capacity, "OpenSoT request or result is null");
    return 1;
  }

  *result = {};
  try
  {
    validate_request(*request);
    const std::string urdf = read_text_file(request->urdf_path);
    auto model = XBot::ModelInterface::getModel(urdf, "pin");

    const XBot::JointNameMap initial = make_joint_map(*request, request->initial_positions);
    const XBot::JointNameMap goal = make_joint_map(*request, request->goal_positions);

    model->setJointPosition(model->getNeutralQ());
    model->setJointPosition(goal);
    model->update();
    const Eigen::Affine3d target_pose =
      get_task_pose(*model, request->end_effector_frame, request->base_frame);
    const Eigen::Vector3d target_position = target_pose.translation();

    model->setJointPosition(model->getNeutralQ());
    model->setJointPosition(initial);
    model->update();
    const Eigen::VectorXd initial_q = model->getJointPosition();

    auto cartesian = std::make_shared<OpenSoT::tasks::velocity::Cartesian>(
      "left_ee", *model, request->end_effector_frame, request->base_frame);
    cartesian->setReference(target_pose);
    cartesian->setLambda(request->cartesian_lambda);
    const std::list<unsigned int> position_indices{0, 1, 2};
    auto position = std::make_shared<OpenSoT::SubTask>(cartesian, position_indices);

    auto postural = std::make_shared<OpenSoT::tasks::velocity::Postural>(*model, "postural");
    postural->setReference(initial_q);
    postural->setLambda(request->postural_lambda);

    Eigen::VectorXd lower;
    Eigen::VectorXd upper;
    Eigen::VectorXd velocity;
    model->getJointLimits(lower, upper);
    model->getVelocityLimits(velocity);
    auto joint_limits = std::make_shared<OpenSoT::constraints::velocity::JointLimits>(
      *model, upper, lower);
    auto velocity_limits = std::make_shared<OpenSoT::constraints::velocity::VelocityLimits>(
      *model, velocity, request->velocity_limit_period_s);

    OpenSoT::AutoStack::Ptr stack =
      (position / postural) << joint_limits << velocity_limits;
    stack->update();
    OpenSoT::solvers::iHQP solver(
      *stack,
      request->regularization,
      OpenSoT::solvers::solver_back_ends::OSQP);

    double error =
      (target_position -
       get_task_pose(*model, request->end_effector_frame, request->base_frame).translation())
        .norm();
    result->initial_position_error_m = error;
    result->joint_limit_violation_count += count_joint_limit_violations(*model, lower, upper);
    emit_sample(
      *request, *model, target_position, 0, 0.0, trace_callback, trace_user_data);

    double total_solve_time_us = 0.0;
    for (uint32_t iteration = 1;
         iteration <= request->maximum_iterations && error > request->position_tolerance_m;
         ++iteration)
    {
      stack->update();
      Eigen::VectorXd delta;
      const auto start = std::chrono::steady_clock::now();
      const bool solved = solver.solve(delta);
      const auto stop = std::chrono::steady_clock::now();
      const double solve_time_us =
        std::chrono::duration<double, std::micro>(stop - start).count();
      if (!solved || !delta.allFinite())
      {
        throw std::runtime_error("OpenSoT iHQP returned an invalid solution");
      }

      model->integrateJointPosition(delta);
      model->update();
      error =
        (target_position -
         get_task_pose(*model, request->end_effector_frame, request->base_frame).translation())
          .norm();
      if (!std::isfinite(error))
      {
        throw std::runtime_error("OpenSoT produced a non-finite task error");
      }

      result->iterations = iteration;
      total_solve_time_us += solve_time_us;
      result->max_solve_time_us = std::max(result->max_solve_time_us, solve_time_us);
      result->joint_limit_violation_count += count_joint_limit_violations(*model, lower, upper);
      emit_sample(
        *request,
        *model,
        target_position,
        iteration,
        solve_time_us,
        trace_callback,
        trace_user_data);
    }

    result->final_position_error_m = error;
    result->converged = error <= request->position_tolerance_m ? 1U : 0U;
    if (result->iterations > 0)
    {
      result->mean_solve_time_us = total_solve_time_us / result->iterations;
    }
    return 0;
  }
  catch (const std::exception& exception)
  {
    set_error(error_message, error_message_capacity, exception.what());
    return 2;
  }
  catch (...)
  {
    set_error(error_message, error_message_capacity, "Unknown OpenSoT bridge failure");
    return 3;
  }
}

extern "C" const char* mcl_opensot_upstream_revision(void)
{
  return kUpstreamRevision;
}
