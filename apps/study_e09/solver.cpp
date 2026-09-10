#include "solver.hpp"
#include <algorithm>
#include <limits>
#include <placo/kinematics/kinematics_solver.h>
#include <placo/model/robot_wrapper.h>
#include <set>
#include <stdexcept>
namespace study_e09 {
namespace {
void ok(const mcc::Status &s) {
  if (!s.ok())
    throw std::runtime_error(s.message);
}
Eigen::VectorXd vec(const Json::Value &a) {
  Eigen::VectorXd v(a.size());
  for (unsigned i = 0; i < a.size(); ++i)
    v[i] = a[i].asDouble();
  return v;
}
Eigen::Matrix3d rot(const Json::Value &a) {
  Eigen::Matrix3d r;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      r(i, j) = a[i][j].asDouble();
  return r;
}
Json::Value array(const Eigen::VectorXd &v) {
  Json::Value a(Json::arrayValue);
  for (int i = 0; i < v.size(); ++i)
    a.append(v[i]);
  return a;
}
} // namespace
struct Solver::Impl {
  Options o;
  bool placo, hierarchical, orientation, posture, link4, posture_only;
  std::vector<std::string> names, active;
  std::vector<int> indices;
  std::shared_ptr<const mcc::RobotModel> model;
  mcc::KinematicsSolver weighted;
  mcc::HierarchicalKinematicsSolver hierarchy;
  std::vector<mcc::PositionTaskHandle> positions;
  std::vector<mcc::OrientationTaskHandle> orientations;
  mcc::PostureTaskHandle posture_handle;
  std::unique_ptr<placo::model::RobotWrapper> robot;
  std::unique_ptr<placo::kinematics::KinematicsSolver> ps;
  std::vector<placo::kinematics::PositionTask *> pp;
  std::vector<placo::kinematics::OrientationTask *> po;
  placo::kinematics::JointsTask *pj = nullptr;
  explicit Impl(const Options &options)
      : o(options), placo(o.config["method"].asString() == "placo"),
        hierarchical(o.config["layers"].asInt() > 1),
        orientation(o.config["workload"].asInt() >= 1),
        posture(o.config["workload"].asInt() >= 2),
        link4(o.config["workload"].asInt() >= 3),
        posture_only(o.config.get("posture_only", false).asBool()) {
    for (auto &n : o.input["joint_names"])
      names.push_back(n.asString());
    const auto &selected = o.config.isMember("active_joint_names")
                               ? o.config["active_joint_names"]
                               : o.input["active_joint_names"];
    for (auto &n : selected) {
      active.push_back(n.asString());
      indices.push_back(std::find(names.begin(), names.end(), n.asString()) -
                        names.begin());
    }
    if (placo) {
      robot = std::make_unique<placo::model::RobotWrapper>(
          o.input["model"]["locator"].asString(),
          placo::model::RobotWrapper::IGNORE_COLLISIONS |
              placo::model::RobotWrapper::IGNORE_GEOMETRY);
      robot->reset();
      ps = std::make_unique<placo::kinematics::KinematicsSolver>(*robot);
      ps->mask_fbase(true);
      for (auto &n : robot->joint_names())
        if (std::find(active.begin(), active.end(), n) == active.end())
          ps->mask_dof(n);
      ps->problem.regularization = o.config["regularization"].asDouble();
      ps->problem.use_sparsity = true;
      ps->problem.rewrite_equalities = true;
      ps->enable_joint_limits(true);
      ps->enable_velocity_limits(o.config["mode"].asString() == "ServoStep");
      ps->dt = o.config["dt_s"].asDouble();
      for (unsigned i = 0; i < names.size(); ++i)
        robot->set_joint(names[i], o.input["initial_state"]["q"][i].asDouble());
      robot->update_kinematics();
      for (auto side : {"left", "right"}) {
        auto frame = o.input["frames"][side].asString();
        auto t = robot->get_T_world_frame(frame);
        auto &p = ps->add_position_task(frame, t.translation());
        p.configure(std::string(side) + "-position", "soft", 1.);
        pp.push_back(&p);
        if (orientation) {
          auto &r = ps->add_orientation_task(frame, t.rotation());
          r.configure(std::string(side) + "-orientation", "soft", 1.);
          po.push_back(&r);
        }
        if (link4) {
          auto f = std::string(side) + "_arm_link4";
          auto &l = ps->add_position_task(
              f, robot->get_T_world_frame(f).translation());
          l.configure(std::string(side) + "-link4", "soft", 1.);
          pp.push_back(&l);
        }
      }
      if (posture) {
        pj = &ps->add_joints_task();
        pj->configure("posture", "soft", o.config["posture_weight"].asDouble());
      }
      return;
    }
    mcc::RobotModelDescription md;
    md.urdf_path = o.input["model"]["locator"].asString();
    md.kinematics_reference_frame = o.input["root_frame"].asString();
    md.joint_names = names;
    ok(mcc::RobotModel::load(md, model));
    mcc::QpSolverConfig qp;
    qp.backend = o.config["backend"].asString() == "eiquadprog"
                     ? mcc::QpBackend::Eiquadprog
                     : mcc::QpBackend::ProxQp;
    qp.regularization = o.config["regularization"].asDouble();
    qp.proxqp.warm_start_enabled = o.config["warm_start"].asBool();
    qp.eiquadprog.maximum_iterations =
        o.config["eiquadprog_maximum_iterations"].asInt();
    if (!o.config["primal_infeasibility_tolerance"].isNull())
      qp.proxqp.primal_infeasibility_tolerance =
          o.config["primal_infeasibility_tolerance"].asDouble();
    qp.proxqp.maximum_iterations = o.config["maximum_qp_iterations"].asInt();
    qp.proxqp.absolute_tolerance = o.config["absolute_tolerance"].asDouble();
    qp.proxqp.relative_tolerance = o.config["relative_tolerance"].asDouble();
    mcc::KinematicsSolverBuilder wb;
    mcc::HierarchicalKinematicsSolverBuilder hb;
    if (hierarchical) {
      mcc::HierarchicalKinematicsSolverConfig c;
      c.execution = mcc::ServoStepOptions(o.config["dt_s"].asDouble());
      c.qp = qp;
      c.maximum_accepted_hard_violation = o.config["hard_tolerance"].asDouble();
      c.maximum_iterations_policy =
          mcc::HierarchicalMaximumIterationsPolicy::Reject;
      ok(hb.configure(model, active, c));
    } else {
      mcc::KinematicsSolverConfig c;
      c.qp = qp;
      c.maximum_accepted_hard_violation = o.config["hard_tolerance"].asDouble();
      c.joint_limit_policy =
          o.config["mode"].asString() == "ServoStep"
              ? mcc::KinematicsJointLimitPolicy::ModelPositionAndVelocity
              : mcc::KinematicsJointLimitPolicy::ModelPositionOnly;
      if (o.config["mode"].asString() == "ServoStep")
        c.execution = mcc::ServoStepOptions(o.config["dt_s"].asDouble());
      else {
        mcc::TargetSolveOptions t;
        t.maximum_iterations = o.config["maximum_target_iterations"].asInt();
        t.minimum_position_improvement_m =
            o.config["minimum_position_improvement_m"].asDouble();
        t.minimum_orientation_improvement_rad =
            o.config["minimum_orientation_improvement_rad"].asDouble();
        t.minimum_posture_improvement_rad =
            o.config["minimum_posture_improvement_rad"].asDouble();
        t.soft_solve_time_budget_ms = o.config["target_budget_ms"].asDouble();
        c.execution = t;
      }
      c.convergence.position_tolerance_m =
          o.config["position_tolerance"].asDouble();
      c.convergence.orientation_tolerance_rad =
          o.config["orientation_tolerance"].asDouble();
      ok(wb.configure(model, active, c));
    }
    auto addpos = [&](std::string frame, std::string name,
                      mcc::PriorityLevel priority) {
      mcc::PositionTaskConfig c;
      c.name = name;
      c.servo_gain_per_s = o.config["servo_gain"].asDouble();
      mcc::PositionTaskHandle h;
      if (hierarchical)
        ok(hb.addPositionTask(
            priority, frame, c,
            Eigen::Vector3d::Constant(
                o.config["preservation_tolerance"].asDouble()),
            h));
      else
        ok(wb.addPositionTask(frame, c, h));
      positions.push_back(h);
    };
    if (!posture_only)
      for (auto side : {"left", "right"}) {
        auto frame = o.input["frames"][side].asString();
        addpos(frame, std::string(side) + "-position",
               mcc::PriorityLevel::Primary);
        if (orientation) {
          mcc::OrientationTaskConfig c;
          c.name = std::string(side) + "-orientation";
          c.servo_gain_per_s = o.config["servo_gain"].asDouble();
          mcc::OrientationTaskHandle h;
          if (hierarchical)
            ok(hb.addOrientationTask(
                mcc::PriorityLevel::Secondary, frame, c,
                Eigen::Vector3d::Constant(
                    o.config["preservation_tolerance"].asDouble()),
                h));
          else
            ok(wb.addOrientationTask(frame, c, h));
          orientations.push_back(h);
        }
        if (link4)
          addpos(std::string(side) + "_arm_link4", std::string(side) + "-link4",
                 o.config["layers"].asInt() == 3
                     ? mcc::PriorityLevel::Tertiary
                     : mcc::PriorityLevel::Secondary);
      }
    if (posture || posture_only) {
      mcc::PostureTaskConfig c;
      c.name = "posture";
      c.servo_gain_per_s = o.config["servo_gain"].asDouble();
      c.enforcement = mcc::squaredL2Penalty(
          o.config["posture_weight"].asDouble(), active.size());
      if (hierarchical)
        ok(hb.addPostureTask(
            o.config["layers"].asInt() == 3 ? mcc::PriorityLevel::Tertiary
                                            : mcc::PriorityLevel::Secondary,
            c,
            Eigen::VectorXd::Constant(
                active.size(), o.config["preservation_tolerance"].asDouble()),
            posture_handle));
      else
        ok(wb.addPostureTask(c, posture_handle));
    }
    if (hierarchical)
      ok(hb.finalize(hierarchy));
    else
      ok(wb.finalize(weighted));
  }
};
Solver::Solver(const Options &o) : impl(std::make_unique<Impl>(o)) {}
Solver::~Solver() = default;
Result Solver::solve(const Json::Value &sample, const Json::Value &proposal) {
  auto &x = *impl;
  Result result;
  result.placo = x.placo;
  result.hierarchical = x.hierarchical;
  for (auto &v : sample["q"])
    result.q.push_back(v.asDouble());
  for (auto &v : sample["v"])
    result.v.push_back(v.asDouble());
  std::vector<Eigen::Vector3d> positions;
  std::vector<Eigen::Matrix3d> rotations;
  for (auto side : {"left", "right"}) {
    auto r = rot(sample["targets"][side]["rotation"]);
    rotations.push_back(r);
    positions.push_back(vec(sample["targets"][side]["position"]) -
                        r * vec(x.o.input["tcp_offsets"][side]));
    if (x.link4)
      positions.push_back(vec(sample["link4_targets"][side]["position"]));
  }
  auto pref = proposal.isNull() ? x.o.input["posture_target"] : proposal;
  if (pref.isNull())
    pref = x.o.input["initial_state"]["q"];
  if (x.placo) {
    for (unsigned i = 0; i < x.names.size(); ++i)
      x.robot->set_joint(x.names[i], result.q[i]);
    x.robot->update_kinematics();
    bool servo = x.o.config["mode"].asString() == "ServoStep";
    int maximum = servo ? 1 : x.o.config["maximum_target_iterations"].asInt();
    long long target_started = nowNs();
    double previous_position = std::numeric_limits<double>::infinity(),
           previous_orientation = previous_position;
    result.termination_reason = servo ? "single-iteration" : "iteration-budget";
    for (int it = 0; it < maximum; ++it) {
      unsigned k = 0;
      for (auto side : {"left", "right"}) {
        std::string frame = x.o.input["frames"][side].asString();
        auto current = x.robot->get_T_world_frame(frame);
        x.pp[k]->target_world =
            servo ? current.translation() +
                        x.o.config["servo_gain"].asDouble() *
                            x.o.config["dt_s"].asDouble() *
                            (positions[k] - current.translation())
                  : positions[k];
        ++k;
        if (x.link4) {
          auto f = std::string(side) + "_arm_link4";
          auto c = x.robot->get_T_world_frame(f);
          x.pp[k]->target_world =
              servo ? c.translation() + x.o.config["servo_gain"].asDouble() *
                                            x.o.config["dt_s"].asDouble() *
                                            (positions[k] - c.translation())
                    : positions[k];
          ++k;
        }
      }
      for (unsigned i = 0; i < x.po.size(); ++i) {
        auto frame = x.o.input["frames"][i == 0 ? "left" : "right"].asString();
        Eigen::Quaterniond current(
            x.robot->get_T_world_frame(frame).rotation()),
            target(rotations[i]);
        x.po[i]->R_world_frame =
            servo ? current
                        .slerp(x.o.config["servo_gain"].asDouble() *
                                   x.o.config["dt_s"].asDouble(),
                               target)
                        .toRotationMatrix()
                  : rotations[i];
      }
      if (x.pj)
        for (int idx : x.indices) {
          double q = x.robot->get_joint(x.names[idx]);
          x.pj->set_joint(x.names[idx],
                          servo ? q + x.o.config["servo_gain"].asDouble() *
                                          x.o.config["dt_s"].asDouble() *
                                          (pref[idx].asDouble() - q)
                                : pref[idx].asDouble());
        }
      x.ps->solve(true);
      x.robot->update_kinematics();
      ++result.iterations;
      double ep = 0, er = 0;
      for (unsigned i = 0; i < 2; ++i) {
        auto t = x.robot->get_T_world_frame(
            x.o.input["frames"][i == 0 ? "left" : "right"].asString());
        ep = std::max(
            ep, (t.translation() - positions[i * (x.link4 ? 2 : 1)]).norm());
        if (x.orientation)
          er = std::max(
              er, Eigen::AngleAxisd(rotations[i].transpose() * t.rotation())
                      .angle());
      }
      result.converged = ep <= x.o.config["position_tolerance"].asDouble() &&
                         er <= x.o.config["orientation_tolerance"].asDouble();
      if (result.converged) {
        result.termination_reason = "converged";
        break;
      }
      if (!servo &&
          previous_position - ep <=
              x.o.config["minimum_position_improvement_m"].asDouble() &&
          previous_orientation - er <=
              x.o.config["minimum_orientation_improvement_rad"].asDouble()) {
        result.termination_reason = "no-progress";
        break;
      }
      previous_position = ep;
      previous_orientation = er;
      if (!servo && x.o.config["target_budget_ms"].asDouble() > 0 &&
          (nowNs() - target_started) * 1e-6 >=
              x.o.config["target_budget_ms"].asDouble()) {
        result.termination_reason = "soft-time-budget";
        break;
      }
    }
    for (int idx : x.indices) {
      double q = x.robot->get_joint(x.names[idx]);
      result.v[idx] =
          servo ? (q - result.q[idx]) / x.o.config["dt_s"].asDouble() : 0.;
      result.q[idx] = q;
    }
    result.accepted = true;
    return result;
  }
  mcc::InverseKinematicsRequest r;
  r.state.joint_positions = vec(sample["q"]);
  r.state.joint_velocities = vec(sample["v"]);
  r.reference_frame_name = x.o.input["root_frame"].asString();
  for (unsigned i = 0; i < x.positions.size(); ++i)
    r.position_targets.emplace_back(x.positions[i], positions[i]);
  for (unsigned i = 0; i < x.orientations.size(); ++i)
    r.orientation_targets.emplace_back(x.orientations[i], rotations[i]);
  if (x.posture || x.posture_only) {
    Eigen::VectorXd p(x.active.size());
    for (unsigned i = 0; i < x.indices.size(); ++i)
      p[i] = pref[x.indices[i]].asDouble();
    r.posture_targets.emplace_back(
        x.posture_handle, p, !sample.get("disable_posture", false).asBool());
  }
  result.status = x.hierarchical
                      ? x.hierarchy.solveInverseKinematics(r, result.solution,
                                                           result.hierarchy)
                      : x.weighted.solveInverseKinematics(r, result.solution,
                                                          result.weighted);
  result.accepted =
      result.solution.disposition == mcc::ResultDisposition::Accepted;
  result.iterations = result.weighted.iterations;
  result.converged = result.weighted.converged;
  if (result.accepted)
    for (unsigned i = 0; i < x.indices.size(); ++i) {
      result.q[x.indices[i]] = result.solution.joint_positions[i];
      if (result.solution.joint_velocities.size())
        result.v[x.indices[i]] = result.solution.joint_velocities[i];
      else
        result.v[x.indices[i]] = 0.;
    }
  return result;
}
Json::Value Solver::evidence(const Result &r) const {
  Json::Value j;
  j["status"]["code"] = int(r.status.code);
  j["status"]["message"] = r.status.message;
  j["disposition"] = r.accepted ? "accepted" : "rejected";
  j["committed"] = r.accepted;
  j["q"] = Json::arrayValue;
  j["v"] = Json::arrayValue;
  for (double v : r.q)
    j["q"].append(v);
  for (double v : r.v)
    j["v"].append(v);
  j["joint_names"] = impl->o.input["joint_names"];
  j["accepted_position_tolerance"] = impl->o.config["hard_tolerance"];
  j["native_qp_status"] = Json::nullValue;
  j["selected_priority"] = Json::nullValue;
  j["highest_completed_priority"] = Json::nullValue;
  int requested = 0, completed = 0;
  if (r.hierarchical) {
    for (auto &p : r.hierarchy.passes)
      if (p.attempted) {
        ++requested;
        if (p.succeeded)
          ++completed;
        Json::Value d;
        d["pass"] = int(p.pass);
        d["native_status"] = p.native_status;
        d["qp_status"] = int(p.backend_status);
        d["succeeded"] = p.succeeded;
        d["qp_time_ms"] = p.solve_time_ms;
        d["warm_start_used"] = p.warm_start_used;
        d["iterations"] = p.iterations;
        d["primal_residual"] = p.primal_residual;
        d["dual_residual"] = p.dual_residual;
        j["passes"].append(d);
        j["native_qp_status"] = p.native_status;
      }
    if (r.hierarchy.selected_priority)
      j["selected_priority"] = int(*r.hierarchy.selected_priority);
    if (r.hierarchy.highest_completed_priority)
      j["highest_completed_priority"] =
          int(*r.hierarchy.highest_completed_priority);
    j["solution_quality"] =
        !r.accepted ? "rejected"
        : r.hierarchy.solution_quality ==
                mcc::HierarchicalSolutionQuality::FeasibleSuboptimal
            ? "feasible-suboptimal"
        : completed < impl->o.config["layers"].asInt() ? "partial-hierarchy"
                                                       : "full";
    if (impl->o.config["observation"].asString() == "full")
      for (auto &t : r.hierarchy.tasks) {
        Json::Value d;
        d["name"] = t.name;
        d["enabled"] = t.enabled;
        d["priority"] = int(t.priority);
        d["residual_optimum"] = array(t.residual_optimum);
        d["preservation_drift"] = array(t.actual_preservation_drift);
        d["target_error_norm"] = t.target_error_norm;
        j["tasks"].append(d);
      }
  } else {
    requested = r.placo ? r.iterations : r.weighted.iterations;
    completed = requested;
    j["solution_quality"] =
        !r.accepted ? "rejected"
        : (impl->o.config["mode"].asString() == "TargetSolve" && !r.converged)
            ? "feasible-suboptimal"
            : "full";
    if (!r.placo) {
      j["native_qp_status"] = r.weighted.optimization.native_status;
      j["qp_time_ms"] = r.weighted.optimization.solve_time_ms;
      j["warm_start_used"] = r.weighted.optimization.warm_start_used;
    } else
      j["native_qp_status_reason"] =
          "PlaCo solve returns vector or throws; backend status not exposed";
  }
  j["requested_passes"] =
      r.hierarchical ? impl->o.config["layers"].asInt() : requested;
  j["completed_passes"] = completed;
  j["target_reached"] = r.converged;
  j["termination_reason"] =
      r.placo          ? Json::Value(r.termination_reason)
      : r.hierarchical ? Json::Value("native-hierarchy")
                       : Json::Value(int(r.weighted.termination_reason));
  j["assembly_time_ms"] = Json::nullValue;
  j["assembly_time_reason"] = "No comparable assembly-only instrumentation";
  j["active_dof"] = int(impl->active.size());
  j["task_position_rows"] = int(impl->positions.size() * 3);
  j["task_orientation_rows"] = int(impl->orientations.size() * 3);
  j["task_posture_rows"] =
      impl->posture || impl->posture_only ? int(impl->active.size()) : 0;
  j["physical_position_bound_components"] = int(impl->active.size());
  j["physical_velocity_bound_components"] =
      impl->o.config["mode"] == "ServoStep" ? int(impl->active.size()) : 0;
  if (r.placo) {
    j["native_variable_count"] = impl->ps->problem.n_variables;
    j["native_free_variables"] = impl->ps->problem.free_variables;
    j["native_slack_variables"] = impl->ps->problem.slack_variables;
    j["native_equalities"] = impl->ps->problem.n_equalities;
    j["native_inequalities"] = impl->ps->problem.n_inequalities;
    j["task_position_rows"] = int(impl->pp.size() * 3);
    j["task_orientation_rows"] = int(impl->po.size() * 3);
  } else {
    j["native_variable_count"] = int(impl->active.size());
    j["native_free_variables"] = Json::nullValue;
    j["native_slack_variables"] = Json::nullValue;
    j["native_matrix_dimensions_reason"] =
        "Only active decision dimension is public; backend "
        "reformulation/preservation row counts unavailable";
  }
  j["allocation_count"] = Json::nullValue;
  j["allocation_bytes"] = Json::nullValue;
  j["allocation_reason"] = "Allocator instrumentation unavailable; not zero";
  return j;
}
} // namespace study_e09
