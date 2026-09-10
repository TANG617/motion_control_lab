#include "solver.hpp"
#include <algorithm>
namespace study_e11 {
Solver::Solver(const Options &opt)
    : o(opt), placo(o.config["method"].asString() == "placo-controlled"),
      hqp(o.config["method"].asString() == "mcc-hqp-2"),
      names(strings(o.input["joint_names"])),
      active(strings(o.input["active_joint_names"])) {
  double dt = o.config["dt_s"].asDouble(),
         gain = o.config["servo_gain_per_s"].asDouble(),
         weight = o.config["posture_weight"].asDouble();
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
    ps->dt = dt;
    ps->problem.regularization = o.config["regularization"].asDouble();
    ps->problem.rewrite_equalities = false;
    ps->enable_joint_limits(true);
    ps->enable_velocity_limits(true);
    for (size_t i = 0; i < names.size(); i++) {
      robot->set_joint(names[i],
                       o.input["initial_state"]["q"][(int)i].asDouble());
      robot->set_joint_limits(names[i],
                              o.input["limits"]["lower"][(int)i].asDouble(),
                              o.input["limits"]["upper"][(int)i].asDouble());
      robot->set_velocity_limit(
          names[i], o.input["limits"]["velocity"][(int)i].asDouble());
    }
    robot->update_kinematics();
    int i = 0;
    for (auto side : {"left", "right"}) {
      auto f = o.input["frames"][side].asString();
      pp[i] = &ps->add_position_task(f, Eigen::Vector3d::Zero());
      pp[i]->configure(std::string(side) + "-position", "soft", 1.);
      po[i] = &ps->add_orientation_task(f, Eigen::Matrix3d::Identity());
      po[i]->configure(std::string(side) + "-orientation", "soft", 1.);
      i++;
    }
    auto &p = ps->add_joints_task();
    pposture = &p;
    p.configure("posture", "soft", weight);
    for (size_t k = 0; k < names.size(); k++)
      if (std::find(active.begin(), active.end(), names[k]) != active.end())
        p.set_joint(names[k], o.input["initial_state"]["q"][(int)k].asDouble());
    return;
  }
  mcc::RobotModelDescription md;
  md.urdf_path = o.input["model"]["locator"].asString();
  md.joint_names = names;
  md.kinematics_reference_frame = o.input["root_frame"].asString();
  require(mcc::RobotModel::load(md, model));
  mcc::KinematicsSolverBuilder wb;
  mcc::HierarchicalKinematicsSolverBuilder hb;
  if (hqp) {
    mcc::HierarchicalKinematicsSolverConfig c;
    c.execution = mcc::ServoStepOptions{dt};
    c.qp.backend = mcc::QpBackend::Eiquadprog;
    c.qp.regularization = o.config["regularization"].asDouble();
    c.maximum_iterations_policy =
        mcc::HierarchicalMaximumIterationsPolicy::Reject;
    require(hb.configure(model, active, c));
  } else {
    mcc::KinematicsSolverConfig c;
    c.execution = mcc::ServoStepOptions{dt};
    c.qp.backend = mcc::QpBackend::Eiquadprog;
    c.qp.regularization = o.config["regularization"].asDouble();
    c.joint_limit_policy =
        mcc::KinematicsJointLimitPolicy::ModelPositionAndVelocity;
    require(wb.configure(model, active, c));
  }
  int i = 0;
  for (auto side : {"left", "right"}) {
    mcc::PositionTaskConfig p;
    p.name = std::string(side) + "-position";
    p.servo_gain_per_s = gain;
    mcc::OrientationTaskConfig r;
    r.name = std::string(side) + "-orientation";
    r.servo_gain_per_s = gain;
    auto f = o.input["frames"][side].asString();
    if (hqp) {
      require(hb.addPositionTask(mcc::PriorityLevel::Primary, f, p,
                                 Eigen::Vector3d::Constant(1e-8), ph[i]));
      require(hb.addOrientationTask(mcc::PriorityLevel::Secondary, f, r,
                                    Eigen::Vector3d::Constant(1e-8), oh[i]));
    } else {
      require(wb.addPositionTask(f, p, ph[i]));
      require(wb.addOrientationTask(f, r, oh[i]));
    }
    i++;
  }
  mcc::PostureTaskConfig p;
  p.enforcement = mcc::squaredL2Penalty(weight, active.size());
  p.servo_gain_per_s = gain;
  if (hqp) {
    require(hb.addPostureTask(mcc::PriorityLevel::Secondary, p,
                              Eigen::VectorXd::Constant(active.size(), 1e-8),
                              posture));
    require(hb.finalize(hierarchy));
  } else {
    require(wb.addPostureTask(p, posture));
    require(wb.finalize(weighted));
  }
}
Result Solver::solve(const std::vector<double> &q, const std::vector<double> &v,
                     const Json::Value &targets) {
  Result out;
  double gain = o.config["servo_gain_per_s"].asDouble(),
         dt = o.config["dt_s"].asDouble();
  if (placo) {
    for (size_t k = 0; k < names.size(); k++) {
      robot->set_joint(names[k], q[k]);
      robot->set_joint_velocity(names[k], v[k]);
    }
    robot->update_kinematics();
    int i = 0;
    for (auto side : {"left", "right"}) {
      auto t = pose(targets[side]);
      Eigen::Vector3d off;
      for (int k = 0; k < 3; k++)
        off[k] = o.input["tcp_offsets"][side][k].asDouble();
      t.translation() -= t.linear() * off;
      auto now = robot->get_T_world_frame(o.input["frames"][side].asString());
      pp[i]->target_world =
          now.translation() + gain * dt * (t.translation() - now.translation());
      Eigen::AngleAxisd aa(t.linear() * now.linear().transpose());
      po[i]->R_world_frame =
          Eigen::AngleAxisd(gain * dt * aa.angle(), aa.axis())
              .toRotationMatrix() *
          now.linear();
      i++;
    }
    for (size_t k = 0; k < names.size(); ++k)
      if (std::find(active.begin(), active.end(), names[k]) != active.end())
        pposture->set_joint(
            names[k],
            q[k] +
                gain * dt *
                    (o.input["initial_state"]["q"][(int)k].asDouble() - q[k]));
    ps->solve(true);
    for (auto &n : names) {
      out.q.push_back(robot->get_joint(n));
      out.v.push_back(robot->get_joint_velocity(n));
    }
    out.accepted = true;
    out.native["status"] = "solve-returned";
    out.native["native_qp_status"] = Json::nullValue;
    out.native["native_qp_status_reason"] =
        "PlaCo API does not expose native termination enum";
    out.native["solution_quality"] = "weighted-returned";
    out.native["quality_category"] = "full";
    out.native["quality_category_basis"] =
        "vendored PlaCo returned after finite QP and native feasibility "
        "checks; independent guardrails separate";
    out.native["target_reached"] = Json::Value();
    out.native["target_reached_reason"] =
        "ServoStep numerical full does not imply pose convergence";
    out.native["requested_passes"] = 1;
    out.native["completed_passes"] = 1;
    return out;
  }
  mcc::InverseKinematicsRequest r;
  r.reference_frame_name = o.input["root_frame"].asString();
  r.state.joint_positions =
      Eigen::Map<const Eigen::VectorXd>(q.data(), q.size());
  r.state.joint_velocities =
      Eigen::Map<const Eigen::VectorXd>(v.data(), v.size());
  int i = 0;
  for (auto side : {"left", "right"}) {
    auto t = pose(targets[side]);
    Eigen::Vector3d off;
    for (int k = 0; k < 3; k++)
      off[k] = o.input["tcp_offsets"][side][k].asDouble();
    t.translation() -= t.linear() * off;
    r.position_targets.push_back({ph[i], t.translation(), true});
    r.orientation_targets.push_back({oh[i], t.linear(), true});
    i++;
  }
  Eigen::VectorXd qp(active.size());
  for (size_t k = 0; k < active.size(); k++) {
    auto pos = std::find(names.begin(), names.end(), active[k]) - names.begin();
    qp[k] = o.input["initial_state"]["q"][(int)pos].asDouble();
  }
  r.posture_targets.push_back({posture, qp, true});
  mcc::InverseKinematicsSolution s;
  mcc::Status status;
  if (hqp) {
    mcc::HierarchicalInverseKinematicsDiagnostics d;
    status = hierarchy.solveInverseKinematics(r, s, d);
    out.native["native_solution_quality"] = (int)d.solution_quality;
    out.native["selected_priority"] =
        d.selected_priority ? Json::Value((int)*d.selected_priority)
                            : Json::Value();
    out.native["highest_completed_priority"] =
        d.highest_completed_priority
            ? Json::Value((int)*d.highest_completed_priority)
            : Json::Value();
    int done = 0;
    for (auto &p : d.passes) {
      Json::Value j;
      j["attempted"] = p.attempted;
      j["succeeded"] = p.succeeded;
      j["backend_status"] = (int)p.backend_status;
      j["native_status"] = p.native_status;
      j["solve_time_ms"] = p.solve_time_ms;
      out.native["passes"].append(j);
      done += p.succeeded;
    }
    out.native["requested_passes"] = 2;
    out.native["completed_passes"] = done;
    out.native["solution_quality"] =
        done == 2
            ? "full"
            : (mcc::isAccepted(s.disposition) ? "partial" : "not-accepted");
    out.native["native_qp_status"] = (int)d.terminal_status;
    const bool accepted = status.ok() && mcc::isAccepted(s.disposition);
    out.native["quality_category"] =
        !accepted
            ? (d.terminal_status == mcc::QpSolveStatus::NotRun ? "not-run"
                                                               : "rejected")
        : d.solution_quality ==
                mcc::HierarchicalSolutionQuality::FeasibleSuboptimal
            ? "feasible-suboptimal"
        : done == 2 && d.solution_quality ==
                           mcc::HierarchicalSolutionQuality::Converged
            ? "full"
            : "partial-hierarchy";
  } else {
    mcc::InverseKinematicsDiagnostics d;
    status = weighted.solveInverseKinematics(r, s, d);
    out.native["native_qp_status"] = (int)d.optimization.solver_status;
    out.native["solution_quality"] = "weighted-native";
    out.native["requested_passes"] = 1;
    out.native["completed_passes"] =
        d.optimization.solver_status == mcc::QpSolveStatus::Optimal ? 1 : 0;
    out.native["quality_category"] =
        status.ok() && mcc::isAccepted(s.disposition)
            ? (d.optimization.solver_status == mcc::QpSolveStatus::Optimal
                   ? "full"
                   : "feasible-suboptimal")
            : (d.optimization.solver_status == mcc::QpSolveStatus::NotRun
                   ? "not-run"
                   : "rejected");
  }
  out.native["target_reached"] = Json::Value();
  out.native["target_reached_reason"] =
      "ServoStep numerical quality is separate from pose convergence";
  out.native["quality_scope"] =
      "native numerical category; independent guardrail status separate";
  out.native["status"] = status.message;
  out.native["status_ok"] = status.ok();
  out.accepted = status.ok() && mcc::isAccepted(s.disposition);
  if (s.joint_positions.size()) {
    out.q = q;
    out.v.assign(q.size(), 0.0);
    for (size_t i = 0; i < active.size(); ++i) {
      const auto full =
          std::find(names.begin(), names.end(), active[i]) - names.begin();
      out.q[full] = s.joint_positions[i];
      out.v[full] = s.joint_velocities[i];
    }
    out.native["joint_mapping"] =
        "active native result mapped by name to full canonical order; inactive "
        "held with zero velocity";
  }
  return out;
}
} // namespace study_e11
