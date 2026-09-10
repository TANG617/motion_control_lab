#include "solver.hpp"
#include <algorithm>
#include <motion_control_core/kinematics/hierarchical_solver.hpp>
#include <motion_control_core/kinematics/solver.hpp>
#include <motion_control_core/optimization/hierarchical_problem.hpp>
#include <motion_control_core/optimization/problem.hpp>
#include <placo/kinematics/kinematics_solver.h>
#include <set>
#include <stdexcept>
namespace study {
namespace mcc = motion_control::core;
void ok(const mcc::Status &s) {
  if (!s.ok())
    throw std::runtime_error(s.message);
}
class Solver::Impl {
public:
  Options o;
  bool hierarchy, placo, analytic;
  std::shared_ptr<const mcc::RobotModel> model;
  mcc::KinematicsSolver weighted;
  mcc::HierarchicalKinematicsSolver hqp;
  mcc::PositionTaskHandle ph[4];
  mcc::OrientationTaskHandle oh[2];
  mcc::PostureTaskHandle posture;
  std::unique_ptr<placo::model::RobotWrapper> robot;
  std::unique_ptr<placo::kinematics::KinematicsSolver> ps;
  placo::kinematics::PositionTask *pp[4]{};
  placo::kinematics::OrientationTask *po[2]{};
  placo::kinematics::JointsTask *pj{};
  mcc::JointNames names, active;
  std::vector<int> indices;
  bool extra;
  double dt, gain;
  explicit Impl(const Options &v)
      : o(v), hierarchy(o.method.find("hqp") != std::string::npos),
        placo(o.method.find("placo") != std::string::npos),
        analytic(o.input.isMember("analytic")),
        extra(o.config["secondary_tasks"].asBool()),
        dt(o.config["period_s"].asDouble()),
        gain(o.config["gain_per_s"].asDouble()) {
    for (auto &n : o.input["joint_names"])
      names.push_back(n.asString());
    for (auto &n : o.input["active_joint_names"]) {
      active.push_back(n.asString());
      indices.push_back(std::find(names.begin(), names.end(), n.asString()) -
                        names.begin());
    }
    if (analytic)
      return;
    if (placo) {
      robot = std::make_unique<placo::model::RobotWrapper>(
          o.input["model"]["locator"].asString(),
          placo::model::RobotWrapper::IGNORE_COLLISIONS |
              placo::model::RobotWrapper::IGNORE_GEOMETRY);
      robot->reset();
      ps = std::make_unique<placo::kinematics::KinematicsSolver>(*robot);
      ps->mask_fbase(true);
      std::set<std::string> a(active.begin(), active.end());
      for (auto &n : robot->joint_names())
        if (!a.count(n))
          ps->mask_dof(n);
      ps->dt = dt;
      ps->problem.regularization = o.config["regularization"].asDouble();
      ps->problem.rewrite_equalities = true;
      ps->problem.use_sparsity = false;
      ps->enable_joint_limits(true);
      ps->enable_velocity_limits(o.config["mode"].asString() == "ServoStep");
      for (int k = 0; k < 2; ++k) {
        std::string s = k == 0 ? "left" : "right";
        pp[k] = &ps->add_position_task(o.input["frames"][s].asString(),
                                       Eigen::Vector3d::Zero());
        pp[k]->configure(s + "-position",
                         o.config["enforcement"].asString() == "hard" ? "hard"
                                                                      : "soft",
                         1.0);
        po[k] = &ps->add_orientation_task(o.input["frames"][s].asString(),
                                          Eigen::Matrix3d::Identity());
        po[k]->configure(s + "-orientation", "soft",
                         o.config["primary_only"].asBool()
                             ? 0.
                             : o.config["orientation_weight"].asDouble());
        if (extra) {
          pp[k + 2] =
              &ps->add_position_task(s + "_arm_link4", Eigen::Vector3d::Zero());
          pp[k + 2]->configure(s + "-elbow", "soft",
                               o.config["secondary_weight"].asDouble());
        }
      }
      if (extra) {
        pj = &ps->add_joints_task();
        pj->configure("posture", "soft",
                      o.config["secondary_weight"].asDouble());
      }
      return;
    }
    mcc::RobotModelDescription d;
    d.urdf_path = o.input["model"]["locator"].asString();
    d.kinematics_reference_frame = o.input["root_frame"].asString();
    d.joint_names = names;
    ok(mcc::RobotModel::load(d, model));
    if (o.config["native_acceleration"].asBool()) {
      d.joint_limits = model->jointLimits();
      for (auto &l : d.joint_limits)
        l.acceleration = o.config["acceleration_limit"].asDouble();
      ok(mcc::RobotModel::load(d, model));
    }
    mcc::KinematicsSolverBuilder w;
    mcc::HierarchicalKinematicsSolverBuilder h;
    mcc::KinematicsSolverConfig c;
    c.execution = mcc::ServoStepOptions{dt};
    if (o.config["mode"].asString() == "TargetSolve") {
      mcc::TargetSolveOptions t;
      t.maximum_iterations = o.config["target_iterations"].asInt();
      t.soft_solve_time_budget_ms = o.config["target_budget_ms"].asDouble();
      t.minimum_position_improvement_m =
          o.config["target_min_position_improvement_m"].asDouble();
      t.minimum_orientation_improvement_rad =
          o.config["target_min_orientation_improvement_rad"].asDouble();
      t.minimum_posture_improvement_rad =
          o.config["target_min_posture_improvement_rad"].asDouble();
      c.execution = t;
    }
    c.convergence.position_tolerance_m =
        o.config["position_tolerance_m"].asDouble();
    c.convergence.orientation_tolerance_rad =
        o.config["orientation_tolerance_rad"].asDouble();
    c.qp.regularization = o.config["regularization"].asDouble();
    c.qp.proxqp.maximum_iterations = o.config["qp_iterations"].asInt();
    c.qp.proxqp.absolute_tolerance =
        o.config["qp_absolute_tolerance"].asDouble();
    c.qp.proxqp.relative_tolerance = 0.;
    c.qp.proxqp.warm_start_enabled = o.config["warm_start"].asBool();
    c.maximum_accepted_hard_violation = o.config["hard_tolerance"].asDouble();
    c.joint_limit_policy =
        mcc::KinematicsJointLimitPolicy::ExplicitRequirements;
    if (hierarchy) {
      mcc::HierarchicalKinematicsSolverConfig hc;
      hc.execution = mcc::ServoStepOptions{dt};
      hc.qp = c.qp;
      hc.joint_limit_policy = c.joint_limit_policy;
      hc.maximum_accepted_hard_violation = c.maximum_accepted_hard_violation;
      hc.maximum_iterations_policy =
          mcc::HierarchicalMaximumIterationsPolicy::Reject;
      ok(h.configure(model, active, hc));
    } else
      ok(w.configure(model, active, c));
    mcc::TaskScaleGroupHandle scales[2];
    bool scaled = o.config["enforcement"].asString() == "scaled";
    if (scaled) {
      for (int k = 0;
           k < (o.config["scale_group"].asString() == "shared" ? 1 : 2); ++k) {
        mcc::TaskScaleGroupConfig sg;
        sg.name = "progress-" + std::to_string(k);
        sg.progress_weight = o.config["scale_progress_weight"].asDouble();
        if (hierarchy)
          ok(h.addTaskScaleGroup(mcc::PriorityLevel::Primary, {sg, 1e-7},
                                 scales[k]));
        else
          ok(w.addTaskScaleGroup(sg, scales[k]));
      }
      if (o.config["scale_group"].asString() == "shared")
        scales[1] = scales[0];
    }
    for (int k = 0; k < 2; ++k) {
      std::string s = k == 0 ? "left" : "right";
      mcc::PositionTaskConfig pc;
      pc.name = s + "-position";
      pc.servo_gain_per_s = gain;
      pc.enforcement = mcc::squaredL2Penalty(1., 3);
      if (o.config["enforcement"].asString() == "hard")
        pc.enforcement =
            mcc::HardEnforcement{c.maximum_accepted_hard_violation};
      if (scaled)
        pc.enforcement = mcc::ScaledEnforcement{
            scales[k], c.maximum_accepted_hard_violation};
      auto priority = o.config["swap_priorities"].asBool()
                          ? mcc::PriorityLevel::Secondary
                          : mcc::PriorityLevel::Primary;
      if (hierarchy)
        ok(h.addPositionTask(priority, o.input["frames"][s].asString(), pc,
                             Eigen::Vector3d::Constant(1e-7), ph[k]));
      else
        ok(w.addPositionTask(o.input["frames"][s].asString(), pc, ph[k]));
      mcc::OrientationTaskConfig oc;
      oc.name = s + "-orientation";
      oc.servo_gain_per_s = gain;
      oc.enforcement =
          mcc::squaredL2Penalty(o.config["orientation_weight"].asDouble(), 3);
      auto op = o.config["swap_priorities"].asBool()
                    ? mcc::PriorityLevel::Primary
                    : mcc::PriorityLevel::Secondary;
      if (hierarchy)
        ok(h.addOrientationTask(op, o.input["frames"][s].asString(), oc,
                                Eigen::Vector3d::Constant(1e-7), oh[k]));
      else
        ok(w.addOrientationTask(o.input["frames"][s].asString(), oc, oh[k]));
      if (extra) {
        pc.name = s + "-elbow";
        pc.enforcement =
            mcc::squaredL2Penalty(o.config["secondary_weight"].asDouble(), 3);
        auto ep = o.method.find("3") != std::string::npos
                      ? mcc::PriorityLevel::Tertiary
                      : mcc::PriorityLevel::Secondary;
        if (hierarchy)
          ok(h.addPositionTask(ep, s + "_arm_link4", pc,
                               Eigen::Vector3d::Constant(1e-7), ph[k + 2]));
        else
          ok(w.addPositionTask(s + "_arm_link4", pc, ph[k + 2]));
      }
    }
    if (extra) {
      mcc::PostureTaskConfig p;
      p.name = "posture";
      p.servo_gain_per_s = gain;
      p.enforcement =
          mcc::squaredL2Penalty(o.config["secondary_weight"].asDouble(), 1);
      if (hierarchy)
        ok(h.addPostureTask(o.method.find("3") != std::string::npos
                                ? mcc::PriorityLevel::Tertiary
                                : mcc::PriorityLevel::Secondary,
                            p, Eigen::VectorXd::Constant(active.size(), 1e-7),
                            posture));
      else
        ok(w.addPostureTask(p, posture));
    }
    mcc::JointPositionLimitConfig lp;
    lp.margin = 0.;
    lp.enforcement = mcc::HardEnforcement{c.maximum_accepted_hard_violation};
    lp.braking_velocity_envelope_enabled =
        o.config["native_acceleration"].asBool();
    mcc::JointPositionLimitHandle lph;
    if (hierarchy)
      ok(h.addJointPositionLimits(lp, lph));
    else
      ok(w.addJointPositionLimits(lp, lph));
    if (o.config["mode"].asString() == "ServoStep") {
      mcc::JointVelocityLimitConfig lv;
      lv.enforcement = lp.enforcement;
      mcc::JointVelocityLimitHandle lvh;
      if (hierarchy)
        ok(h.addJointVelocityLimits(lv, lvh));
      else
        ok(w.addJointVelocityLimits(lv, lvh));
      if (o.config["native_acceleration"].asBool()) {
        mcc::JointAccelerationLimitConfig la;
        la.enforcement = lp.enforcement;
        mcc::JointAccelerationLimitHandle lah;
        if (hierarchy)
          ok(h.addJointAccelerationLimits(la, lah));
        else
          ok(w.addJointAccelerationLimits(la, lah));
      }
    }
    if (hierarchy)
      ok(h.finalize(hqp));
    else
      ok(w.finalize(weighted));
  }
  Json::Value solveAnalytic() {
    auto a = o.input["analytic"];
    Json::Value out;
    out["record_type"] = "analytic";
    out["problem"] = a;
    if (placo) {
      placo::problem::Problem p;
      p.regularization = o.config["regularization"].asDouble();
      p.use_sparsity = false;
      p.rewrite_equalities = true;
      auto &x = p.add_variable(2);
      placo::problem::Expression e;
      e.A = Eigen::Matrix2d::Identity();
      e.b = -vector(a["lower"]);
      p.add_constraint(e >= 0);
      e.A = -Eigen::Matrix2d::Identity();
      e.b = vector(a["upper"]);
      p.add_constraint(e >= 0);
      int index = 0;
      for (auto task : a["tasks"]) {
        if (o.config["primary_only"].asBool() && index++ > 0)
          break;
        if (!task.get("enabled", true).asBool())
          continue;
        e.A = matrix(task["A"]);
        e.b = -vector(task["b"]);
        p.add_constraint(e == 0).configure("soft",
                                           task.get("weight", 1.).asDouble());
      }
      p.solve();
      out["candidate"] = json(x.value);
      out["accepted"] = true;
      out["status"] = "PlaCo Problem.solve returned";
      out["native_qp_status"] = Json::nullValue;
      out["native_qp_status_reason"] = "throw-on-failure API";
      return out;
    }
    mcc::QpSolverConfig c;
    c.regularization = o.config["regularization"].asDouble();
    mcc::OptimizationProblem w;
    mcc::HierarchicalOptimizationProblem h;
    mcc::VariableBlockHandle x;
    if (hierarchy) {
      ok(h.configure(c));
      ok(h.addVariableBlock("x", 2, x));
    } else {
      ok(w.configure(c));
      ok(w.addVariableBlock("x", 2, x));
    }
    mcc::VariableBoundsRequirement bounds;
    bounds.name = "box";
    bounds.block = x;
    bounds.bounds.lower = vector(a["lower"]);
    bounds.bounds.upper = vector(a["upper"]);
    mcc::RequirementHandle bh;
    if (!hierarchy)
      ok(w.addVariableBounds(bounds, bh));
    int k = 0;
    for (auto task : a["tasks"]) {
      if (o.config["primary_only"].asBool() && k > 0)
        break;
      mcc::LinearRequirement r;
      r.name = "task-" + std::to_string(k++);
      r.unit = "unit";
      r.A = matrix(task["A"]);
      r.offset = Eigen::VectorXd::Zero(r.A.rows());
      r.bounds.lower = vector(task["b"]);
      r.bounds.upper = r.bounds.lower;
      r.enabled = task.get("enabled", true).asBool();
      r.enforcement =
          mcc::squaredL2Penalty(task.get("weight", 1.).asDouble(), r.A.rows());
      mcc::RequirementHandle rh;
      if (hierarchy) {
        mcc::HierarchyLevelHandle l;
        ok(h.addLevel({r.name}, l));
        ok(h.addLevelRequirement(
            l, r, {Eigen::VectorXd::Constant(r.A.rows(), 1e-7)}, rh));
      } else
        ok(w.addRequirement(r, rh));
    }
    if (hierarchy) {
      // The public hierarchy API requires registered levels before shared
      // requirements.
      ok(h.addSharedVariableBounds(bounds, bh));
      ok(h.finalize());
      mcc::HierarchicalOptimizationSolution s;
      mcc::HierarchicalOptimizationDiagnostics d;
      auto st = h.solve(s, d);
      out["status"] = st.message;
      out["native_status_code"] = int(st.code);
      out["candidate"] = json(s.values);
      out["accepted"] = st.ok();
      for (auto &l : d.levels) {
        Json::Value p;
        p["name"] = l.name;
        p["native_qp_status"] = l.optimization.native_status;
        p["iterate"] = json(l.last_iterate);
        p["succeeded"] = l.succeeded;
        out["passes"].append(p);
      }
    } else {
      ok(w.finalize());
      mcc::OptimizationSolution s;
      mcc::OptimizationDiagnostics d;
      auto st = w.solve(s, d);
      out["status"] = st.message;
      out["native_status_code"] = int(st.code);
      out["candidate"] = json(s.values);
      out["accepted"] = st.ok();
      out["native_qp_status"] = d.native_status;
    }
    return out;
  }
  Json::Value run(const Json::Value &state, const Json::Value &sample,
                  int seq) {
    if (analytic)
      return solveAnalytic();
    Json::Value out;
    out["record_type"] = "attempt";
    out["input_state"] = state;
    out["targets"] = sample["targets"];
    out["q"] = state["q"];
    out["v"] = state["v"];
    out["joint_names"] = o.input["joint_names"];
    Eigen::VectorXd q = vector(state["q"]), v = vector(state["v"]);
    bool enabled = sample.get("secondary_enabled", true).asBool() &&
                   !o.config["primary_only"].asBool();
    if (placo) {
      for (int j = 0; j < q.size(); ++j)
        robot->set_joint(names[j], q[j]);
      robot->update_kinematics();
      for (int k = 0; k < 2; ++k) {
        std::string side = k == 0 ? "left" : "right";
        Eigen::Matrix3d R = matrix(sample["targets"][side]["rotation"]);
        Eigen::Vector3d target = vector(sample["targets"][side]["position"]) -
                                 R * vector(o.input["tcp_offsets"][side]);
        auto current =
            robot->get_T_world_frame(o.input["frames"][side].asString());
        double alpha =
            o.config["mode"].asString() == "ServoStep" ? gain * dt : 1.;
        pp[k]->target_world =
            current.translation() + alpha * (target - current.translation());
        Eigen::AngleAxisd delta(R * current.rotation().transpose());
        po[k]->R_world_frame =
            Eigen::AngleAxisd(alpha * delta.angle(), delta.axis())
                .toRotationMatrix() *
            current.rotation();
        if (extra) {
          auto elbow_current = robot->get_T_world_frame(side + "_arm_link4")
                                   .translation()
                                   .eval();
          pp[k + 2]->target_world =
              elbow_current +
              alpha * (vector(sample["elbows"][side]) - elbow_current);
          pp[k + 2]->configure(side + "-elbow", "soft",
                               enabled && o.config["secondary_link4"].asBool()
                                   ? o.config["secondary_weight"].asDouble()
                                   : 0.);
        }
      }
      if (extra)
        for (unsigned j = 0; j < active.size(); ++j) {
          pj->set_joint(active[j],
                        q[indices[j]] +
                            (o.config["mode"].asString() == "ServoStep"
                                 ? gain * dt
                                 : 1.) *
                                (sample["posture"][indices[j]].asDouble() -
                                 q[indices[j]]));
        }
      if (extra)
        pj->configure("posture", "soft",
                      enabled && o.config["secondary_posture"].asBool()
                          ? o.config["secondary_weight"].asDouble()
                          : 0.);
      const auto start = now();
      int iterations = o.config["mode"].asString() == "TargetSolve"
                           ? o.config["target_iterations"].asInt()
                           : 1;
      for (int it = 0; it < iterations; ++it) {
        Json::Value linearization(Json::arrayValue);
        for (const auto &name : names)
          linearization.append(robot->get_joint(name));
        out["native_task_linearization_q"] = linearization;
        ps->solve(true);
        robot->update_kinematics();
      }
      out["start_ns"] = Json::Int64(start);
      out["finish_ns"] = Json::Int64(now());
      for (unsigned j = 0; j < names.size(); ++j) {
        double result = robot->get_joint(names[j]);
        out["q"][j] = result;
        if (o.config["mode"].asString() == "ServoStep")
          out["v"][j] = (result - q[j]) / dt;
      }
      out["velocity_measurement_status"] =
          o.config["mode"].asString() == "ServoStep"
              ? "commanded"
              : "unavailable-TargetSolve-retains-input-velocity";
      out["status"] = "solve returned";
      out["native_qp_status"] = Json::nullValue;
      out["native_qp_status_reason"] =
          "PlaCo Problem throws on failure; no normalized status accessor";
      out["disposition"] = "native-returned";
      out["solution_quality"] = "native-status-unavailable";
      out["committed"] = true;
      out["requested_passes"] = 1;
      out["completed_passes"] = 1;
      out["internal_variable_count"] = ps->N;
      for (const auto &name : active)
        out["active_variable_columns"].append(robot->get_joint_v_offset(name));
      out["native_dimensions"]["variables"] = ps->problem.n_variables;
      out["native_dimensions"]["equalities"] = ps->problem.n_equalities;
      out["native_dimensions"]["inequalities"] = ps->problem.n_inequalities;
      out["native_dimensions"]["free_variables"] = ps->problem.free_variables;
      out["native_dimensions"]["determined_variables"] =
          ps->problem.determined_variables;
      out["native_dimensions"]["slack_variables"] = ps->problem.slack_variables;
      for (int k = 0; k < 2; ++k) {
        Json::Value task;
        task["A"] = jsonMatrix(pp[k]->A);
        task["b"] = json(pp[k]->b);
        task["name"] = k == 0 ? "left-position" : "right-position";
        task["decision_unit"] = "joint increment rad";
        out["native_task_rows"].append(task);
      }
      return out;
    }
    mcc::InverseKinematicsRequest request;
    request.state.joint_positions = q;
    request.state.joint_velocities = v;
    request.reference_frame_name = o.input["root_frame"].asString();
    for (int k = 0; k < 2; ++k) {
      std::string side = k == 0 ? "left" : "right";
      Eigen::Matrix3d R = matrix(sample["targets"][side]["rotation"]);
      Eigen::Vector3d target = vector(sample["targets"][side]["position"]) -
                               R * vector(o.input["tcp_offsets"][side]);
      request.position_targets.emplace_back(ph[k], target);
      request.orientation_targets.emplace_back(
          oh[k], R, !o.config["primary_only"].asBool());
      if (extra)
        request.position_targets.emplace_back(
            ph[k + 2], vector(sample["elbows"][side]),
            enabled && o.config["secondary_link4"].asBool());
    }
    if (extra) {
      Eigen::VectorXd p(active.size());
      for (unsigned j = 0; j < active.size(); ++j)
        p[j] = sample["posture"][indices[j]].asDouble();
      request.posture_targets.emplace_back(
          posture, p, enabled && o.config["secondary_posture"].asBool());
    }
    mcc::InverseKinematicsSolution sol;
    mcc::InverseKinematicsDiagnostics d;
    mcc::HierarchicalInverseKinematicsDiagnostics hd;
    auto start = now();
    auto st = hierarchy ? hqp.solveInverseKinematics(request, sol, hd)
                        : weighted.solveInverseKinematics(request, sol, d);
    auto finish = now();
    out["start_ns"] = Json::Int64(start);
    out["finish_ns"] = Json::Int64(finish);
    out["status"] = st.message;
    out["native_status_code"] = int(st.code);
    out["internal_primary_variable_count"] = int(active.size());
    bool accepted = mcc::isAccepted(sol.disposition);
    out["committed"] = accepted;
    out["disposition"] = accepted ? "accepted" : "rejected";
    out["accepted_position_tolerance"] =
        o.config["hard_tolerance"].asDouble() *
        (o.config["mode"].asString() == "ServoStep" ? dt : 1.0);
    out["native_hard_tolerance"] = o.config["hard_tolerance"];
    out["native_hard_tolerance_unit"] =
        o.config["mode"].asString() == "ServoStep"
            ? "rad/s joint velocity bound"
            : "rad joint position bound";
    out["candidate_active_q"] = json(sol.joint_positions);
    out["candidate_active_v"] = json(sol.joint_velocities);
    if (accepted)
      for (unsigned j = 0; j < indices.size(); ++j) {
        out["q"][indices[j]] = sol.joint_positions[j];
        if (sol.joint_velocities.size())
          out["v"][indices[j]] = sol.joint_velocities[j];
      }
    if (hierarchy) {
      out["native_solution_quality"] = int(hd.solution_quality);
      out["selected_priority"] = hd.selected_priority
                                     ? Json::Value(int(*hd.selected_priority))
                                     : Json::Value();
      out["highest_completed_priority"] =
          hd.highest_completed_priority
              ? Json::Value(int(*hd.highest_completed_priority))
              : Json::Value();
      out["task_scale_reference"] = json(hd.task_scale_reference);
      out["task_scale_reference_projection_max_change"] =
          hd.task_scale_reference_projection_max_change;
      for (const auto &c : hd.constraints) {
        Json::Value e;
        e["name"] = c.name;
        e["component_name"] = c.component_name;
        e["bound_source"] = int(c.bound_source);
        e["bound_side"] = int(c.bound_side);
        e["unit"] = c.unit;
        e["state"] = int(c.state);
        e["value"] = c.value;
        e["lower"] = c.lower;
        e["upper"] = c.upper;
        e["minimum_slack"] = c.minimum_slack;
        e["maximum_violation"] = c.maximum_violation;
        out["native_constraints"].append(e);
      }
      int attempted = 0, completed = 0;
      for (auto &p : hd.passes) {
        if (p.attempted)
          ++attempted;
        if (p.succeeded)
          ++completed;
        Json::Value row;
        row["pass"] = int(p.pass);
        row["attempted"] = p.attempted;
        row["succeeded"] = p.succeeded;
        row["native_qp_status"] = p.native_status;
        row["backend_status"] = int(p.backend_status);
        row["iterations"] = p.iterations;
        row["solve_time_ms"] = p.solve_time_ms;
        row["warm_start_used"] = p.warm_start_used;
        row["primal_residual"] = p.primal_residual;
        row["dual_residual"] = p.dual_residual;
        row["objective_value"] = p.objective_value;
        row["active_set_size"] = p.active_set_size;
        row["last_iterate_available"] = p.last_iterate_available;
        for (const auto &t : p.last_iterate_tasks) {
          Json::Value e;
          e["name"] = t.name;
          e["residual_at_level"] = json(t.residual_optimum);
          e["preservation_drift"] = json(t.actual_preservation_drift);
          e["residual_norm"] = t.residual_norm;
          row["last_iterate_tasks"].append(e);
        }
        for (const auto &c : p.last_iterate_constraints) {
          Json::Value e;
          e["name"] = c.name;
          e["value"] = c.value;
          e["lower"] = c.lower;
          e["upper"] = c.upper;
          e["maximum_violation"] = c.maximum_violation;
          row["last_iterate_constraints"].append(e);
        }
        out["passes"].append(row);
      }
      int declared =
          o.config["primary_only"].asBool()
              ? 1
              : (extra && enabled && o.method.find("3") != std::string::npos
                     ? 3
                     : 2);
      out["attempted_passes"] = attempted;
      out["requested_passes"] = declared;
      out["solution_quality"] =
          !accepted
              ? "rejected"
              : (hd.solution_quality ==
                         mcc::HierarchicalSolutionQuality::FeasibleSuboptimal
                     ? "feasible-suboptimal"
                     : (completed == declared ? "full" : "partial-hierarchy"));
      out["completed_passes"] = completed;
      out["native_qp_status"] = out["passes"];
      for (auto &t : hd.tasks) {
        Json::Value row;
        row["name"] = t.name;
        row["priority"] = int(t.priority);
        row["enabled"] = t.enabled;
        row["state"] = int(t.state);
        row["residual_at_level"] = json(t.residual_optimum);
        row["actual_preservation_drift"] = json(t.actual_preservation_drift);
        row["residual_norm"] = t.residual_norm;
        row["baseline_velocity"] = json(t.baseline_velocity);
        out["tasks"].append(row);
      }
      for (auto &s : hd.task_scales) {
        Json::Value row;
        row["name"] = s.name;
        row["value"] = s.weighted_progress_scale;
        row["evaluated"] = s.evaluated;
        row["drift"] = s.actual_preservation_drift;
        out["scales"].append(row);
      }
    } else {
      out["native_qp_status"] = d.optimization.native_status;
      out["solution_quality"] =
          !accepted
              ? "rejected"
              : ((o.config["mode"].asString() == "ServoStep" || d.converged)
                     ? "full"
                     : "feasible-suboptimal");
      out["requested_passes"] = 1;
      out["completed_passes"] = st.ok() ? 1 : 0;
      out["iterations"] = d.iterations;
      out["termination_reason"] = int(d.termination_reason);
      out["task_scale_reference"] = json(d.task_scale_reference);
      for (const auto &scale : d.optimization.task_scales) {
        Json::Value r;
        r["name"] = scale.name;
        r["evaluated"] = scale.active;
        r["value"] = scale.scale;
        r["cost"] = scale.cost;
        r["stuck"] = scale.stuck;
        r["degraded"] = scale.degraded;
        out["scales"].append(r);
      }
      out["maximum_native_hard_violation"] =
          d.optimization.maximum_hard_violation;
    }
    return out;
  }
};
Solver::Solver(const Options &o) : impl(std::make_unique<Impl>(o)) {}
Solver::~Solver() = default;
Json::Value Solver::solve(const Json::Value &a, const Json::Value &b, int n) {
  return impl->run(a, b, n);
}
} // namespace study
