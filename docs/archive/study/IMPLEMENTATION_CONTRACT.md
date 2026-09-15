# Frozen implementation interface, Wave 0

> 历史归档（2026-09-14）。本文保留当时的设计、命令和证据范围，不是当前运行指南。现行入口见 [文档索引](../../README.md)。

> 历史 study 实现合同/验收记录，保留原命令和结论范围。现役程序与声明已迁移，使用 [公开 app 执行接口](../../../tools/app_execution/README.md)，不要直接执行下述历史构建/启动命令。

Version: `study-request.v1` / `experiment.v2` / `run_manifest.v2` / `metric_row.v2`.
This implements experiments only. Analysis identity is reserved for a later schema extension;
no Analysis writer, runner, statistics, figure or paper code is provided.

## Ownership

Parent owns `tools/mcc_placo_study/`, `contracts/`, top-level CMake, architecture registry,
shared documentation and integration. Existing baseline and all production/Core semantics remain unchanged.
A owns E06/E07/E08 experiment directories and `apps/study_e06/`, `study_e07/`, `study_e08/`.
B owns E09/E10 experiment directories and `apps/study_e09/`, `study_e10/`.
C owns E11/E12 experiment directories and `apps/study_e11/`, `study_e12/`.
Each owner supplies its own CMakeLists, options, solver, loop, short main, and planning only if used.
No app-to-app includes or links. Duplication of app business code is intentional.
Owners may read public modules but must request public changes from parent.

Initial complete app inventory: baseline, cartesian_planning, hierarchical_inverse_dynamics_torque_sim,
hierarchical_kinematics_step, plot_core_planning, psibot_teleop, replay_plan, single_arm_step, step, target.
No direct app source files at apps root; only AGENTS.md. Architecture test is the ownership registry.

## Public Python API and executable contract

Add Lab `tools/mcc_placo_study` to sys.path. `inputs.canonical_snapshot(model,seed,perturbation,fixture)`
returns independent URDF FK targets, 20 production-ordered full joints and 16 active joints.
`inputs.save_input(path,data)` writes canonical JSON and returns absolute descriptor filename.
`evidence.write_json`, `artifact`, `stable_hash`, `validate_definition` are mechanical helpers.
`metrics.py` is the authoritative pure scalar measurement implementation. App-specific independent
verification stays under experiment directories and may use these functions. No root Python imports.

Input `canonical_input.v1`: model locator/sha256/size, joint_names, active_joint_names,
root_frame, frames{left,right}, tcp_offsets{left,right}, limits{lower,upper,velocity},
initial_state{q,v}, samples[{sequence,source_time_s,q,v,targets{left,right:{position[3],rotation[3][3]}}}].
All q/v arrays follow joint_names; non-study URDF joints are fixed at neutral zero in independent FK.
Additional explicit task, event, pipeline, raw source and derivative fields are allowed.
Descriptor `input_descriptor.v1` has canonical artifact, input_id, fixture, session_id, split_id, model.
Hash binds bytes, so rewrite a descriptor explicitly after intentional input regeneration.

Definition `experiment.v2` requires experiment_id, question, units, failure_policy, metrics,
evaluation_windows, controlled_factors. Optional repeats maps development/pilot/confirmatory counts.
Each unit: case_id, method_id, arm_id (safe [A-Za-z0-9_.-]+), input (real descriptor reference),
config (ALL resolved algorithm settings), required, binary (`mcl_study_eNN`),
smoke_config (explicit bounded override), optional phase_config, repeats, available/unavailable_reason.
No hidden default selection: generate every README required family/factor, including unavailable cells.
A unit can instead provide `command:[...]`, replacing `{request}`, `{output_dir}`, `{input_path}`.
Use this only for independent preexisting executables or labelled fixtures, never cross-app imports.

App CLI: `mcl_study_eNN --request /absolute/request.json`; support `--help`.
Request fields: schema_version=`study_request.v1`, identity, phase, smoke, config,
input (fully loaded canonical JSON), input_descriptor, output_dir (existing empty unit directory
apart from orchestrator metadata). App reads JsonCpp directly; app owns parsing and algorithm semantics.
identity includes run/experiment/case/method/arm/repeat/session/split/window/component and input/model/config hashes.
App writes `raw.jsonl`, flushes each attempt so a crash retains evidence; never writes orchestrator status.json.
App writes native_status.json, resolved_config.json and experiment-specific evidence files as appropriate.

Raw attempt fields: record_type=`attempt`, attempt_sequence,input_sequence,state_sequence,task_revision,
q (full ordered resulting state),v, joint_names,targets (actual reference at this attempt), status (native operation),
native_qp_status, disposition,solution_quality,selected_priority,highest_completed_priority,
requested_passes,completed_passes,committed,committed_sequence, execution_state,
release_ns,start_ns,finish_ns,proposal_revision,proposal_created_ns,proposal_state_capture_ns.
Use null plus a reason for absent measurements; no synthetic status/zero timing. Analytic problems
use record_type=`analytic` and retain A,b,bounds, candidate and reference/check fields.
`accepted_position_tolerance` must reflect actual native frozen contract, never a new epsilon.
Add per-stage snapshots/timing/workload/scale equations/pass references and provenance as experiment needs.

Parent verifier produces independent_checks.jsonl and validation.json from q and canonical URDF,
using independent XML tree FK, never candidate diagnostics. App-specific validators add their own
files; never overwrite raw native status. Failed original attempts remain available for later consumption.

CLI: `python3 tools/mcc_placo_study/study.py --experiment E06 --method ... --case ... --repeat 0
--phase development --smoke --timeout 60 --output-root ...`. Default installed path is
`${MCL_INSTALL_PREFIX:-/workspace/install/algorithm}/bin/<binary>`; MCL_BINARY has highest precedence.
`--dry-run --inventory-output <path>` enumerates full selection including missing/unavailable inputs.
`--preflight` is read-only and returns nonzero for required missing prerequisites. Formal phases require
RT and freeze/threshold/host attestation; never run them in this implementation task.

Wave 0 gate: five tests passed 2026-09-10: seed, independent known rotation/FK, scalar/window/deadline,
v1 compatibility/hash tampering, labelled fixture subprocess success/failure/continue/timeout retaining
all raw evidence and independent FK checks. This is contract validation, not a real solver smoke.

Compatible parent additions: units may declare `validation_command` with the same request/output/input
substitutions; it runs after native process exit outside timing and has separate logs/status. Command
adapters may combine binary plus command and use `{binary}` to preserve native runtime fingerprints.
Formal freeze validation lives in tools/mcc_placo_study/freeze.py; it pins immutable artifacts and
requires explicit current-boot host attestation. Boolean freeze flags cannot admit formal execution.

Final additive evidence metadata: `experiment.v2.metric_roles` declares scalar metric roles; it is copied into the request and passed to the independent verifier, never to candidate algorithms. Unknown roles default to diagnostic only for declarations without explicit role metadata. `relabel_metrics.py` produces new hash-bound role-only correction files for development artifacts; source bytes, values, identities and raw locators stay unchanged. Missing bounded smoke configuration is preflight unavailable; empty result evidence cannot pass a required batch. Unavailable/not-run units retain resolved declaration and status artifact hashes.
