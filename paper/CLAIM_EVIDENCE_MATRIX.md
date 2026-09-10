# Claim–evidence matrix

Status: `planned`. All six claim families below are hypotheses with no study evidence yet. IDs are stable interfaces for later metric, figure, table and manuscript generators.

The common definitions and acceptance thresholds come from [PROTOCOL](../docs/mcc_placo_study/PROTOCOL.md), not from a favorable plot. Paired comparisons require E06 semantic admission. Exact run IDs and hashes are populated only after actual execution, never with fabricated placeholders that resemble completed runs.

The paper presents the MCC framework. D1 (hierarchical semantics) maps to C1/C2, D2 (multirate execution) to C3/C4, and D3 (kinematic pipeline integration) to C5/C6; see [PAPER_FRAMEWORK](PAPER_FRAMEWORK.md). C1–C6 test these designs and their tradeoffs rather than requiring MCC to win every comparison. Source provenance supports architecture descriptions; empirical effects require experimental evidence. “Practical” does not add an unmeasured hardware-deployment claim.

This matrix remains a preregistration during experiment implementation. Analysis decision rows and manuscript claims are produced only after the experimental campaign is closed, with explicit missing/failed-unit status and frozen artifacts.

| ID | Proposed bounded claim / question | Required evidence | Primary / guardrail families | Visuals | Current status |
|---|---|---|---|---|---|
| C1 | Declared hierarchy preserves completed higher-priority outcomes within stated tolerances while exposing the available secondary-task tradeoff | E06/E07 → A01; independent small problems and R1 snapshots/evolution | preservation_ratio, secondary_gain / hard_violation_excess, full_quality_rate, physical tracking | F02/F03, T01 | planned |
| C2 | Under specified difficult conditions, task and scale policies produce measurable differences in feasible progress and recovery | E06/E08 → A01; classified initial feasibility and fixed recovery windows | progress, stall_duration, recovery_time / hard_violation_excess, coverage, quality | F04 | planned |
| C3 | MCC's hierarchy and implementation choices have quantifiable quality–cost tradeoffs, evaluated through internal ablations and admitted PlaCo comparisons | E06/E09 → A02; matched timing scope and verified RT environment | ik_call_time P99 / quality, hard limits, sample completeness | F05/F06, T03 | planned |
| C4 | In MCC's reference execution architecture, rate division, asynchronous coupling and additional cores have separable effects on control latency and task quality | E10 → A02; resource-matched controls, captured state/proposal times and load injections | release_to_finish P99, deadline_miss_rate / tracking, quality, hard limits, full duration | F01/F07/F08, T03 | planned |
| C5 | Some local IK effects persist, diminish or reverse under explicitly controlled Cartesian and joint planning | E11 → A03; synchronized stage records and fixed feedback model | execution tracking, target_execution_lag / raw and executed constraints, coverage | F09 | planned |
| C6 | The selected effects have a measured coverage and failure envelope in recorded motions, with held-out generalization only where unexposed sessions exist | E12 → A03; frozen candidates, session/exposure/split inventory | execution tracking, valid_completion_rate / constraints, deadlines, source and pair coverage | F10, T02/T04 | planned |

## Decision rules

- C1 supported requires independent checks within declared tolerances and a measured secondary improvement in applicable cases, with failure/partial rates visible. Preservation violations contradict that property for the affected conditions. Zero available redundancy or zero reference cost is reported separately rather than forced into a positive gain.
- C2 can support a scoped progress/recovery advantage only with guardrails intact. Valid rejection of an infeasible initial state is not an algorithm defect; unexplained solver failure is not a reachability proof. No recovery by the observation end remains censored.
- C3 names the exact backend, task size, execution mode, observation configuration and platform. A cost reduction with lower quality is a tradeoff, not unconditional superiority. If confidence/coverage is insufficient, report descriptive results without a speedup claim.
- C4 requires the intended resource/rate contrast and valid quality. Additional-core gains cannot be attributed solely to coupling. A correlation between reference age and error requires injected/controlled comparisons for a causal interpretation.
- C5 requires paired pipeline contrasts; a downstream loss is an equally valid finding. Results apply to the stated kinematic feedback model, not a physical robot.
- C6 separates historical migration, controlled comparison and unexposed holdout. Missing/contaminated holdout supports at most recorded case-study statements. Failure-conditioned means cannot describe all required recordings.

After evidence review, each claim receives supported, contradicted, mixed or inconclusive plus condition-level scope. A missing required source yields evidence-insufficient and cannot silently remove the claim from the report. No automatic requirement exists that all six hypotheses be positive.

## Required future evidence rows

Each claim instance records claim_id, condition/window, experiment and Analysis IDs, exact source run/result locators, declaration/model/config/manifest hashes, metric version and row identity, absolute effect/uncertainty, guardrail and coverage verdicts, figure/table IDs, and reviewer status.

Empirical sentences and abstract numbers must resolve through this chain. Source/configuration diagrams can be supported by pinned source rather than a measured run, but must be labeled as design documentation. Never cite E05 completion counts as C3, C4 or C6 acceptance.
