# Figure and table contract

Status: `planned`. The following are preregistered figure specifications, not populated results. Figure/table generation and publication rendering code are deferred until the experimental campaign is closed and A01–A03 are implemented. Experiment code must capture their required source fields now; it does not generate these paper artifacts.

## Main figures

| ID and English title | Source / owner | Required panels and comparisons | Caption must disclose |
|---|---|---|---|
| F01 — MCC framework, reference execution architecture, and evidence flow | Pinned configuration/source; A02 | D1–D3 overview; Core/app boundary; target/feedback/control/secondary clocks; proposal mailbox; goal-to-execution interfaces; independent evidence path | Design diagram, actual declared topology and component ownership, no measured performance or hardware deployment implied |
| F02 — Admitted task and constraint comparisons | E06/E07; A01 | Joint/frame mapping, baseline versus controlled arms, hard/soft/scaled and layer layouts, analytic/R1 scenes | Allowed differences, missing capabilities, internal problem dimensions |
| F03 — Priority preservation and secondary-task benefit | E07; A01 | Drift with tolerance band, paired secondary gain, 2/3-layer comparison, weight sweep and task-transition trace | Task units, completed versus partial hierarchy, sample/pair counts and zero-denominator conditions |
| F04 — Progress and recovery near difficult configurations | E08; A01 | Bounds/scale/progress over fixed windows; singular/limit buckets; recovery and failure matrix | Initial feasibility, local reachability scope, censored recovery, soft collision semantics |
| F05 — Complete IK cost and latency tails | E09; A02 | Distribution/tail plot, cold versus steady, same-scope cost decomposition, quality proportions | Backend, mode, sample count, raw quantile or histogram upper bound, observation profile |
| F06 — Cost across workload and hierarchy size | E09; A02 | Measured cost versus active task/constraint size and hierarchy layers | Actual variables/rows, uncertainty across process repeats, unavailable backends |
| F07 — Worker execution and proposal consumption | E10; A02 | Gantt-style planned release/start/finish, proposal capture/creation/consumption, pauses and missed/skipped releases | Fixed displayed window, true measured schedule versus virtual diagnostic schedule |
| F08 — Resource, freshness, and control-quality tradeoffs | E10; A02 | Same-core and two-core paired contrasts; secondary rate/age versus tracking; total CPU cost | Resource budget, feedback age versus proposal age, load and observation settings |
| F09 — Error propagation through the control pipeline | E11; A03 | Goal/reference/IK/execution traces; stage errors; direct/cartesian/joint/full paired effects; projection markers | Frames, feedback/plant model, native versus derived derivatives, no post-hoc lag correction |
| F10 — Recorded-motion effects, coverage, and failures | E12; A03 | Per-motion paired effects, session intervals, all-required failure/coverage matrix | Session/exposure/split, conditional denominators, holdout eligibility and unsupported generalization |

## Tables

| ID and title | Required contents |
|---|---|
| T01 — Methods, semantics, and admitted comparisons | Pinned implementation; execution mode; enforcement/scale/layers; active DOFs; constraints; defaults versus controlled tuning; unavailable and mismatch reasons |
| T02 — Dataset provenance and evaluation coverage | Raw/canonical hashes; sessions/actions; durations; exposure/splits; motion envelope; missing/invalid input and required-arm coverage |
| T03 — Runtime environment and measurement conditions | Workstation/CPU/core type and allocation; RT kernel/boot; scheduling; compiler/dependencies/binaries; observation; windows/repetitions and time scopes |
| T04 — Effects, uncertainty, guardrails, and scope | C1–C6 main effects with units/intervals; full-quality/coverage/failure counts; condition scope and supported/negative/insufficient verdicts |

## Artifact and visual requirements

PDF and SVG are primary scientific figure outputs; PNG is the preview. Use Matplotlib for these artifacts, keeping scientific calculation in the metric/Analysis pipeline. HTML and Foxglove are interactive views of the same evidence, not separate sources of truth.

Assign consistent method colors, pair them with distinct line styles/markers, and preserve the mapping in all figures. Labels and units remain legible at final column width and in grayscale. Never encode success solely by color. Include tolerance/deadline lines where meaningful; disclose log axes and clipping. Display rules are fixed before confirmatory results.

Plot simplification may downsample dense display traces using a declared extrema-preserving policy; metrics, threshold exceedances and tail quantiles always use complete valid evidence. Avoid smoothing away misses or projection events. Same comparison panels use the same scale unless an explicit axis label explains otherwise.

Every generated artifact has a manifest linking F/T ID, Analysis run/hash, metric rows, source cases/windows, renderer version and display settings. Table cells and figure annotations are generated from those rows; error bars state the statistical unit. No handwritten improvement percentages or selected best-repeat plots.

Choose representative timeline cases by predeclared input/case ID or input-only rule. An automatically chosen worst measured case may be shown as a labeled diagnostic, accompanied by the complete aggregate; it must not replace the main evaluation window.

Missing evidence is displayed in coverage/validation tables. Do not draw blank empirical panels or synthetic smooth curves as placeholders for unrun experiments. In a methods-only draft, list the planned figure and its pending evidence outside a results presentation.
