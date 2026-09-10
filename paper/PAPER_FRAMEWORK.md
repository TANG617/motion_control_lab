# Manuscript framework

Status: `planned`. This file defines an intended argument, not a finding.

## Positioning and research questions

Use **MCC: A Practical Framework for Hierarchical Whole-Body Kinematic Control**. The paper introduces MCC through the practical requirements it addresses, its concrete architecture and the evidence for its design choices. PlaCo comparisons provide context and controlled tests, while MCC ablations identify the contribution of individual choices. Do not claim invention of HQP, task-priority IK or multirate robotics middleware.

The intended contribution structure is:

| Design group | Intended system contribution | Evidence questions |
|---|---|---|
| D1 — Hierarchical task and constraint semantics | Explicit task priorities, hard constraints, scaling and result-quality semantics for composing kinematic control problems | C1/C2; E06–E08 distinguish expressiveness, numerical correctness and observed benefit |
| D2 — Multirate reference execution architecture | An application-owned execution design with explicit state/proposal freshness, consumption, failure and reset policies around the computational Core | C3/C4; E09/E10 separate solver cost, reduced work, asynchronous coupling and processor resources |
| D3 — Kinematic control pipeline integration | Explicit interfaces and execution policies connecting goals, references, IK, trajectory generation and feedback, with reproducible deployment-oriented evidence | C5/C6; E11/E12 establish scope and failure coverage for the declared kinematic model and recordings |

For each group, explain the requirement, design decision, alternative, implementation ownership, and observed tradeoff. API availability alone is not proof of scientific novelty or practical benefit. Source-grounded architecture can establish what was built; experiments establish its properties under declared conditions. This scope covers the computational library and its reference application architecture without moving scheduling responsibilities into Core.

- RQ1: How do declared priorities affect preservation of primary task outcomes and use of redundancy?
- RQ2: How do constraints, task scaling and difficult configurations affect feasibility, progress and recovery?
- RQ3: What computational costs are attributable to the solver implementation, hierarchy and backend?
- RQ4: What changes arise from reduced update rates, asynchronous coupling and additional computing resources?
- RQ5: How do local IK effects propagate through Cartesian and joint planning?
- RQ6: Which effects persist in recorded motions and, where available, genuinely unexposed sessions?

These map to C1–C6 in [CLAIM_EVIDENCE_MATRIX](CLAIM_EVIDENCE_MATRIX.md). E06 is a methodological admission check, not a claim of superiority.

## Main section order

| Section | Required content | Evidence / visuals |
|---|---|---|
| 1. Introduction and Contributions | Practical control requirements, concrete conflicts, MCC framework scope, D1–D3 and RQ1–RQ6 | C1–C6 as pending questions until evidence exists |
| 2. Related Work | Hierarchical IK, constrained QP, framework/runtime boundaries and empirical methodology | Verified sources in RELATED_WORK; no universal SOTA ranking |
| 3. Control Requirements and Problem Formulation | State, execution modes, task equations, bounds, scaling, priority tolerances and required failure semantics | D1–D3 requirements; F02; common protocol |
| 4. MCC Framework Design and Implementation | Core API and topology lifecycle; hierarchy and result semantics; app-owned multirate runtime; proposal freshness/reset; planning/feedback interfaces; build and integration provenance | D1–D3, F01 and pinned source; actual two/three-level configurations; no implied stability theorem |
| 5. Experimental Protocol and Baseline Semantics | Production-static versus controlled PlaCo; MCC ablations; admitted comparisons; inputs, tuning, independent verification, RT environment, observation and failure policy | E06, F02, T01–T03 |
| 6. Evaluation of Hierarchical Task Handling | Analytic controls, R1 redundancy, conflicts, limits, progress and recovery | D1; C1/C2, F03/F04 |
| 7. Evaluation of Computational Cost and Multirate Execution | Quality-conditioned interpretation, tail costs, scalability and separated scheduling factors | D2; C3/C4, F05–F08 |
| 8. Kinematic Pipeline Integration and Recorded-Motion Evaluation | Stage errors, pipeline interactions, per-motion outcomes, coverage and failure; what these establish about practical integration | D3; C5/C6, F09/F10, T04; no implied hardware deployment |
| 9. Limitations and Threats to Validity | Single model/platform, shared kinematics dependency, finite observation, local linearization, exposure, unavailable metrics and missing hardware | Every unsupported extension of C1–C6 |
| 10. Conclusion | Only already-supported results, observed tradeoffs and bounded implications | No new experiment, number or claim |

Appendices: full arm matrices and resolved parameters; mathematical/metric definitions and independent checks; extra negative/edge cases; complete per-session results; build/runtime provenance and reproduction steps. Manuscript length follows evidential need until a venue is chosen; avoid shrinking essential method details merely to fit an arbitrary template.

## Formal analysis boundary

Explain canonical optimization, residual preservation, finite tolerance bands, and state/time semantics using the implemented equations. State the assumptions of any equivalence argument. Demonstrations on small convex problems establish those local properties; they do not prove global nonlinear IK optimality, closed-loop stability or collision safety.

If numerical results contradict a hypothesized property, report them and localize the discrepancy. Do not retrofit a theorem or change the solver under the same method identity. Exact maximum-scale and literal zero-nullspace-drift claims require separate evidence and are not promised by this protocol.

## Writing sequence

The experiment implementation stage maintains these specifications and collects auditable evidence; it does not implement A01–A03, generate paper tooling or assemble a manuscript, including a methods-only draft. After the experimental campaign is closed with a complete status inventory, implement and run the analyses on pinned artifacts, then build and author the paper from their verified outputs. Preregistered metrics and decision rules cannot be changed opportunistically after results are seen.

Results, abstract headline numbers and conclusions are authored only from a frozen evidence profile. Missing or failed experiments remain visible and restrict the claims. Favorable, unfavorable and mixed tradeoffs receive equal treatment. A single-source English manuscript is sufficient; no duplicate Chinese manuscript is required.
