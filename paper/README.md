# MCC framework paper workspace

Status: `planned`. There is no manuscript result section, generated figure, measurement, or submission package in this directory yet.

Working title: **MCC: A Practical Framework for Hierarchical Whole-Body Kinematic Control**.

The target is an English systems manuscript presenting MCC as a practical framework for hierarchical whole-body kinematic control. The central argument connects concrete control requirements to framework design, implementation and reproducible validation. Controlled PlaCo comparisons and MCC ablations test the value and cost of those design choices; a universal ranking is not the contribution.

The framework description distinguishes the MCC computational library from the reference execution architecture built around it. Core owns computation; the application owns scheduling, coupling and execution policy. Three intended contribution groups cover hierarchical task/constraint semantics, multirate execution, and integration through the kinematic control pipeline. Their evidence maps to the existing C1–C6 and E06–E12 IDs.

“Practical” is a property to substantiate, not a claim that deployment has already been validated. The current experiments cover the stated R1 kinematic whole-body IK configuration and workstation. They do not establish hardware deployment, torque/contact control, stability or universal robot support. Hardware deployment claims require a separately scoped closed-loop study with matching runtime provenance. A venue, author list, publication license and public dataset policy remain unset. OpenSoT is contextual literature rather than a measured third baseline.

## Authoritative documents

- [Paper framework](PAPER_FRAMEWORK.md): narrative, research questions, sections and appendices.
- [Claim–evidence matrix](CLAIM_EVIDENCE_MATRIX.md): six pending claim families and their decision rules.
- [Figure and table plan](FIGURE_TABLE_PLAN.md): F01–F10 and T01–T04, data sources and visual acceptance.
- [Authoring constraints](AUTHORING_CONSTRAINTS.md): language, attribution, evidence and missing-result rules.
- [Related-work register](RELATED_WORK.md): initial verified sources and remaining literature review work.
- [Study protocol](../docs/mcc_placo_study/PROTOCOL.md) and [data contract](../docs/mcc_placo_study/DATA_ANALYSIS.md): sole authorities for mathematical metrics, pairing and statistics.

## Planned evidence and build lifecycle

Implement E06–E12 and validate evidence capture → complete the declared experimental campaign → implement and run A01/A02/A03 on frozen artifacts → build the paper evidence index and figure/table tools → author the manuscript → scientific review → publication preparation.

The current implementation prompt covers experiments and their evidence infrastructure only. A01–A03 code, cross-run statistical analysis, publication figures, paper-build facilities and manuscript drafting are deferred until the experimental campaign is closed and its artifacts are frozen. Protocols, metric definitions, claim IDs and figure specifications remain preregistered now. Campaign closure records every required unit, including failures and unavailable inputs; it does not require positive results or imply evidence completeness.

At that later stage, provide separate draft and evidence-complete checks. A draft may explicitly disclose missing evidence; an evidence-complete check must fail on missing sources, unsupported claims or handwritten result numbers. Generated files must be tied to source and analysis hashes, never implicit latest directories.

Formal timing and real holdout execution are deferred until the workstation is rebooted into and verified under its RT kernel, followed by a separately initiated confirmatory run. Code readiness, run readiness and manuscript evidence completeness remain distinct statuses.

This stage creates only the documentation framework. Reusable authoring ideas from the neighboring OTG Lab paper are architectural references, not transferable scientific evidence or mandatory venue rules.
