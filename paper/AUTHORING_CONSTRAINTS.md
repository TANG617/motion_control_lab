# Authoring and evidence constraints

Status: `planned`. Applies to future manuscript text, captions, generated tables, abstract, HTML reports and supplementary material.

## Terminology and claim boundaries

| Use | Required meaning / limitation |
|---|---|
| MCC framework | The computational library plus the explicitly described reference application architecture; identify which component owns each capability |
| practical | A design goal supported to the extent of integration, execution and failure evidence; not automatic proof of hardware deployment or universal usability |
| kinematic whole-body IK | Multiple kinematic objectives on the stated R1 model; no torque/contact-stability claim |
| priority preservation | Same-linearization residual/scale preservation within declared tolerances |
| null-space behavior | A scoped redundancy/preservation interpretation; not automatically an exact projector or zero nonlinear drift |
| weighted progress | The implemented finite-weight objective; not exact maximum feasible scale |
| full hierarchy | Every declared semantic level completed and verified, distinct from feasible-suboptimal and fallback |
| measured timing | Observed distribution under pinned conditions; not WCET or universal hard-real-time assurance |
| recorded case study | Evidence from available recordings, without implying unseen-session generalization |
| holdout | Sessions unexposed to development and tuning, with frozen configuration and split provenance |

Attribute algorithm formulation, backend, task policy, scheduler, processor resources and downstream planning separately. A faster pipeline does not establish that every MCC component is faster. A favorable MCC-versus-PlaCo experiment does not rank MCC above OpenSoT or other unmeasured systems.

The manuscript introduces MCC and evaluates its design decisions. Comparisons and ablations support this argument; they do not replace the explanation of the framework or require uniformly favorable results. Separate existing methodological ideas from specific MCC design contributions. Hardware deployment statements require a separately scoped physical closed-loop evaluation with matching build/runtime evidence.

## Source and numeric discipline

Use original papers, authors' repositories and official documentation for technical citations. Register exact bibliographic metadata and which proposition each source supports. Read the relevant primary material before comparing algorithms or claiming novelty; an abstract or README alone is not enough for a detailed mathematical comparison.

Every result number must be generated from a pinned Analysis row. A caption identifies condition, window, sample/statistical unit, uncertainty, failure/coverage policy and source. Draft author/venue fields remain explicitly unset; do not invent identities or publication permissions.

Do not treat independent recomputation using the same model library as completely independent modeling evidence. Disclose common URDF/kinematics assumptions, local linearization, constraint classification, tuning budget, finite windows, runtime resource and observation effects.

## Positive and negative evidence

Give unfavorable, mixed, missing and incomplete cases the same visibility as favorable cases. Report conditional-on-completion effects together with all-required coverage. Explain failed hypotheses directly; do not silently change metrics, scale axes, input filtering or narrative scope to preserve a success claim.

No manuscript conclusion can be stronger than its corresponding C1–C6 record. No hardware improvement, safety barrier, closed-loop stability, global optimality or universal platform claim can be inferred from this study. A new such claim requires a separately declared method and evidence scope.

## Draft and completeness checks

The experiment code stage does not implement Analysis or paper-generation tools and does not build a manuscript, even a methods-only draft. Keep protocols and this authoring specification available as constraints. After the experimental campaign is closed and its artifacts are frozen, implement A01–A03 and then the paper tools and manuscript. Any subsequent draft with missing evidence must disclose that status; empirical curves, measured numbers and result prose require genuine evidence.

The later evidence-complete check must fail on missing/changed hashes, unverified comparison conditions, contaminated holdout presented as unseen, missing formal environment proof, placeholder figures, manual empirical numbers, unsupported claims or inconsistent C/F/T references. Draft build success is not evidence completeness or submission readiness.

Author/venue/data-publication choices remain separate from code completion. Do not publish, push, package private recordings for public distribution or promote results automatically. Human scientific review follows the existing Lab result lifecycle.
