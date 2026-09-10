# Related-work register and remaining review

Status: `planned`. Initial source check: 2026-09-10. This is a bounded bibliography and reading plan, not a completed novelty search. Sources below establish context; none proves MCC's measured advantage.

## Initial primary-source register

| ID | Source and metadata | Role in this study | Review status / boundary |
|---|---|---|---|
| RW01 | Escande, Mansard, Wieber. *Hierarchical quadratic programming: Fast online humanoid-robot motion generation*. IJRR 33(7), 1006–1028, 2014. [DOI](https://doi.org/10.1177/0278364914521306); [author-uploaded paper](https://www.researchgate.net/publication/274504328_Hierarchical_quadratic_programming_Fast_online_humanoid-robot_motion_generation) | Prior strict-priority least-squares/QP work; mathematical context for C1/C2 | Bibliography/abstract checked; relevant full derivations must be read before asserting equivalence or novelty |
| RW02 | OpenSoT journal article, 2025, IEEE Robotics & Automation Magazine 32(1), 24–34. [DOI](https://doi.org/10.1109/MRA.2024.3487395); [official repository and citation record](https://github.com/ADVRHumanoids/OpenSoT) | Existing constrained hierarchical robot-control software; task/framework context | Official citation metadata and project scope checked; background only, no measured OpenSoT arm |
| RW03 | Duclusaud, Passault, Padois, Ly. *PlaCo: a QP-based robot planning and control framework*. arXiv:2511.06141, submitted 8 November 2025. [Paper and publication metadata](https://arxiv.org/abs/2511.06141); [full text](https://arxiv.org/html/2511.06141v1); [official repository and citation](https://github.com/Rhoban/placo); [official kinematics introduction](https://placo.readthedocs.io/en/stable/kinematics/getting_started.html); [local source provenance](../third_party/placo/MOTION_CONTROL_LAB.md) | Framework contribution, baseline API context and pinned implementation scope | On 2026-09-10, the checked arXiv record and official citation identify a preprint; peer-reviewed conference/journal publication was not verified. Metadata, introduction, architecture and application overview checked; detailed mathematical comparison remains pending. Local v0.9.23 plus recorded integration changes is the measured object, not every capability described by the paper or current online docs |
| RW04 | Laurenzi, Antonucci, Tsagarakis, Muratore. *The XBot2 real-time middleware for robotics*. Robotics and Autonomous Systems 163, 104379, 2023. [DOI](https://doi.org/10.1016/j.robot.2023.104379); [official documentation and citation record](https://advrhumanoids.github.io/xbot2/master/index.html) | Existing runtime/component separation and robotics middleware context for C4 | Official metadata/features checked; not a numerical baseline or proof of equivalent coupling semantics |

OpenSoT already addresses hierarchically organized robot tasks under constraints; MCC must be positioned through its specific interfaces, semantics and measured tradeoffs rather than the existence of hierarchy itself. The official XBot2 material describes a robotics runtime with RT/non-RT plugins and communication facilities; threading or component communication alone is not a new contribution. These observations follow the linked project sources and do not replace full paper comparison.

PlaCo's documented soft-task configuration is a baseline feature, not evidence that it lacks constrained secondary optimization. The actual hard/soft/scaled behavior and variable representation must be audited in the pinned C++ source. Backend-specific numerical premises are disclosed in the experimental parity table.

RW03 is a framework paper, with contributions covering QP formulation abstraction, task/constraint composition, Python prototyping and C++ execution. Its application section presents quadruped balancing, kinematic loop closure and a differential joint. These are related-work claims and examples, not measurements transferred into this study. The initial register omitted this paper despite its citation in the vendored README; the omission is corrected here. A public preprint must not be described as confirmed peer-reviewed publication without a verifiable venue record.

## Required remaining reading before results submission

1. Read the relevant sections of RW01 and distinguish exact lexicographic objectives, residual preservation, finite tolerance bands, rank deficiency and inequality handling from the implemented MCC policies.
2. Read RW02 and its cited implementation lineage for task/constraint composition and solver/runtime boundaries. Identify similarities and concrete differences without adding a third experimental baseline.
3. Read RW03 in full, especially Sections III–V, and compare its formulation, priority semantics, dimensionality reduction and evaluation scope with the pinned baseline and MCC. Separate paper descriptions, current upstream capabilities and the measured local implementation; recheck publication status before submission.
4. Review primary work on asynchronous/multirate robot control and stale reference/state effects. RW04 is runtime context only; it does not establish a novel or equivalent control-coupling algorithm.
5. Register the exact versions and primary references for the used kinematics, QP and trajectory-generation dependencies once build provenance is frozen. Do not choose references merely because a package name is familiar.
6. Document how prior empirical studies control quality, constraints, hardware and timing scope. Never import a speedup number measured on a different robot or platform as a direct comparator.

For each completed reading, retain citation metadata, the supported statement, the relevant section/page, the distinction from MCC and whether the source was fully accessible. Unread or inaccessible details remain open review items rather than invented summaries.

## Attribution policy

The intended contribution is MCC as a practical framework for hierarchical whole-body kinematic control, with explicit computational-library and reference-runtime boundaries. Explain concrete design decisions and substantiate their properties through reproducible evaluation, including controlled PlaCo comparisons and MCC ablations. Practical usability alone does not establish novelty; compare the specific design with existing frameworks. New theoretical claims are not required and cannot be manufactured to satisfy a submission narrative. If the evidence supports only a bounded engineering case study, the manuscript must say so.

The bibliography and remaining reading requirements are preserved as research constraints. Full related-work authoring and manuscript integration are deferred with the paper stage until after the experimental campaign; baseline-source inspection needed to implement E06 remains part of experiment development.
