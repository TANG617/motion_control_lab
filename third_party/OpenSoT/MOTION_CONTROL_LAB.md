# Motion Control Lab source provenance

- Upstream: https://github.com/ADVRHumanoids/OpenSoT
- Branch: `ros2`
- Upstream commit: `f1d8c733bcff49ed5b213191ac85a0629bb97d43`
- Upstream project version: `4.0.1`
- Imported: 2026-08-14 with `git archive`; this directory contains no nested Git repository.
- Vendored source SHA-256: `5c0f51af072260e154f506d4d06ff82786a4b3a1c027b82e65e44dfa958f79b8`

The digest is computed over the sorted relative paths and SHA-256 values of
all files in this directory except this provenance note, so the recorded value
is stable and covers the upstream snapshot plus every local source patch.

The full upstream source is retained. Motion Control Lab carries these narrow integration changes:

1. documentation and Python bindings are controlled by options and disabled by default;
2. `tf2_eigen_kdl` is linked for the always-built Cartesian conversion source;
3. `mcl_bridge/` exposes the native Cartesian/Postural/limits/iHQP(OSQP) path through a C ABI so ROS2, XBot2, OpenSoT headers, and C++20 do not enter Lab targets.

The bridge is opt-in through `MCL_BUILD_E04_OPENSOT_SMOKE`. OpenSoT is built as an isolated external project with collision support and the upstream default available solver backends. Its non-ROS source dependencies are installed in the dedicated system prefix `/opt/motion-control-lab/opensot-deps` at these tested revisions:

- `matlogger2`: `b44688e25ae470e4d445cb07e451bfd497b0b8fb`
- `xbot2_interface`: `36fd44b7e1dec328bca289c51a40e04f26ad4d21`

After installing XBot2, `libxbot2_interface`,
`libxbot2_interface_collision`, and `libmodelinterface2_pin` retain `$ORIGIN` and add
`/opt/openrobots/lib`, `/opt/ros/jazzy/lib`, and
`/opt/ros/jazzy/lib/x86_64-linux-gnu` to their RUNPATH. This keeps the pinned
prefix relocatable while resolving the collision library's private Coal and
Jazzy dependencies without exporting those paths through Lab targets.

The ROS baseline is `/opt/ros/jazzy`. These dependency prefixes are passed only to the nested OpenSoT build.
