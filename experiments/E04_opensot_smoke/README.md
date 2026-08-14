# E04：OpenSoT ROS2 R1 Smoke

E04 exercises the vendored OpenSoT 4.0.1 ROS2 source through a private C ABI bridge. The bridge owns XBot2 model loading and the native OpenSoT Cartesian, Postural, joint/velocity limits, iHQP, and OSQP objects. No ROS2 or OpenSoT C++ header is included by the Lab executable.

E01 and E04 both use the hard-coded `/workspace/models/r1.cos.urdf`, the same named initial/goal joint maps, and the same `left_arm_ee_link` position-only target derived by forward kinematics. Solver-native regularization and step policy are the only declared differences.

Build and run:

```bash
source /opt/ros/jazzy/setup.bash
cmake -S . -B build/opensot -DMCL_BUILD_E04_OPENSOT_SMOKE=ON
cmake --build build/opensot --target e04_opensot_smoke -j8
LD_LIBRARY_PATH="build/opensot/third_party/opensot/install/lib:/opt/motion-control-lab/opensot-deps/lib:${LD_LIBRARY_PATH}" \
  build/opensot/e04_opensot_smoke
```

The default Lab configuration keeps `MCL_BUILD_E04_OPENSOT_SMOKE=OFF`, so configuring or building ordinary Lab targets does not discover or compile ROS2, XBot2, MoveIt, or OpenSoT.

Pinned non-ROS dependencies are installed under
`/opt/motion-control-lab/opensot-deps`; exact revisions and the vendored source
fingerprint are documented in
[`MOTION_CONTROL_LAB.md`](../../third_party/OpenSoT/MOTION_CONTROL_LAB.md) and
recorded in every run manifest. Evidence follows the same bundle shape as E01,
under `arms/opensot_v4_0_1_ros2_osqp/psi_r1_cos/`.
