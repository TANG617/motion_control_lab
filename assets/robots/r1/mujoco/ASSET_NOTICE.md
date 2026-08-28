# PSI R1 MuJoCo asset notice

The R1 URDF and mesh files in this directory are derived from the PSI R1 robot-description
asset set at `products/synrobot/modules/common/robot_description/psi_r1`.

The repository owner confirmed that these robot assets may be used and redistributed in this
research repository. They remain PSI Robotics robot-description assets and are not covered by
the source-code license of Motion Control Lab. Do not redistribute them outside an authorized
PSI Robotics repository without confirming the applicable asset terms.

`mjcf/r1.xml` is generated from MuJoCo 3.11.0's compiled URDF output and curated by
`prepare_asset.py`. Runtime loading does not invoke ROS, xacro, or the production repository.
