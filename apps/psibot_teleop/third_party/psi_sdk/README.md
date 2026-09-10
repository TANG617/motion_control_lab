# PSI SDK snapshot for psibot_teleop

This is a read-only delivery snapshot copied from
`products/synrobot/modules/control/psi_cortex/3rdparty/psi_sdk`. It is consumed as
paired public headers and architecture-specific shared libraries; no SDK source
is compiled by the Lab.

The x86_64 build is SDK 0.1.0 from commit
`4e70d7e8984df95ef89a6a2a0bd8b910510f5610` (Ubuntu 24.04 / GCC 13 / ROS Jazzy).
See `lib/x86_64/BUILD_INFO.json`. Its headers match this snapshot; SDK HEAD
68e8365 changed the hand API ABI and must not be mixed with these libraries.
The aarch64 artifacts are preserved from the existing package; there is no
verified source/build manifest for that architecture. ARM execution is untested.

`SHA256SUMS` records regular payload files; `libpsi_sdk.so -> libpsi_sdk.so.0 ->
libpsi_sdk.so.0.1.0` symlinks are preserved. Every architecture also carries its
matching `libinex_client.so`. The current libraries require system ROS runtime
libraries. This copy does not remove that dependency.

The original package had no SDK/Inex license file. The available bundled spdlog
license is preserved under `licenses/`; no redistribution permission is inferred.
Update this directory as a complete compatible SDK delivery, retaining source
records, licenses, symlinks and checksums. Never replace headers or a single .so
independently.
