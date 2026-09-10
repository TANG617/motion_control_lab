#!/usr/bin/env python3
"""Static SDK snapshot check; never loads a client or connects to any device."""
import hashlib
import pathlib
import subprocess
import sys
import tempfile

root = pathlib.Path(sys.argv[1])
for row in (root / 'SHA256SUMS').read_text().splitlines():
    digest, relative = row.split('  ', 1)
    assert hashlib.sha256((root / relative).read_bytes()).hexdigest() == digest, relative
for arch, machine in [('x86_64', 'Advanced Micro Devices X86-64'), ('aarch64', 'AArch64')]:
    directory = root / 'lib' / arch
    assert (directory / 'libpsi_sdk.so').is_symlink()
    assert (directory / 'libpsi_sdk.so').resolve().parent == directory.resolve()
    for name in ['libpsi_sdk.so', 'libinex_client.so']:
        header = subprocess.check_output(['readelf', '-h', directory / name], text=True)
        assert machine in header, (arch, name)
    symbols = subprocess.check_output(['nm', '-D', '-C', directory / 'libpsi_sdk.so'], text=True)
    for name in ['moveEndPose(', 'moveWbcPose(', 'startWbc(', 'stopWbc(', 'getJointStates(',
                 'getEndPose(', 'setEnable(', 'getEnabled(', 'setProfile(', 'getProfile(', 'disconnect(',
                 'getRobotLimits(', 'setControlBackend(', 'isConnected(']:
        assert 'psibot_robot_sdk::Robot::' + name in symbols, (arch, name)

# Configure the actual app-local CMake against interface-only Lab dependencies.
# This checks package override and target selection without compiling either SDK.
with tempfile.TemporaryDirectory(prefix='psibot-sdk-cmake-') as temporary:
    source = pathlib.Path(temporary)
    app = pathlib.Path(__file__).resolve().parents[1]
    (source / 'CMakeLists.txt').write_text('''cmake_minimum_required(VERSION 3.20)
project(sdk_selection LANGUAGES CXX)
include(GNUInstallDirs)
set(CMAKE_SYSTEM_PROCESSOR "${TEST_PROCESSOR}")
set(BUILD_TESTING OFF)
foreach(name keyboard_teleop cartesian_teleop terminal_frontend tui scheduler)
  add_library(motion_control_lab::${name} INTERFACE IMPORTED)
endforeach()
add_library(jsoncpp_lib INTERFACE)
function(mcl_add_app)
  cmake_parse_arguments(APP "" "TARGET" "SOURCES;LIBRARIES" ${ARGN})
  add_executable(${APP_TARGET} ${APP_SOURCES})
  target_link_libraries(${APP_TARGET} PRIVATE ${APP_LIBRARIES})
  get_target_property(selected mcl_psibot_sdk IMPORTED_LOCATION)
  file(WRITE "${CMAKE_BINARY_DIR}/selected.txt" "${selected}")
endfunction()
add_subdirectory("${TELEOP_APP}" teleop)
''')
    for processor, expected in [('x86_64', 'x86_64'), ('aarch64', 'aarch64'), ('arm64', 'aarch64')]:
        build = source / processor
        result = subprocess.run(['cmake', '-S', str(source), '-B', str(build),
            f'-DTEST_PROCESSOR={processor}', f'-DTELEOP_APP={app}',
            f'-DMCL_PSIBOT_SDK_ROOT={root.resolve()}'], text=True, capture_output=True)
        assert result.returncode == 0, result.stdout + result.stderr
        assert (build / 'selected.txt').read_text() == str(root.resolve() / 'lib' / expected / 'libpsi_sdk.so')
print('SDK hashes, symlinks, ELF, symbols and CMake architecture/package selection passed; ARM not executed')
