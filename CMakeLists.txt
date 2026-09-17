cmake_minimum_required(VERSION 3.28)

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

project(
  motion_control_lab
  VERSION 0.1.0
  DESCRIPTION "Reproducible motion-control experiments"
  LANGUAGES CXX
)

include(CTest)
include(GNUInstallDirs)

option(MCL_BUILD_PSIBOT_TELEOP "Build the Psi SDK keyboard teleoperation client" OFF)
option(MCL_BUILD_JOINT_PATH_PLANNING "Build interactive joint-space obstacle planning" OFF)

option(
  MCL_BUILD_STEP
  "Build the dual-arm Step interactive IK application"
  ON
)
option(
  MCL_BUILD_TARGET
  "Build the dual-arm Target interactive IK application"
  ON
)
option(
  MCL_BUILD_BASELINE
  "Build the PlaCo production-static baseline application"
  ON
)
option(
  MCL_BUILD_HIERARCHICAL_KINEMATICS_STEP
  "Build the profile-driven hierarchical kinematics Step application"
  ON
)
option(
  MCL_BUILD_HIERARCHICAL_INVERSE_DYNAMICS_TORQUE_SIM
  "Build the fixed-base R1 hierarchical inverse-dynamics torque simulation"
  ON
)
set(MCL_ENABLE_CPU_AFFINITY_DEFAULT OFF)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  set(MCL_ENABLE_CPU_AFFINITY_DEFAULT ON)
endif()
option(
  MCL_ENABLE_CPU_AFFINITY
  "Compile Linux CPU affinity into IK applications"
  ${MCL_ENABLE_CPU_AFFINITY_DEFAULT}
)
unset(MCL_ENABLE_CPU_AFFINITY_DEFAULT)
option(MCL_ENABLE_TUI "Build the FTXUI terminal renderer" ON)
option(MCL_ENABLE_FOXGLOVE_TRANSPORT "Build Foxglove WebSocket/MCAP preview transports" ON)

if(MCL_ENABLE_CPU_AFFINITY AND NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR "MCL_ENABLE_CPU_AFFINITY is supported only on Linux")
endif()


set(MCL_BUILD_ANY_INTERACTIVE_IK OFF)
if(MCL_BUILD_JOINT_PATH_PLANNING OR MCL_BUILD_STEP OR
   MCL_BUILD_TARGET OR MCL_BUILD_BASELINE OR
   MCL_BUILD_HIERARCHICAL_KINEMATICS_STEP OR
   MCL_BUILD_HIERARCHICAL_INVERSE_DYNAMICS_TORQUE_SIM)
  set(MCL_BUILD_ANY_INTERACTIVE_IK ON)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(APPLE)
  set(CMAKE_INSTALL_RPATH "@loader_path/../${CMAKE_INSTALL_LIBDIR}")
elseif(UNIX)
  # Core's bundled Ruckig is an indirect dependency. Old-dtags RPATH is
  # intentionally transitive so build- and install-tree executables resolve
  # the complete native stack without an LD_LIBRARY_PATH workaround.
  add_link_options("LINKER:--disable-new-dtags")
  set(CMAKE_INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}")
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
endif()

list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(cmake/Dependencies.cmake)
include(cmake/MclApp.cmake)
option(MCL_BUILD_OPTIMIZATION_PROBLEM "Build explicit matrix optimization debugger" ON)
option(MCL_BUILD_PLANNED_KINEMATICS_STEP "Build Cartesian planning weighted IK and joint OTG app" ON)

install(
  PROGRAMS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/launch_app.sh"
  DESTINATION scripts
)

find_package(Python3 REQUIRED COMPONENTS Interpreter)

if(BUILD_TESTING)
  set(MCL_TEST_URDF "/workspace/models/r1.cos.urdf" CACHE FILEPATH
      "R1 model for runtime-only solver regression tests")
  find_program(MCL_BASH_EXECUTABLE bash REQUIRED)
  find_program(MCL_TRUE_EXECUTABLE true REQUIRED)
  find_program(MCL_SCRIPT_EXECUTABLE script)
endif()

set(MCL_VISUALIZATION_CONTRACT_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/contracts/visualization")
set(MCL_GENERATED_CONTRACT_DIR
    "${CMAKE_CURRENT_BINARY_DIR}/generated/contracts/visualization")
set(MCL_CONTRACT_GENERATOR
    "${MCL_VISUALIZATION_CONTRACT_DIR}/generate_contract_header.py")
set(MCL_VISUALIZATION_CONTRACT_JSONS
    "${MCL_VISUALIZATION_CONTRACT_DIR}/mcl_state.v1.json"
    "${MCL_VISUALIZATION_CONTRACT_DIR}/mcl_planning.v1.json"
    "${MCL_VISUALIZATION_CONTRACT_DIR}/mcl_execution.v1.json"
    "${MCL_VISUALIZATION_CONTRACT_DIR}/mcl_nullspace.v1.json"
    "${MCL_VISUALIZATION_CONTRACT_DIR}/mcl_telemetry.v1.json")

set(MCL_CHANNEL_SPEC_HEADER
    "${MCL_GENERATED_CONTRACT_DIR}/channel_spec.hpp")
add_custom_command(
  OUTPUT "${MCL_CHANNEL_SPEC_HEADER}"
  COMMAND
    "${Python3_EXECUTABLE}" "${MCL_CONTRACT_GENERATOR}"
    --common-output "${MCL_CHANNEL_SPEC_HEADER}"
  DEPENDS "${MCL_CONTRACT_GENERATOR}"
  VERBATIM
)

function(mcl_generate_visualization_contract BASENAME NAMESPACE)
  set(INPUT "${MCL_VISUALIZATION_CONTRACT_DIR}/${BASENAME}.v1.json")
  set(OUTPUT "${MCL_GENERATED_CONTRACT_DIR}/${BASENAME}_v1.hpp")
  add_custom_command(
    OUTPUT "${OUTPUT}"
    COMMAND
      "${Python3_EXECUTABLE}" "${MCL_CONTRACT_GENERATOR}"
      --input "${INPUT}"
      --output "${OUTPUT}"
      --namespace "${NAMESPACE}"
    DEPENDS "${MCL_CONTRACT_GENERATOR}" "${INPUT}" "${MCL_CHANNEL_SPEC_HEADER}"
    VERBATIM
  )
  set(MCL_GENERATED_CONTRACT_HEADERS
      ${MCL_GENERATED_CONTRACT_HEADERS} "${OUTPUT}" PARENT_SCOPE)
endfunction()

mcl_generate_visualization_contract(
  mcl_state motion_control_lab::contracts::mcl_state_v1)
mcl_generate_visualization_contract(
  mcl_planning motion_control_lab::contracts::mcl_planning_v1)
mcl_generate_visualization_contract(
  mcl_execution motion_control_lab::contracts::mcl_execution_v1)
mcl_generate_visualization_contract(
  mcl_nullspace motion_control_lab::contracts::mcl_nullspace_v1)
mcl_generate_visualization_contract(
  mcl_telemetry motion_control_lab::contracts::mcl_telemetry_v1)

add_custom_target(
  mcl_visualization_contract_headers
  DEPENDS "${MCL_CHANNEL_SPEC_HEADER}" ${MCL_GENERATED_CONTRACT_HEADERS}
)
add_library(mcl_visualization_contracts INTERFACE)
add_library(
  motion_control_lab::visualization_contracts
  ALIAS mcl_visualization_contracts
)
add_dependencies(
  mcl_visualization_contracts mcl_visualization_contract_headers
)
target_sources(
  mcl_visualization_contracts
  INTERFACE "${MCL_CHANNEL_SPEC_HEADER}" ${MCL_GENERATED_CONTRACT_HEADERS}
)
target_include_directories(
  mcl_visualization_contracts
  INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated>"
)

if(MCL_BUILD_HIERARCHICAL_KINEMATICS_STEP)
  find_package(Protobuf REQUIRED)
  set(MCL_TELEMETRY_PROTO
      "${CMAKE_CURRENT_SOURCE_DIR}/contracts/telemetry/mcl_telemetry_v1.proto")
  protobuf_generate_cpp(
    MCL_TELEMETRY_PROTO_SOURCES MCL_TELEMETRY_PROTO_HEADERS
    "${MCL_TELEMETRY_PROTO}"
  )
  add_library(
    mcl_telemetry_proto STATIC
    ${MCL_TELEMETRY_PROTO_SOURCES} ${MCL_TELEMETRY_PROTO_HEADERS}
  )
  add_library(motion_control_lab::telemetry_proto ALIAS mcl_telemetry_proto)
  target_include_directories(
    mcl_telemetry_proto
    PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
  )
  target_link_libraries(mcl_telemetry_proto PUBLIC protobuf::libprotobuf)
endif()


if(MCL_BUILD_ANY_INTERACTIVE_IK OR MCL_BUILD_OPTIMIZATION_PROBLEM OR MCL_BUILD_PLANNED_KINEMATICS_STEP)
  option(MCL_BUILD_HARP "Enable native HARP inference in hierarchical_kinematics_step" OFF)
  if(MCL_BUILD_HARP)
    find_package(motion_control_core 0.5 CONFIG REQUIRED COMPONENTS posture_reference)
  else()
    find_package(motion_control_core 0.5 CONFIG REQUIRED)
  endif()
endif()

if(MCL_BUILD_HIERARCHICAL_KINEMATICS_STEP OR
   MCL_BUILD_HIERARCHICAL_INVERSE_DYNAMICS_TORQUE_SIM)
  find_package(motion_control_sim 0.1 CONFIG REQUIRED)
endif()

if(MCL_BUILD_JOINT_PATH_PLANNING)
  find_package(motion_control_core 0.5 CONFIG REQUIRED COMPONENTS joint_planning)
endif()

if(MCL_BUILD_ANY_INTERACTIVE_IK)
  find_package(motion_control_viz 0.5 CONFIG REQUIRED)
endif()

add_library(mcl_input_contract INTERFACE)
add_library(motion_control_lab::input_contract ALIAS mcl_input_contract)
target_include_directories(
  mcl_input_contract INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
)
target_link_libraries(mcl_input_contract INTERFACE Eigen3::Eigen)

add_library(mcl_presentation_contract INTERFACE)
add_library(motion_control_lab::presentation_contract ALIAS mcl_presentation_contract)
target_include_directories(
  mcl_presentation_contract INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
)
target_link_libraries(mcl_presentation_contract INTERFACE Eigen3::Eigen)

add_library(mcl_runtime_contract INTERFACE)
add_library(motion_control_lab::runtime_contract ALIAS mcl_runtime_contract)
target_include_directories(
  mcl_runtime_contract INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
)

add_library(
  mcl_mcap
  adapters/data/source/mcap_implementation.cpp
)
add_library(motion_control_lab::mcap ALIAS mcl_mcap)
target_include_directories(
  mcl_mcap
  SYSTEM PUBLIC "${MCAP_CPP_SOURCE_DIR}/cpp/mcap/include"
)
target_compile_definitions(mcl_mcap PUBLIC MCAP_COMPRESSION_NO_LZ4)
target_link_libraries(mcl_mcap PUBLIC ${MCL_ZSTD_TARGET})

add_library(
  mcl_data
  contracts/data/sample_time.cpp
  adapters/data/source/data_source.cpp
  adapters/data/source/mcap_source.cpp
  adapters/data/source/csv_source.cpp
  adapters/data/decoder/cdr_reader.cpp
  adapters/data/decoder/decoder_utils.cpp
  adapters/data/decoder/ros2_pose_stamped_cdr_decoder.cpp
  adapters/data/decoder/ros2_joint_state_cdr_decoder.cpp
  adapters/data/decoder/csv_pose_decoder.cpp
  adapters/data/temporal/timestamp_policy.cpp
  adapters/data/temporal/timestamp_projector.cpp
  adapters/data/temporal/stream_alignment.cpp
  adapters/data/temporal/replay_clock.cpp
  adapters/data/projection/dual_arm_timeline.cpp
)
add_library(motion_control_lab::data ALIAS mcl_data)
target_include_directories(
  mcl_data
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
)
target_link_libraries(
  mcl_data
  PUBLIC Eigen3::Eigen motion_control_lab::mcap Threads::Threads
)

add_library(
  mcl_run_artifacts
  adapters/execution/src/run_artifacts.cpp
  adapters/execution/src/sha256.cpp
)
add_library(motion_control_lab::run_artifacts ALIAS mcl_run_artifacts)
target_include_directories(
  mcl_run_artifacts
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/adapters/execution/include>"
)
target_link_libraries(mcl_run_artifacts PUBLIC jsoncpp_lib)

add_library(mcl_replay adapters/replay/replay_support.cpp components/replay/replay_source.cpp)
add_library(motion_control_lab::replay ALIAS mcl_replay)
target_include_directories(mcl_replay PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
target_link_libraries(
  mcl_replay PUBLIC
    motion_control_lab::data
    motion_control_lab::input_contract
    motion_control_lab::run_artifacts
    jsoncpp_lib
)

add_library(mcl_cpu_affinity components/app_helpers/cpu_affinity.cpp)
add_library(motion_control_lab::cpu_affinity ALIAS mcl_cpu_affinity)
target_include_directories(mcl_cpu_affinity PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
target_compile_definitions(
  mcl_cpu_affinity PUBLIC MCL_ENABLE_CPU_AFFINITY=$<BOOL:${MCL_ENABLE_CPU_AFFINITY}>
)
target_link_libraries(mcl_cpu_affinity PRIVATE Threads::Threads)

add_library(mcl_r1_robot_config components/robot/r1/r1_robot_config.cpp)
add_library(motion_control_lab::r1_robot_config ALIAS mcl_r1_robot_config)
target_include_directories(mcl_r1_robot_config PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
target_link_libraries(mcl_r1_robot_config PUBLIC Eigen3::Eigen)

add_library(mcl_app_helpers components/app_helpers/app_helpers.cpp)
add_library(motion_control_lab::app_helpers ALIAS mcl_app_helpers)
target_include_directories(mcl_app_helpers PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
target_link_libraries(
  mcl_app_helpers PUBLIC
    motion_control_lab::cpu_affinity
    motion_control_lab::presentation_contract
    Eigen3::Eigen
)

add_library(
  mcl_scheduler
  components/scheduler/grouped_worker.cpp
  components/scheduler/rolling_percentiles.cpp
  components/scheduler/single_rate_scheduler.cpp
)
add_library(motion_control_lab::scheduler ALIAS mcl_scheduler)
target_include_directories(mcl_scheduler PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
target_link_libraries(mcl_scheduler PUBLIC motion_control_lab::runtime_contract Threads::Threads)

add_library(
  mcl_terminal_frontend
  components/terminal_frontend/terminal_frontend.cpp
  components/terminal_frontend/key_router.cpp
)
add_library(motion_control_lab::terminal_frontend ALIAS mcl_terminal_frontend)
target_include_directories(mcl_terminal_frontend PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
target_link_libraries(mcl_terminal_frontend PUBLIC motion_control_lab::input_contract)

add_library(mcl_keyboard_teleop components/teleop/keyboard/keyboard_teleop.cpp)
add_library(motion_control_lab::keyboard_teleop ALIAS mcl_keyboard_teleop)
target_include_directories(mcl_keyboard_teleop PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
target_link_libraries(mcl_keyboard_teleop PUBLIC motion_control_lab::input_contract)

add_library(mcl_cartesian_teleop components/teleop/cartesian/cartesian_teleop.cpp)
add_library(motion_control_lab::cartesian_teleop ALIAS mcl_cartesian_teleop)
target_include_directories(mcl_cartesian_teleop PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
target_link_libraries(mcl_cartesian_teleop PUBLIC motion_control_lab::input_contract Eigen3::Eigen)

add_library(mcl_keyboard_input components/teleop/keyboard/keyboard_target_source.cpp)
add_library(motion_control_lab::keyboard_input ALIAS mcl_keyboard_input)
target_include_directories(mcl_keyboard_input PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
target_link_libraries(
  mcl_keyboard_input PUBLIC
    motion_control_lab::terminal_frontend
    motion_control_lab::keyboard_teleop
    motion_control_lab::cartesian_teleop
)

add_library(mcl_standard_ik_tui components/tui/standard_ik_tui.cpp)
add_library(motion_control_lab::standard_ik_tui ALIAS mcl_standard_ik_tui)
target_include_directories(
  mcl_standard_ik_tui PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
)
target_link_libraries(mcl_standard_ik_tui PUBLIC motion_control_lab::presentation_contract)

add_library(mcl_planned_grouped_tui components/tui/planned_grouped_tui.cpp)
add_library(motion_control_lab::planned_grouped_tui ALIAS mcl_planned_grouped_tui)
target_include_directories(
  mcl_planned_grouped_tui PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
)
target_link_libraries(
  mcl_planned_grouped_tui
  PUBLIC
    motion_control_lab::standard_ik_tui
    motion_control_lab::tui
)

add_library(mcl_tui components/tui/tui_help.cpp components/tui/tui_renderer.cpp)
add_library(motion_control_lab::tui ALIAS mcl_tui)
target_include_directories(mcl_tui PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
target_link_libraries(
  mcl_tui PUBLIC motion_control_lab::input_contract motion_control_lab::presentation_contract
  PRIVATE motion_control_lab::terminal_frontend
)
if(MCL_ENABLE_TUI)
  target_compile_definitions(mcl_tui PRIVATE MCL_WITH_FTXUI=1)
  target_link_libraries(mcl_tui PRIVATE ftxui::ftxui)
else()
  target_compile_definitions(mcl_tui PRIVATE MCL_WITH_FTXUI=0)
endif()

add_library(mcl_app_scaffold INTERFACE)
if(MCL_BUILD_PSIBOT_TELEOP)
  add_subdirectory(apps/psibot_teleop)
endif()
add_library(motion_control_lab::app_scaffold ALIAS mcl_app_scaffold)
target_include_directories(mcl_app_scaffold INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
target_link_libraries(mcl_app_scaffold INTERFACE motion_control_lab::runtime_contract)

if(MCL_BUILD_ANY_INTERACTIVE_IK)
  add_library(
    mcl_preview_transport
    components/visualization/preview_transport.cpp
  )
  add_library(motion_control_lab::preview_transport ALIAS mcl_preview_transport)
  target_include_directories(
    mcl_preview_transport
    PUBLIC
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
  )
  target_link_libraries(
    mcl_preview_transport
    PUBLIC motion_control_viz::render
  )
  if(MCL_ENABLE_FOXGLOVE_TRANSPORT AND TARGET motion_control_viz::foxglove)
    target_link_libraries(
      mcl_preview_transport
      PRIVATE motion_control_viz::foxglove
    )
    target_compile_definitions(
      mcl_preview_transport
      PRIVATE MCL_WITH_FOXGLOVE_TRANSPORT=1
    )
  else()
    target_compile_definitions(
      mcl_preview_transport
      PRIVATE MCL_WITH_FOXGLOVE_TRANSPORT=0
    )
  endif()
endif()

if(MCL_BUILD_ANY_INTERACTIVE_IK)
  add_library(
    mcl_preview_projection
    components/visualization/preview_projection.cpp
  )
  add_library(motion_control_lab::preview_projection ALIAS mcl_preview_projection)
  target_include_directories(
    mcl_preview_projection
    PUBLIC
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
  )
  target_link_libraries(
    mcl_preview_projection
    PUBLIC
      motion_control_lab::presentation_contract
      motion_control_lab::r1_robot_config
      motion_control_lab::visualization_contracts
      motion_control_viz::render
  )

  if(MCL_BUILD_STEP)
    add_subdirectory(apps/step)
  endif()
  if(MCL_BUILD_JOINT_PATH_PLANNING)
    add_subdirectory(apps/joint_path_planning)
  endif()
  if(MCL_BUILD_TARGET)
    add_subdirectory(apps/target)
  endif()
  if(MCL_BUILD_BASELINE)
    add_subdirectory(apps/baseline)
  endif()
  if(MCL_BUILD_HIERARCHICAL_KINEMATICS_STEP)
    add_subdirectory(
      apps/hierarchical_kinematics_step)
  endif()
  if(MCL_BUILD_HIERARCHICAL_INVERSE_DYNAMICS_TORQUE_SIM)
    add_subdirectory(apps/hierarchical_inverse_dynamics_torque_sim)
  endif()
endif()

install(
  DIRECTORY assets/robots/r1/mujoco/
  DESTINATION ${CMAKE_INSTALL_DATADIR}/motion-control-lab/robots/r1/mujoco
  PATTERN "prepare_asset.py" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)




add_subdirectory(apps/replay_plan)

install(
  TARGETS libplaco
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Functional execution programs, reusable across experiment declarations.
if(MCL_BUILD_OPTIMIZATION_PROBLEM)
  add_subdirectory(apps/optimization_problem)
endif()
if(MCL_BUILD_PLANNED_KINEMATICS_STEP)
  add_subdirectory(apps/planned_kinematics_step)
endif()

include(cmake/SharedTests.cmake)
