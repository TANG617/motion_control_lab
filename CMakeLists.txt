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

option(
  MCL_BUILD_SINGLE_ARM_STEP
  "Build the single-arm interactive IK application"
  ON
)
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
  MCL_BUILD_HIERARCHICAL_STEP
  "Build the Red/Yellow dual-arm IK application"
  ON
)
option(
  MCL_BUILD_PLANNED_HIERARCHICAL_STEP
  "Build the online-planned Red/Yellow dual-arm IK application"
  ON
)
option(
  MCL_BUILD_PLANNED_HIERARCHICAL_STEP_OTG
  "Build the online-planned Red/Yellow IK plus joint-OTG application"
  ON
)
option(
  MCL_BUILD_MCC_PLOTS
  "Build the optional motion_control_core matplotlib renderer"
  OFF
)
option(
  MCL_BUILD_CARTESIAN_PLANNING
  "Build the JSON-driven Cartesian MoveLine visualization application"
  ON
)
option(
  MCL_BUILD_E03_BATCH_REPLAY_IK
  "Build the E03 PSI R1 motion-library batch replay IK runner"
  ON
)
option(
  MCL_BUILD_E04_OPENSOT_SMOKE
  "Build the isolated ROS2 OpenSoT R1 smoke experiment"
  OFF
)
option(
  MCL_BUILD_E03_REPLAY_VISUALIZATION
  "Build optional Foxglove live visualization into the E03 legacy replay runner"
  OFF
)
option(
  MCL_ENABLE_CPU_AFFINITY
  "Compile Linux CPU affinity into IK applications"
  OFF
)
option(MCL_ENABLE_TUI "Build the FTXUI terminal renderer" ON)
option(MCL_ENABLE_FOXGLOVE_TRANSPORT "Build Foxglove WebSocket/MCAP preview transports" ON)

if(MCL_ENABLE_CPU_AFFINITY AND NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR "MCL_ENABLE_CPU_AFFINITY is supported only on Linux")
endif()

if(MCL_BUILD_E03_REPLAY_VISUALIZATION AND
   NOT MCL_BUILD_E03_BATCH_REPLAY_IK)
  message(
    FATAL_ERROR
    "Replay visualization requires an enabled replay IK runner"
  )
endif()
if(MCL_BUILD_E03_REPLAY_VISUALIZATION AND
   NOT MCL_ENABLE_FOXGLOVE_TRANSPORT)
  message(
    FATAL_ERROR
    "E03 replay visualization requires MCL_ENABLE_FOXGLOVE_TRANSPORT=ON"
  )
endif()

set(MCL_BUILD_ANY_INTERACTIVE_IK OFF)
if(MCL_BUILD_SINGLE_ARM_STEP OR MCL_BUILD_STEP OR
   MCL_BUILD_TARGET OR MCL_BUILD_BASELINE OR MCL_BUILD_HIERARCHICAL_STEP OR
   MCL_BUILD_PLANNED_HIERARCHICAL_STEP OR MCL_BUILD_PLANNED_HIERARCHICAL_STEP_OTG)
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

install(
  PROGRAMS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/launch_app.sh"
  DESTINATION scripts
)

find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(MCL_VISUALIZATION_CONTRACT_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/contracts/visualization")
set(MCL_GENERATED_CONTRACT_DIR
    "${CMAKE_CURRENT_BINARY_DIR}/generated/contracts/visualization")
set(MCL_CONTRACT_GENERATOR
    "${MCL_VISUALIZATION_CONTRACT_DIR}/generate_contract_header.py")
set(MCL_VISUALIZATION_CONTRACT_JSONS
    "${MCL_VISUALIZATION_CONTRACT_DIR}/foxglove_ik.v1.json"
    "${MCL_VISUALIZATION_CONTRACT_DIR}/foxglove_planned_hierarchical_step.v1.json"
    "${MCL_VISUALIZATION_CONTRACT_DIR}/foxglove_planned_hierarchical_step_otg.v1.json")

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
  foxglove_ik motion_control_lab::contracts::foxglove_ik_v1)
mcl_generate_visualization_contract(
  foxglove_planned_hierarchical_step
  motion_control_lab::contracts::foxglove_planned_hierarchical_step_v1)
mcl_generate_visualization_contract(
  foxglove_planned_hierarchical_step_otg
  motion_control_lab::contracts::foxglove_planned_hierarchical_step_otg_v1)

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

if(MCL_BUILD_E04_OPENSOT_SMOKE)
  include(cmake/OpenSoTExternal.cmake)
endif()

if(MCL_BUILD_ANY_INTERACTIVE_IK OR MCL_BUILD_MCC_PLOTS OR MCL_BUILD_CARTESIAN_PLANNING OR
   MCL_BUILD_E03_BATCH_REPLAY_IK)
  find_package(motion_control_core CONFIG REQUIRED)
endif()

if(MCL_BUILD_ANY_INTERACTIVE_IK OR MCL_BUILD_CARTESIAN_PLANNING OR
   MCL_BUILD_E03_REPLAY_VISUALIZATION)
  find_package(motion_control_viz 0.4 CONFIG REQUIRED)
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
)
if(MCL_ENABLE_TUI)
  target_compile_definitions(mcl_tui PRIVATE MCL_WITH_FTXUI=1)
  target_link_libraries(mcl_tui PRIVATE ftxui::ftxui)
else()
  target_compile_definitions(mcl_tui PRIVATE MCL_WITH_FTXUI=0)
endif()

add_library(mcl_app_scaffold INTERFACE)
add_library(motion_control_lab::app_scaffold ALIAS mcl_app_scaffold)
target_include_directories(mcl_app_scaffold INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
target_link_libraries(mcl_app_scaffold INTERFACE motion_control_lab::runtime_contract)

if(MCL_BUILD_ANY_INTERACTIVE_IK OR MCL_BUILD_CARTESIAN_PLANNING OR
   MCL_BUILD_E03_REPLAY_VISUALIZATION)
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

  if(MCL_BUILD_SINGLE_ARM_STEP)
    add_subdirectory(apps/single_arm_step)
  endif()
  if(MCL_BUILD_STEP)
    add_subdirectory(apps/step)
  endif()
  if(MCL_BUILD_TARGET)
    add_subdirectory(apps/target)
  endif()
  if(MCL_BUILD_BASELINE)
    add_subdirectory(apps/baseline)
  endif()
  if(MCL_BUILD_HIERARCHICAL_STEP)
    add_subdirectory(apps/hierarchical_step)
  endif()
  if(MCL_BUILD_PLANNED_HIERARCHICAL_STEP)
    add_subdirectory(apps/planned_hierarchical_step)
  endif()
  if(MCL_BUILD_PLANNED_HIERARCHICAL_STEP_OTG)
    add_subdirectory(apps/planned_hierarchical_step_otg)
  endif()
endif()

if(MCL_BUILD_CARTESIAN_PLANNING)
  if(NOT TARGET motion_control_viz::foxglove)
    message(
      FATAL_ERROR
      "MCL_BUILD_CARTESIAN_PLANNING requires motion_control_viz with Foxglove support"
    )
  endif()
  find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)
  execute_process(
    COMMAND "${Python3_EXECUTABLE}" -c "import matplotlib, numpy"
    RESULT_VARIABLE MCL_CARTESIAN_MATPLOTLIB_IMPORT_RESULT
    OUTPUT_VARIABLE MCL_CARTESIAN_MATPLOTLIB_IMPORT_OUTPUT
    ERROR_VARIABLE MCL_CARTESIAN_MATPLOTLIB_IMPORT_ERROR
  )
  if(NOT MCL_CARTESIAN_MATPLOTLIB_IMPORT_RESULT EQUAL 0)
    message(
      FATAL_ERROR
      "MCL_BUILD_CARTESIAN_PLANNING requires matplotlib and numpy:\n"
      "${MCL_CARTESIAN_MATPLOTLIB_IMPORT_OUTPUT}${MCL_CARTESIAN_MATPLOTLIB_IMPORT_ERROR}"
    )
  endif()
  add_subdirectory(apps/cartesian_planning)
endif()

if(MCL_BUILD_MCC_PLOTS)
  find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)
  execute_process(
    COMMAND "${Python3_EXECUTABLE}" -c "import matplotlib, numpy"
    RESULT_VARIABLE MCL_MATPLOTLIB_IMPORT_RESULT
    OUTPUT_VARIABLE MCL_MATPLOTLIB_IMPORT_OUTPUT
    ERROR_VARIABLE MCL_MATPLOTLIB_IMPORT_ERROR
  )
  if(NOT MCL_MATPLOTLIB_IMPORT_RESULT EQUAL 0)
    message(
      FATAL_ERROR
      "MCL_BUILD_MCC_PLOTS requires matplotlib and numpy:\n"
      "${MCL_MATPLOTLIB_IMPORT_OUTPUT}${MCL_MATPLOTLIB_IMPORT_ERROR}"
    )
  endif()

  add_subdirectory(apps/plot_core_planning)
endif()

if(MCL_BUILD_E03_BATCH_REPLAY_IK)
  add_library(
    mcl_e03_legacy_replay_ik_engine
    experiments/E03_psi_r1_dual_arm_motion_library_replay_ik/src/legacy_replay/replay_ik_engine.cpp
  )
  add_library(
    motion_control_lab::e03_legacy_replay_ik_engine
    ALIAS mcl_e03_legacy_replay_ik_engine
  )
  target_include_directories(
    mcl_e03_legacy_replay_ik_engine
    PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
  )
  target_link_libraries(
    mcl_e03_legacy_replay_ik_engine
    PUBLIC
      motion_control_lab::replay
      motion_control_lab::r1_robot_config
      motion_control_core::motion_control_core
  )
endif()

add_subdirectory(apps/replay_plan)

set(MCL_SOURCE_REVISION "unborn")
set(MCL_SOURCE_DIRTY true)
execute_process(
  COMMAND git rev-parse HEAD
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  RESULT_VARIABLE MCL_GIT_REVISION_RESULT
  OUTPUT_VARIABLE MCL_GIT_REVISION
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET
)
if(MCL_GIT_REVISION_RESULT EQUAL 0)
  set(MCL_SOURCE_REVISION "${MCL_GIT_REVISION}")
endif()
execute_process(
  COMMAND git status --porcelain
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  RESULT_VARIABLE MCL_GIT_STATUS_RESULT
  OUTPUT_VARIABLE MCL_GIT_STATUS
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET
)
if(MCL_GIT_STATUS_RESULT EQUAL 0 AND MCL_GIT_STATUS STREQUAL "")
  set(MCL_SOURCE_DIRTY false)
endif()

set(E01_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/experiments/E01_placo_smoke")
set(E01_DEFINITION "${E01_DIRECTORY}/definition.json")
set(E01_MODEL "/workspace/models/r1.cos.urdf")
set(MCL_PLACO_UPSTREAM_REVISION "e6c288604639d67b979a16cb2ad26913413c8e3a")
set(
  MCL_RUNTIME_DESCRIPTION
  "C++17;${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR};${CMAKE_CXX_COMPILER_ID}-${CMAKE_CXX_COMPILER_VERSION}"
)

set(E02_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/experiments/E02_dual_arm_replay_ik")
set(E02_DEFINITION "${E02_DIRECTORY}/definition.json")
file(SHA256 "${E02_DEFINITION}" E02_DEFINITION_SHA256)
set_property(
  DIRECTORY
  APPEND
  PROPERTY CMAKE_CONFIGURE_DEPENDS "${E02_DEFINITION}"
)
configure_file(
  "${E02_DIRECTORY}/src/build_config.hpp.in"
  "${CMAKE_CURRENT_BINARY_DIR}/generated/e02_build_config.hpp"
  @ONLY
)

set(
  E03_DIRECTORY
  "${CMAKE_CURRENT_SOURCE_DIR}/experiments/E03_psi_r1_dual_arm_motion_library_replay_ik"
)
set(E03_DEFINITION "${E03_DIRECTORY}/definition.json")
set(
  E03_INPUT_CONTRACT
  "${E03_DIRECTORY}/inputs/psi_r1_dual_arm_motion_library.v1.json"
)
get_filename_component(
  MCL_WORKSPACE_DIRECTORY
  "${CMAKE_CURRENT_SOURCE_DIR}/../.."
  ABSOLUTE
)
set(
  E03_DEFAULT_LIBRARY_DIRECTORY
  "${MCL_WORKSPACE_DIRECTORY}/fixtures/datasets/psi_r1_dual_arm_motion_library"
)
file(SHA256 "${E03_DEFINITION}" E03_DEFINITION_SHA256)
set_property(
  DIRECTORY
  APPEND
  PROPERTY CMAKE_CONFIGURE_DEPENDS "${E03_DEFINITION}" "${E03_INPUT_CONTRACT}"
)
configure_file(
  "${E03_DIRECTORY}/src/build_config.hpp.in"
  "${CMAKE_CURRENT_BINARY_DIR}/generated/e03_build_config.hpp"
  @ONLY
)

if(MCL_BUILD_E03_BATCH_REPLAY_IK)
  add_subdirectory(experiments/E03_psi_r1_dual_arm_motion_library_replay_ik)
endif()

file(SHA256 "${E01_DEFINITION}" E01_DEFINITION_SHA256)
file(SHA256 "${E01_MODEL}" E01_MODEL_SHA256)
file(
  GLOB_RECURSE MCL_PLACO_BUILD_INPUTS
  CONFIGURE_DEPENDS
  LIST_DIRECTORIES false
  RELATIVE "${MCL_PLACO_SOURCE_DIR}"
  "${MCL_PLACO_SOURCE_DIR}/src/*"
)
list(APPEND MCL_PLACO_BUILD_INPUTS "CMakeLists.txt")
list(SORT MCL_PLACO_BUILD_INPUTS)
set(MCL_PLACO_FINGERPRINT_INPUT "")
foreach(MCL_PLACO_BUILD_INPUT IN LISTS MCL_PLACO_BUILD_INPUTS)
  file(
    SHA256
    "${MCL_PLACO_SOURCE_DIR}/${MCL_PLACO_BUILD_INPUT}"
    MCL_PLACO_BUILD_INPUT_SHA256
  )
  string(
    APPEND MCL_PLACO_FINGERPRINT_INPUT
    "${MCL_PLACO_BUILD_INPUT}:${MCL_PLACO_BUILD_INPUT_SHA256}\n"
  )
endforeach()
string(
  SHA256
  MCL_PLACO_SOURCE_SHA256
  "${MCL_PLACO_FINGERPRINT_INPUT}"
)
set(
  MCL_PLACO_REVISION
  "upstream:${MCL_PLACO_UPSTREAM_REVISION};source-sha256:${MCL_PLACO_SOURCE_SHA256}"
)
set_property(
  DIRECTORY
  APPEND
  PROPERTY CMAKE_CONFIGURE_DEPENDS "${E01_DEFINITION}" "${E01_MODEL}"
)

configure_file(
  "${E01_DIRECTORY}/src/build_config.hpp.in"
  "${CMAKE_CURRENT_BINARY_DIR}/generated/e01_build_config.hpp"
  @ONLY
)

add_executable(
  e01_placo_smoke
  experiments/E01_placo_smoke/src/main.cpp
)
if(APPLE)
  set_target_properties(e01_placo_smoke PROPERTIES INSTALL_RPATH "@loader_path/../lib")
else()
  set_target_properties(e01_placo_smoke PROPERTIES INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}")
endif()
target_include_directories(
  e01_placo_smoke
  PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/generated"
)
target_link_libraries(
  e01_placo_smoke
  PRIVATE
    libplaco
    motion_control_lab::run_artifacts
)

set(MCL_INSTALL_RUNTIME_TARGETS e01_placo_smoke)
if(MCL_BUILD_E04_OPENSOT_SMOKE)
  set(E04_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/experiments/E04_opensot_smoke")
  set(E04_DEFINITION "${E04_DIRECTORY}/definition.json")
  set(E04_MODEL "/workspace/models/r1.cos.urdf")
  set(MCL_OPENSOT_UPSTREAM_REVISION "f1d8c733bcff49ed5b213191ac85a0629bb97d43")
  set(
    E04_RUNTIME_DESCRIPTION
    "C++17-Lab/C++20-OpenSoT;${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR};${CMAKE_CXX_COMPILER_ID}-${CMAKE_CXX_COMPILER_VERSION};ROS2-Jazzy"
  )

  file(SHA256 "${E04_DEFINITION}" E04_DEFINITION_SHA256)
  file(SHA256 "${E04_MODEL}" E04_MODEL_SHA256)
  file(
    GLOB_RECURSE MCL_OPENSOT_BUILD_INPUTS
    CONFIGURE_DEPENDS
    LIST_DIRECTORIES false
    RELATIVE "${MCL_OPENSOT_SOURCE_DIR}"
    "${MCL_OPENSOT_SOURCE_DIR}/*"
  )
  # The provenance note records this digest and is therefore not part of the
  # digest input. All imported source, licenses, build files, and local patches are.
  list(REMOVE_ITEM MCL_OPENSOT_BUILD_INPUTS "MOTION_CONTROL_LAB.md")
  list(SORT MCL_OPENSOT_BUILD_INPUTS)
  set(MCL_OPENSOT_FINGERPRINT_INPUT "")
  foreach(MCL_OPENSOT_BUILD_INPUT IN LISTS MCL_OPENSOT_BUILD_INPUTS)
    file(
      SHA256
      "${MCL_OPENSOT_SOURCE_DIR}/${MCL_OPENSOT_BUILD_INPUT}"
      MCL_OPENSOT_BUILD_INPUT_SHA256
    )
    string(
      APPEND MCL_OPENSOT_FINGERPRINT_INPUT
      "${MCL_OPENSOT_BUILD_INPUT}:${MCL_OPENSOT_BUILD_INPUT_SHA256}\n"
    )
  endforeach()
  string(
    SHA256
    MCL_OPENSOT_SOURCE_SHA256
    "${MCL_OPENSOT_FINGERPRINT_INPUT}"
  )
  set(
    MCL_OPENSOT_REVISION
    "upstream:${MCL_OPENSOT_UPSTREAM_REVISION};source-sha256:${MCL_OPENSOT_SOURCE_SHA256}"
  )
  set_property(
    DIRECTORY
    APPEND
    PROPERTY CMAKE_CONFIGURE_DEPENDS "${E04_DEFINITION}" "${E04_MODEL}"
  )
  configure_file(
    "${E04_DIRECTORY}/src/build_config.hpp.in"
    "${CMAKE_CURRENT_BINARY_DIR}/generated/e04_build_config.hpp"
    @ONLY
  )

  add_executable(
    e04_opensot_smoke
    experiments/E04_opensot_smoke/src/main.cpp
  )
  add_dependencies(e04_opensot_smoke mcl_opensot_external)
  target_include_directories(
    e04_opensot_smoke
    PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/generated"
  )
  target_link_libraries(
    e04_opensot_smoke
    PRIVATE
      mcl_opensot_bridge
      motion_control_lab::run_artifacts
  )
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_link_options(
      e04_opensot_smoke
      PRIVATE
        "LINKER:-rpath-link,${MCL_OPENSOT_EXTERNAL_INSTALL_DIR}/lib"
        "LINKER:-rpath-link,${MCL_OPENSOT_DEPENDENCY_PREFIX}/lib"
        "LINKER:-rpath-link,${MCL_OPENSOT_ROS_PREFIX}/lib"
    )
  endif()
  set_target_properties(
    e04_opensot_smoke
    PROPERTIES
      BUILD_RPATH
        "${MCL_OPENSOT_EXTERNAL_INSTALL_DIR}/lib;${MCL_OPENSOT_DEPENDENCY_PREFIX}/lib;/opt/openrobots/lib;${MCL_OPENSOT_ROS_PREFIX}/lib"
      INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}/motion-control-lab/opensot"
  )
  list(APPEND MCL_INSTALL_RUNTIME_TARGETS e04_opensot_smoke)

  install(
    DIRECTORY "${MCL_OPENSOT_EXTERNAL_INSTALL_DIR}/lib/"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/motion-control-lab/opensot"
    OPTIONAL
    FILES_MATCHING
      PATTERN "cmake" EXCLUDE
      PATTERN "libmcl_opensot_bridge.so*"
      PATTERN "libOpenSoT.so*"
      PATTERN "libOpenSotBackEndOSQP.so*"
  )
endif()

if(MCL_BUILD_MCC_PLOTS)
  list(APPEND MCL_INSTALL_RUNTIME_TARGETS motion_control_lab_plot_core_planning)
endif()
install(
  TARGETS ${MCL_INSTALL_RUNTIME_TARGETS}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(
  TARGETS libplaco
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(BUILD_TESTING)
  find_program(MCL_BASH_EXECUTABLE bash REQUIRED)
  file(
    GLOB_RECURSE MCL_APP_SHELL_SCRIPTS
    CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.sh"
    "${CMAKE_CURRENT_SOURCE_DIR}/apps/*/scripts/*.sh"
    "${CMAKE_CURRENT_SOURCE_DIR}/experiments/*/scripts/*.sh"
  )
  add_test(NAME scripts.bash_syntax COMMAND "${MCL_BASH_EXECUTABLE}" -n ${MCL_APP_SHELL_SCRIPTS})
  find_program(MCL_SCRIPT_EXECUTABLE script)

  if(MCL_BUILD_PLANNED_HIERARCHICAL_STEP_OTG)
    add_test(
      NAME scripts.source_launcher_path_resolution
      COMMAND
        "${CMAKE_COMMAND}" -E env
        --unset=MCL_BINARY
        "PATH=$<TARGET_FILE_DIR:mcl_planned_hierarchical_step_otg>:$ENV{PATH}"
        "MCL_UI=none"
        "MCL_VIZ=none"
        "${CMAKE_CURRENT_SOURCE_DIR}/apps/planned_hierarchical_step_otg/scripts/run_keyboard.sh"
        --help
    )

    if(MCL_SCRIPT_EXECUTABLE)
      add_test(
        NAME scripts.planned_hierarchical_step_otg_keyboard_smoke
        COMMAND
          "${CMAKE_COMMAND}" -E env
          --unset=MCL_BINARY
          "PATH=$<TARGET_FILE_DIR:mcl_planned_hierarchical_step_otg>:$ENV{PATH}"
          "MCL_UI=none"
          "MCL_VIZ=none"
          "${MCL_SCRIPT_EXECUTABLE}" -qec
          "${CMAKE_CURRENT_SOURCE_DIR}/apps/planned_hierarchical_step_otg/scripts/run_keyboard.sh --duration 0.05"
          /dev/null
      )
      set_tests_properties(
        scripts.planned_hierarchical_step_otg_keyboard_smoke
        PROPERTIES TIMEOUT 15
      )
    endif()
  endif()

  if(MCL_BUILD_PLANNED_HIERARCHICAL_STEP AND MCL_SCRIPT_EXECUTABLE)
    add_test(
      NAME scripts.planned_hierarchical_step_keyboard_smoke
      COMMAND
        "${CMAKE_COMMAND}" -E env
        --unset=MCL_BINARY
        "PATH=$<TARGET_FILE_DIR:mcl_planned_hierarchical_step>:$ENV{PATH}"
        "MCL_UI=none"
        "MCL_VIZ=none"
        "${MCL_SCRIPT_EXECUTABLE}" -qec
        "${CMAKE_CURRENT_SOURCE_DIR}/apps/planned_hierarchical_step/scripts/run_keyboard.sh --duration 0.05"
        /dev/null
    )
    set_tests_properties(
      scripts.planned_hierarchical_step_keyboard_smoke
      PROPERTIES TIMEOUT 15
    )
  endif()

  add_test(
    NAME contracts.visualization_generator
    COMMAND
      "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/tests/visualization_contract_generator.py"
      "${MCL_CONTRACT_GENERATOR}"
      ${MCL_VISUALIZATION_CONTRACT_JSONS}
  )

  add_executable(
    test_data_pipeline
    tests/data_pipeline.cpp
  )
  target_link_libraries(
    test_data_pipeline
    PRIVATE motion_control_lab::replay
  )
  add_test(NAME data.pipeline COMMAND test_data_pipeline)

  add_executable(test_input_teleop tests/input_teleop.cpp)
  target_link_libraries(
    test_input_teleop PRIVATE motion_control_lab::keyboard_teleop motion_control_lab::cartesian_teleop
  )
  add_test(NAME input.teleop_contract COMMAND test_input_teleop)

  add_executable(test_replay_source tests/replay_source.cpp)
  target_link_libraries(test_replay_source PRIVATE motion_control_lab::replay)
  add_test(NAME replay.state_machine COMMAND test_replay_source)

  add_executable(test_replay_manifest tests/replay_manifest.cpp)
  target_link_libraries(test_replay_manifest PRIVATE motion_control_lab::replay)
  add_test(NAME artifacts.replay_manifest COMMAND test_replay_manifest)

  add_executable(test_app_scaffold tests/app_scaffold.cpp)
  target_link_libraries(test_app_scaffold PRIVATE motion_control_lab::app_scaffold)
  add_test(NAME runtime.app_scaffold COMMAND test_app_scaffold)

  add_test(
    NAME architecture.boundaries
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/architecture_boundaries.py"
            "${CMAKE_CURRENT_SOURCE_DIR}"
  )

  if(TARGET mcl_preview_transport)
    add_executable(test_preview_null tests/preview_null.cpp)
    target_link_libraries(test_preview_null PRIVATE motion_control_lab::preview_transport)
    add_test(NAME visualization.null_sink COMMAND test_preview_null)
  endif()

  add_executable(
    test_cpu_affinity
    tests/cpu_affinity.cpp
  )
  target_link_libraries(
    test_cpu_affinity
    PRIVATE motion_control_lab::cpu_affinity
  )
  add_test(NAME apps.cpu_affinity COMMAND test_cpu_affinity)

  if(MCL_BUILD_ANY_INTERACTIVE_IK)
    add_executable(
      test_interactive_ik_configuration
      tests/interactive_ik_configuration.cpp
    )
    target_link_libraries(
      test_interactive_ik_configuration
      PRIVATE
        motion_control_lab::app_helpers
        motion_control_lab::r1_robot_config
        motion_control_lab::scheduler
        motion_control_lab::preview_projection
        motion_control_lab::visualization_contracts
    )
    add_test(
      NAME interactive.ik_configuration
      COMMAND test_interactive_ik_configuration
    )

    add_executable(
      test_rolling_percentiles
      tests/rolling_percentiles.cpp
    )
    target_link_libraries(
      test_rolling_percentiles
      PRIVATE motion_control_lab::scheduler
    )
    add_test(
      NAME interactive.rolling_percentiles
      COMMAND test_rolling_percentiles
    )

    add_executable(
      test_grouped_runtime
      tests/grouped_runtime.cpp
    )
    target_link_libraries(
      test_grouped_runtime
      PRIVATE
        motion_control_lab::scheduler
        Threads::Threads
    )
    add_test(
      NAME interactive.grouped_runtime
      COMMAND test_grouped_runtime
    )

    add_executable(
      test_tui_console
      tests/tui_console.cpp
    )
    target_link_libraries(
      test_tui_console
      PRIVATE
        motion_control_lab::terminal_frontend
        motion_control_lab::keyboard_input
        motion_control_lab::tui
    )

    add_executable(test_standard_ik_tui tests/standard_ik_tui.cpp)
    target_link_libraries(test_standard_ik_tui PRIVATE motion_control_lab::standard_ik_tui)
    add_test(NAME interactive.standard_ik_tui COMMAND test_standard_ik_tui)
    if(MCL_ENABLE_TUI)
      add_test(
        NAME interactive.tui_console_pty
        COMMAND
          "${Python3_EXECUTABLE}"
          "${CMAKE_CURRENT_SOURCE_DIR}/tests/run_tui_pty.py"
          "$<TARGET_FILE:test_tui_console>"
      )
      add_test(
        NAME interactive.tui_exception_pty
        COMMAND
          "${Python3_EXECUTABLE}"
          "${CMAKE_CURRENT_SOURCE_DIR}/tests/run_tui_pty.py"
          "$<TARGET_FILE:test_tui_console>"
          --expect-exception
      )
      add_test(
        NAME interactive.tui_fault_hold_pty
        COMMAND
          "${Python3_EXECUTABLE}"
          "${CMAKE_CURRENT_SOURCE_DIR}/tests/run_tui_pty.py"
          "$<TARGET_FILE:test_tui_console>"
          --fault-hold
      )
      add_test(
        NAME interactive.tui_replay_controls_pty
        COMMAND
          "${Python3_EXECUTABLE}"
          "${CMAKE_CURRENT_SOURCE_DIR}/tests/run_tui_pty.py"
          "$<TARGET_FILE:test_tui_console>"
          --replay
      )
      add_test(
        NAME interactive.tui_replay_start_paused_pty
        COMMAND
          "${Python3_EXECUTABLE}"
          "${CMAKE_CURRENT_SOURCE_DIR}/tests/run_tui_pty.py"
          "$<TARGET_FILE:test_tui_console>"
          --replay-start-paused
      )
      set_tests_properties(
        interactive.tui_console_pty
        interactive.tui_exception_pty
        interactive.tui_fault_hold_pty
        interactive.tui_replay_controls_pty
        interactive.tui_replay_start_paused_pty
        PROPERTIES TIMEOUT 15
      )

      if(TARGET mcl_planned_hierarchical_step)
        add_test(
          NAME apps.mcl_planned_hierarchical_step_tui_pages_pty
          COMMAND
            "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/run_planned_tui_pages_pty.py"
            "$<TARGET_FILE:mcl_planned_hierarchical_step>"
            6
            "/workspace/models/Psi_R1_visual_collision.urdf"
        )
        set_tests_properties(
          apps.mcl_planned_hierarchical_step_tui_pages_pty
          PROPERTIES TIMEOUT 20
        )
      endif()

      if(TARGET mcl_planned_hierarchical_step_otg)
        add_test(
          NAME apps.mcl_planned_hierarchical_step_otg_tui_pages_pty
          COMMAND
            "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/run_planned_tui_pages_pty.py"
            "$<TARGET_FILE:mcl_planned_hierarchical_step_otg>"
            7
            "/workspace/products/synrobot/modules/common/robot_description/psi_r1/urdf/Psi_R1_rev1.urdf"
        )
        set_tests_properties(
          apps.mcl_planned_hierarchical_step_otg_tui_pages_pty
          PROPERTIES TIMEOUT 20
        )
      endif()
    endif()

    foreach(MCL_DUAL_ARM_TARGET IN ITEMS
        mcl_step
        mcl_target)
      if(TARGET ${MCL_DUAL_ARM_TARGET})
        if(MCL_DUAL_ARM_TARGET STREQUAL "mcl_step")
          set(MCL_DUAL_ARM_SUBCOMMAND teleop)
        else()
          set(MCL_DUAL_ARM_SUBCOMMAND "")
        endif()
        add_test(
          NAME apps.${MCL_DUAL_ARM_TARGET}_solver_cli
          COMMAND
            "${CMAKE_COMMAND}"
            "-DAPP=$<TARGET_FILE:${MCL_DUAL_ARM_TARGET}>"
            "-DSUBCOMMAND=${MCL_DUAL_ARM_SUBCOMMAND}"
            -P "${CMAKE_CURRENT_SOURCE_DIR}/tests/check_dual_arm_solver_cli.cmake"
        )
      endif()
    endforeach()

    foreach(MCL_SERVO_TARGET IN ITEMS
        mcl_step
        mcl_hierarchical_step
        mcl_planned_hierarchical_step
        mcl_planned_hierarchical_step_otg)
      if(TARGET ${MCL_SERVO_TARGET})
        set(MCL_SERVO_PLANNED OFF)
        if(MCL_SERVO_TARGET STREQUAL "mcl_planned_hierarchical_step" OR
           MCL_SERVO_TARGET STREQUAL "mcl_planned_hierarchical_step_otg")
          set(MCL_SERVO_PLANNED ON)
        endif()
        add_test(
          NAME apps.${MCL_SERVO_TARGET}_subcommands
          COMMAND
            "${CMAKE_COMMAND}"
            "-DAPP=$<TARGET_FILE:${MCL_SERVO_TARGET}>"
            "-DPLANNED=${MCL_SERVO_PLANNED}"
            -P "${CMAKE_CURRENT_SOURCE_DIR}/tests/check_servo_subcommands.cmake"
        )
      endif()
    endforeach()

    if(TARGET mcl_step)
      add_executable(
        test_dual_arm_servo_solver_semantics
        tests/dual_arm_servo_solver_semantics.cpp
      )
      target_link_libraries(
        test_dual_arm_servo_solver_semantics
        PRIVATE
          motion_control_lab::step_support
          motion_control_lab::cpu_affinity
          motion_control_lab::app_helpers
          motion_control_lab::r1_robot_config
          motion_control_core::motion_control_core
          motion_control_lab::scheduler
          motion_control_lab::keyboard_input
          motion_control_lab::standard_ik_tui
          motion_control_lab::tui
          motion_control_lab::preview_transport
          motion_control_lab::preview_projection
          motion_control_lab::replay
          libplaco
      )
      add_test(
        NAME apps.mcl_step_semantics
        COMMAND test_dual_arm_servo_solver_semantics "${E01_MODEL}"
      )
      set_tests_properties(
        apps.mcl_step_semantics
        PROPERTIES
          TIMEOUT 20
          ENVIRONMENT_MODIFICATION
            "LD_LIBRARY_PATH=path_list_prepend:$<TARGET_FILE_DIR:libplaco>"
      )
    endif()

    if(TARGET mcl_target)
      add_executable(
        test_dual_arm_target_solver_semantics
        tests/dual_arm_target_solver_semantics.cpp
      )
      target_link_libraries(
        test_dual_arm_target_solver_semantics
        PRIVATE
          motion_control_lab::target_support
          motion_control_lab::cpu_affinity
          motion_control_lab::app_helpers
          motion_control_lab::r1_robot_config
          motion_control_core::motion_control_core
          motion_control_lab::scheduler
          motion_control_lab::keyboard_input
          motion_control_lab::standard_ik_tui
          motion_control_lab::tui
          motion_control_lab::preview_transport
          motion_control_lab::preview_projection
          libplaco
      )
      add_test(
        NAME apps.mcl_target_semantics
        COMMAND test_dual_arm_target_solver_semantics "${E01_MODEL}"
      )
      set_tests_properties(
        apps.mcl_target_semantics
        PROPERTIES
          TIMEOUT 20
          ENVIRONMENT_MODIFICATION
            "LD_LIBRARY_PATH=path_list_prepend:$<TARGET_FILE_DIR:libplaco>"
      )
    endif()

    if(MCL_ENABLE_TUI)
      foreach(MCL_DUAL_ARM_TARGET IN ITEMS
          mcl_step
          mcl_target)
        if(TARGET ${MCL_DUAL_ARM_TARGET})
          foreach(MCL_SOLVER_TEST IN ITEMS default mcc_proxqp mcc_eiquadprog placo_ignored_backend)
          if(MCL_SOLVER_TEST STREQUAL "default")
            set(MCL_SOLVER_ARGUMENT default)
            set(MCL_BACKEND_ARGUMENT default)
          elseif(MCL_SOLVER_TEST STREQUAL "mcc_proxqp")
            set(MCL_SOLVER_ARGUMENT mcc)
            set(MCL_BACKEND_ARGUMENT proxqp)
          elseif(MCL_SOLVER_TEST STREQUAL "mcc_eiquadprog")
            set(MCL_SOLVER_ARGUMENT mcc)
            set(MCL_BACKEND_ARGUMENT eiquadprog)
          else()
            set(MCL_SOLVER_ARGUMENT placo)
            set(MCL_BACKEND_ARGUMENT proxqp)
          endif()
          add_test(
            NAME apps.${MCL_DUAL_ARM_TARGET}_${MCL_SOLVER_TEST}_pty
            COMMAND
              "${Python3_EXECUTABLE}"
              "${CMAKE_CURRENT_SOURCE_DIR}/tests/run_dual_arm_solver_pty.py"
              "$<TARGET_FILE:${MCL_DUAL_ARM_TARGET}>"
              "${MCL_SOLVER_ARGUMENT}"
              "${MCL_BACKEND_ARGUMENT}"
              "${E01_MODEL}"
          )
          set_tests_properties(
            apps.${MCL_DUAL_ARM_TARGET}_${MCL_SOLVER_TEST}_pty
            PROPERTIES
              TIMEOUT 20
              ENVIRONMENT_MODIFICATION
                "LD_LIBRARY_PATH=path_list_prepend:$<TARGET_FILE_DIR:libplaco>"
          )
          endforeach()
        endif()
      endforeach()
    endif()
  endif()

  add_test(
    NAME contracts.e01_definition
    COMMAND
      "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/tests/validate_contracts.py"
      definition
      "${E01_DEFINITION}"
  )

  add_test(
    NAME contracts.e02_definition
    COMMAND
      "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/tests/validate_contracts.py"
      definition
      "${E02_DEFINITION}"
  )

  add_test(
    NAME contracts.e03_definition
    COMMAND
      "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/tests/validate_contracts.py"
      definition
      "${E03_DEFINITION}"
  )

  set(E04_DEFINITION_FOR_CONTRACT
      "${CMAKE_CURRENT_SOURCE_DIR}/experiments/E04_opensot_smoke/definition.json")
  add_test(
    NAME contracts.e04_definition
    COMMAND
      "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/tests/validate_contracts.py"
      definition
      "${E04_DEFINITION_FOR_CONTRACT}"
  )
  add_test(
    NAME contracts.r1_smoke_pair
    COMMAND
      "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/tests/validate_r1_smoke_pair.py"
      "${E01_DEFINITION}"
      "${E04_DEFINITION_FOR_CONTRACT}"
  )
  set_tests_properties(
    contracts.r1_smoke_pair
    PROPERTIES DEPENDS "contracts.e01_definition;contracts.e04_definition"
  )

  add_test(
    NAME experiments.e01_placo_smoke
    COMMAND
      "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/tests/run_e01_smoke.py"
      --executable "$<TARGET_FILE:e01_placo_smoke>"
      --definition "${E01_DEFINITION}"
      --work-root "${CMAKE_CURRENT_BINARY_DIR}/test-runs"
  )
  set_tests_properties(
    experiments.e01_placo_smoke
    PROPERTIES
      DEPENDS "contracts.e01_definition;contracts.r1_smoke_pair"
      ENVIRONMENT_MODIFICATION
        "LD_LIBRARY_PATH=path_list_prepend:$<TARGET_FILE_DIR:libplaco>"
  )

  if(MCL_BUILD_E04_OPENSOT_SMOKE)
    add_test(
      NAME experiments.e04_opensot_smoke
      COMMAND
        "${Python3_EXECUTABLE}"
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/run_e04_smoke.py"
        --executable "$<TARGET_FILE:e04_opensot_smoke>"
        --definition "${E04_DEFINITION}"
        --work-root "${CMAKE_CURRENT_BINARY_DIR}/test-runs"
        --ros-setup "${MCL_OPENSOT_ROS_PREFIX}/setup.bash"
        --library-path "${MCL_OPENSOT_RUNTIME_LIBRARY_PATH}"
    )
    set_tests_properties(
      experiments.e04_opensot_smoke
      PROPERTIES
        DEPENDS "contracts.e04_definition;contracts.r1_smoke_pair"
        TIMEOUT 120
    )
  endif()
endif()
