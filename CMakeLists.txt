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
  MCL_BUILD_SINGLE_ARM_IK
  "Build the single-arm interactive IK application"
  ON
)
option(
  MCL_BUILD_DUAL_ARM_IK
  "Build the dual-arm interactive IK application"
  ON
)
option(
  MCL_BUILD_GROUPED_DUAL_ARM_IK
  "Build the Red/Yellow/Green dual-arm IK application"
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
  OFF
)

set(MCL_BUILD_ANY_INTERACTIVE_IK OFF)
if(MCL_BUILD_SINGLE_ARM_IK OR MCL_BUILD_DUAL_ARM_IK OR MCL_BUILD_GROUPED_DUAL_ARM_IK)
  set(MCL_BUILD_ANY_INTERACTIVE_IK ON)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(cmake/Dependencies.cmake)

if(MCL_BUILD_ANY_INTERACTIVE_IK OR MCL_BUILD_MCC_PLOTS OR MCL_BUILD_CARTESIAN_PLANNING)
  find_package(motion_control_core CONFIG REQUIRED)
endif()

if(MCL_BUILD_ANY_INTERACTIVE_IK OR MCL_BUILD_CARTESIAN_PLANNING)
  find_package(motion_control_viz 0.3 CONFIG REQUIRED)
endif()

if(MCL_BUILD_ANY_INTERACTIVE_IK)

  add_library(
    mcl_interactive_support
    adapters/interactive/src/config/interactive_ik_options.cpp
    adapters/interactive/src/runtime/grouped_worker.cpp
    adapters/interactive/src/runtime/interactive_scheduler.cpp
    adapters/interactive/src/sinks/ik_visualization.cpp
    adapters/interactive/src/sinks/visualization_sink_factory.cpp
    adapters/interactive/src/teleop/tui_teleop_source.cpp
  )
  add_library(motion_control_lab::interactive_support ALIAS mcl_interactive_support)
  target_include_directories(
    mcl_interactive_support
    PUBLIC
      "${CMAKE_CURRENT_SOURCE_DIR}/adapters/interactive/src"
  )
  target_link_libraries(
    mcl_interactive_support
    PUBLIC
      Eigen3::Eigen
      motion_control_viz::motion_control_viz
  )

  if(TARGET motion_control_viz::foxglove_sink)
    target_link_libraries(
      mcl_interactive_support
      PUBLIC motion_control_viz::foxglove_sink
    )
    target_compile_definitions(
      mcl_interactive_support
      PRIVATE MCL_WITH_FOXGLOVE_SINK=1
    )
  else()
    target_compile_definitions(
      mcl_interactive_support
      PRIVATE MCL_WITH_FOXGLOVE_SINK=0
    )
  endif()

  add_subdirectory(apps/common)

  if(MCL_BUILD_SINGLE_ARM_IK)
    add_subdirectory(apps/single_arm_ik)
  endif()
  if(MCL_BUILD_DUAL_ARM_IK)
    add_subdirectory(apps/dual_arm_ik)
  endif()
  if(MCL_BUILD_GROUPED_DUAL_ARM_IK)
    add_subdirectory(apps/grouped_dual_arm_ik)
  endif()
endif()

if(MCL_BUILD_CARTESIAN_PLANNING)
  if(NOT TARGET motion_control_viz::foxglove_sink)
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

  add_executable(
    motion_control_lab_plot_core_planning
    apps/plot_core_planning.cpp
  )
  target_include_directories(
    motion_control_lab_plot_core_planning
    SYSTEM PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/third_party/matplotlib-cpp"
  )
  target_compile_definitions(
    motion_control_lab_plot_core_planning
    PRIVATE MATPLOTLIBCPP_PYTHON_EXECUTABLE="${Python3_EXECUTABLE}"
  )
  target_link_libraries(
    motion_control_lab_plot_core_planning
    PRIVATE
      motion_control_core::motion_control_core
      Python3::Python
      Python3::NumPy
  )
endif()

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
set(E01_MODEL "${E01_DIRECTORY}/model/two_link.urdf")
set(MCL_PLACO_UPSTREAM_REVISION "e6c288604639d67b979a16cb2ad26913413c8e3a")
set(
  MCL_RUNTIME_DESCRIPTION
  "C++17;${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR};${CMAKE_CXX_COMPILER_ID}-${CMAKE_CXX_COMPILER_VERSION}"
)

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
if(MCL_BUILD_MCC_PLOTS)
  list(APPEND MCL_INSTALL_RUNTIME_TARGETS motion_control_lab_plot_core_planning)
endif()
install(
  TARGETS ${MCL_INSTALL_RUNTIME_TARGETS}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  if(MCL_BUILD_ANY_INTERACTIVE_IK)
    add_executable(
      test_interactive_ik_configuration
      tests/interactive_ik_configuration.cpp
    )
    target_link_libraries(
      test_interactive_ik_configuration
      PRIVATE
        motion_control_lab::app_common
        motion_control_lab::interactive_support
    )
    add_test(
      NAME interactive.ik_configuration
      COMMAND test_interactive_ik_configuration
    )

    add_executable(
      test_grouped_runtime
      tests/grouped_runtime.cpp
    )
    target_link_libraries(
      test_grouped_runtime
      PRIVATE
        motion_control_lab::interactive_support
        Threads::Threads
    )
    add_test(
      NAME interactive.grouped_runtime
      COMMAND test_grouped_runtime
    )
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
    PROPERTIES DEPENDS contracts.e01_definition
  )
endif()
