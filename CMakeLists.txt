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

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(cmake/Dependencies.cmake)

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

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

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
