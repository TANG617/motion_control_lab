add_library(
  mcl_baseline_runtime
  baseline_config.cpp
  baseline_solver.cpp
)
target_include_directories(
  mcl_baseline_runtime
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(
  mcl_baseline_runtime
  PUBLIC
    motion_control_lab::interactive_support
    libplaco
    jsoncpp_lib
)

add_executable(mcl_baseline main.cpp)
set_target_properties(
  mcl_baseline
  PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
)
if(APPLE)
  set_property(
    TARGET mcl_baseline
    PROPERTY INSTALL_RPATH "@loader_path/../${CMAKE_INSTALL_LIBDIR}"
  )
else()
  set_property(
    TARGET mcl_baseline
    PROPERTY INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}"
  )
  target_link_options(
    mcl_baseline
    PRIVATE "LINKER:--disable-new-dtags"
  )
endif()
target_link_libraries(
  mcl_baseline
  PRIVATE
    mcl_baseline_runtime
    motion_control_lab::cpu_affinity
    motion_control_lab::app_common
    motion_control_lab::replay_support
    motion_control_lab::run_artifacts
)
install(
  TARGETS mcl_baseline
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  set(MCL_BASELINE_TEST_MODEL "/workspace/models/r1.cos.urdf")
  add_test(
    NAME apps.mcl_baseline_help
    COMMAND
      "${CMAKE_COMMAND}"
      "-DAPP=$<TARGET_FILE:mcl_baseline>"
      -P "${PROJECT_SOURCE_DIR}/tests/check_app_help.cmake"
  )
  add_test(
    NAME apps.mcl_baseline_subcommands
    COMMAND
      "${CMAKE_COMMAND}"
      "-DAPP=$<TARGET_FILE:mcl_baseline>"
      -P "${CMAKE_CURRENT_SOURCE_DIR}/tests/check_baseline_cli.cmake"
  )

  add_executable(
    test_baseline_config_contract
    tests/baseline_config_contract.cpp
  )
  target_link_libraries(
    test_baseline_config_contract
    PRIVATE mcl_baseline_runtime motion_control_lab::run_artifacts
  )
  add_test(
    NAME apps.mcl_baseline_config_contract
    COMMAND test_baseline_config_contract
  )

  add_executable(
    test_baseline_solver_semantics
    tests/baseline_solver_semantics.cpp
  )
  target_link_libraries(
    test_baseline_solver_semantics
    PRIVATE mcl_baseline_runtime
  )
  add_test(
    NAME apps.mcl_baseline_solver_semantics
    COMMAND test_baseline_solver_semantics "${MCL_BASELINE_TEST_MODEL}"
  )
  set_tests_properties(
    apps.mcl_baseline_solver_semantics
    PROPERTIES
      TIMEOUT 20
      ENVIRONMENT_MODIFICATION
        "LD_LIBRARY_PATH=path_list_prepend:$<TARGET_FILE_DIR:libplaco>"
  )

  add_test(
    NAME apps.mcl_baseline_replay_determinism
    COMMAND
      "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/tests/replay_determinism.py"
      "$<TARGET_FILE:mcl_baseline>"
      "${MCL_BASELINE_TEST_MODEL}"
      "${CMAKE_CURRENT_SOURCE_DIR}/tests/baseline_replay.csv"
  )
  set_tests_properties(
    apps.mcl_baseline_replay_determinism
    PROPERTIES
      TIMEOUT 30
      ENVIRONMENT_MODIFICATION
        "LD_LIBRARY_PATH=path_list_prepend:$<TARGET_FILE_DIR:libplaco>"
  )

  add_test(
    NAME apps.mcl_baseline_teleop_pty
    COMMAND
      "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/tests/run_baseline_pty.py"
      "$<TARGET_FILE:mcl_baseline>"
      "${MCL_BASELINE_TEST_MODEL}"
  )
  set_tests_properties(
    apps.mcl_baseline_teleop_pty
    PROPERTIES
      TIMEOUT 20
      ENVIRONMENT_MODIFICATION
        "LD_LIBRARY_PATH=path_list_prepend:$<TARGET_FILE_DIR:libplaco>"
  )
endif()
