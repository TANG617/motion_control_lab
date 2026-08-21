add_library(
  mcl_baseline_support
  solver.cpp
  loop.cpp
)
add_library(
  motion_control_lab::baseline_support
  ALIAS mcl_baseline_support
)
target_include_directories(
  mcl_baseline_support
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(
  mcl_baseline_support
  PUBLIC
    motion_control_lab::presentation_contract
    libplaco
    jsoncpp_lib
    motion_control_lab::cpu_affinity
    motion_control_lab::app_helpers
    motion_control_lab::r1_robot_config
    motion_control_lab::scheduler
    motion_control_lab::keyboard_input
    motion_control_lab::standard_ik_tui
    motion_control_lab::tui
    motion_control_lab::preview_transport
    motion_control_lab::preview_projection
    motion_control_lab::replay
    motion_control_lab::run_artifacts
)

mcl_add_app(
  TARGET mcl_baseline
  SOURCES main.cpp options.cpp
  LIBRARIES motion_control_lab::baseline_support
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
if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  set(MCL_BASELINE_TEST_MODEL "/workspace/models/r1.cos.urdf")
  add_executable(
    test_baseline_app_options
    tests/app_options_contract.cpp
    options.cpp
  )
  target_link_libraries(
    test_baseline_app_options
    PRIVATE
      motion_control_lab::baseline_support
      motion_control_lab::replay
      motion_control_lab::tui
      motion_control_lab::r1_robot_config
  )
  add_test(
    NAME apps.mcl_baseline_options
    COMMAND test_baseline_app_options
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
    PRIVATE motion_control_lab::baseline_support motion_control_lab::run_artifacts
  )
  add_test(
    NAME apps.mcl_baseline_config_contract
    COMMAND test_baseline_config_contract
  )

  add_executable(
    test_baseline_tui_projection
    tests/tui_projection.cpp
  )
  target_link_libraries(
    test_baseline_tui_projection
    PRIVATE motion_control_lab::standard_ik_tui
  )
  add_test(
    NAME apps.mcl_baseline_tui_projection
    COMMAND test_baseline_tui_projection
  )

  add_executable(
    test_baseline_solver_semantics
    tests/baseline_solver_semantics.cpp
  )
  target_link_libraries(
    test_baseline_solver_semantics
    PRIVATE motion_control_lab::baseline_support
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

  if(MCL_ENABLE_TUI)
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
endif()
