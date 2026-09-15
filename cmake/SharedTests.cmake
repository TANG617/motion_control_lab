# Shared component, data, and multi-app contract tests.
if(BUILD_TESTING)
  add_test(
    NAME experiments.e05_batch_replay
    COMMAND "${Python3_EXECUTABLE}" -m unittest discover
      -s "${CMAKE_CURRENT_SOURCE_DIR}/experiments/E05_real_scene_planned_mcap_batch_replay/tests" -v
  )
  set_tests_properties(experiments.e05_batch_replay PROPERTIES TIMEOUT 60)
  file(
    GLOB_RECURSE MCL_APP_SHELL_SCRIPTS
    CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.sh"
    "${CMAKE_CURRENT_SOURCE_DIR}/apps/*/scripts/*.sh"
    "${CMAKE_CURRENT_SOURCE_DIR}/experiments/*/scripts/*.sh"
  )
  add_test(NAME scripts.bash_syntax COMMAND "${MCL_BASH_EXECUTABLE}" -n ${MCL_APP_SHELL_SCRIPTS})


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
        mcl_hierarchical_kinematics_step)
      if(TARGET ${MCL_SERVO_TARGET})
        set(MCL_SERVO_PLANNED OFF)
        if(MCL_SERVO_TARGET STREQUAL "mcl_hierarchical_kinematics_step")
          set(MCL_SERVO_PLANNED ON)
          set(MCL_SERVO_PROFILE planned-otg-nullspace)
        else()
          set(MCL_SERVO_PROFILE "")
        endif()
        add_test(
          NAME apps.${MCL_SERVO_TARGET}_subcommands
          COMMAND
            "${CMAKE_COMMAND}"
            "-DAPP=$<TARGET_FILE:${MCL_SERVO_TARGET}>"
            "-DPLANNED=${MCL_SERVO_PLANNED}"
            "-DPROFILE=${MCL_SERVO_PROFILE}"
            -P "${CMAKE_CURRENT_SOURCE_DIR}/tests/check_servo_subcommands.cmake"
        )
      endif()
    endforeach()



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
              "${MCL_TEST_URDF}"
          )
          set_tests_properties(
            apps.${MCL_DUAL_ARM_TARGET}_${MCL_SOLVER_TEST}_pty
            PROPERTIES
              SKIP_RETURN_CODE 77
              TIMEOUT 20
              ENVIRONMENT_MODIFICATION
                "LD_LIBRARY_PATH=path_list_prepend:$<TARGET_FILE_DIR:libplaco>"
          )
          endforeach()
        endif()
      endforeach()
    endif()
  endif()

endif()
if(BUILD_TESTING)
  add_executable(test_execution_request tests/execution_request.cpp)
  target_link_libraries(test_execution_request PRIVATE motion_control_lab::run_artifacts)
  target_compile_options(test_execution_request PRIVATE -UNDEBUG)
  add_test(NAME execution.public_request COMMAND test_execution_request "${PROJECT_BINARY_DIR}/test-runs/execution-request")
  add_test(NAME execution.python_contracts COMMAND "${Python3_EXECUTABLE}" -m unittest discover -s "${PROJECT_SOURCE_DIR}/tools/app_execution/tests" -v)
endif()
if(BUILD_TESTING)
  add_test(NAME evidence.public_contracts
    COMMAND "${Python3_EXECUTABLE}" -m unittest discover
      -s "${PROJECT_SOURCE_DIR}/tools/mcc_placo_study/tests" -v)
  set_tests_properties(evidence.public_contracts PROPERTIES TIMEOUT 60)
endif()

if(BUILD_TESTING)
  add_test(NAME study.e09_measurements COMMAND "${Python3_EXECUTABLE}"
    "${PROJECT_SOURCE_DIR}/experiments/E09_solver_cost_and_scalability/test_measurements.py")
  add_test(NAME study.e10_schedule COMMAND "${Python3_EXECUTABLE}"
    "${PROJECT_SOURCE_DIR}/experiments/E10_multirate_scheduling_and_coupling/test_schedule.py")
endif()

if(BUILD_TESTING)
  foreach(number RANGE 6 12)
    if(number LESS 10)
      set(experiment_prefix E0${number})
    else()
      set(experiment_prefix E${number})
    endif()
    file(GLOB study_experiment_dirs "${PROJECT_SOURCE_DIR}/experiments/${experiment_prefix}_*")
    foreach(experiment_dir IN LISTS study_experiment_dirs)
      if(EXISTS "${experiment_dir}/tests")
        if(number LESS 9)
          add_test(NAME study.${experiment_prefix}_independent
            COMMAND "${Python3_EXECUTABLE}" -m pytest "${experiment_dir}/tests" -q)
        else()
          add_test(NAME study.${experiment_prefix}_independent
            COMMAND "${Python3_EXECUTABLE}" -m unittest discover -s "${experiment_dir}/tests" -v)
        endif()
      endif()
      add_test(NAME study.${experiment_prefix}_definition
        COMMAND "${Python3_EXECUTABLE}" "${PROJECT_SOURCE_DIR}/tests/validate_contracts.py"
          definition "${experiment_dir}/definition.json")
    endforeach()
  endforeach()
endif()

if(BUILD_TESTING AND TARGET mcl_optimization_problem)
  add_test(NAME execution.native_declaration
    COMMAND "${Python3_EXECUTABLE}" "${PROJECT_SOURCE_DIR}/tools/app_execution/tests/test_native_declaration.py"
      "$<TARGET_FILE:mcl_optimization_problem>")
  set_tests_properties(execution.native_declaration PROPERTIES TIMEOUT 30 RUN_SERIAL TRUE)
endif()
