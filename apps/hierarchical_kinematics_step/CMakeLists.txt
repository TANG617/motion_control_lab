add_library(
  mcl_hierarchical_kinematics_step_support
  options.cpp
  execution.cpp
  allocation.cpp
  elbow_reference.cpp
  elbow_reference_visualization.cpp
  admittance.cpp
  solver.cpp
  nullspace.cpp
  planning.cpp
  rejection_policy.cpp
  telemetry.cpp
  loop.cpp
)
add_library(
  motion_control_lab::hierarchical_kinematics_step_support
  ALIAS mcl_hierarchical_kinematics_step_support
)
target_include_directories(
  mcl_hierarchical_kinematics_step_support
  PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>"
)
target_compile_definitions(
  mcl_hierarchical_kinematics_step_support
  PRIVATE
    MCL_R1_MUJOCO_MODEL_PATH="${PROJECT_SOURCE_DIR}/assets/robots/r1/mujoco/mjcf/r1.xml"
)
target_link_libraries(
  mcl_hierarchical_kinematics_step_support
  PUBLIC
    motion_control_core::motion_control_core
    motion_control_sim::mujoco
    motion_control_sim::viewer
    motion_control_lab::run_artifacts
    motion_control_lab::cpu_affinity
    motion_control_lab::app_helpers
    motion_control_lab::scheduler
    motion_control_lab::keyboard_input
    motion_control_lab::planned_grouped_tui
    motion_control_lab::preview_transport
    motion_control_lab::preview_projection
    motion_control_lab::visualization_contracts
    motion_control_lab::telemetry_proto
    motion_control_lab::replay
    Threads::Threads
)

mcl_add_app(
  TARGET mcl_hierarchical_kinematics_step
  SOURCES main.cpp
  LIBRARIES motion_control_lab::hierarchical_kinematics_step_support
)
install(
  DIRECTORY
    scripts/_launcher
    scripts/profiles
    scripts/harp
  DESTINATION
    "${CMAKE_INSTALL_LIBEXECDIR}/motion-control-lab/mcl_hierarchical_kinematics_step"
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING
    PATTERN "*.py"
    PATTERN "requirements.txt"
    PATTERN "__pycache__" EXCLUDE
)

install(DIRECTORY foxglove/
  DESTINATION "${CMAKE_INSTALL_LIBEXECDIR}/motion-control-lab/mcl_hierarchical_kinematics_step/foxglove"
  FILES_MATCHING PATTERN "*.json")
install(FILES HARP.md
  DESTINATION "${CMAKE_INSTALL_LIBEXECDIR}/motion-control-lab/mcl_hierarchical_kinematics_step")

if(BUILD_TESTING)
  add_test(NAME apps.mcl_hierarchical_kinematics_step_show_com_contract
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/show_com_contract.py"
    --binary $<TARGET_FILE:mcl_hierarchical_kinematics_step>)
  add_executable(test_hierarchical_kinematics_step_replay_accounting tests/replay_accounting.cpp)
  target_link_libraries(test_hierarchical_kinematics_step_replay_accounting PRIVATE motion_control_lab::hierarchical_kinematics_step_support)
  add_test(NAME apps.mcl_hierarchical_kinematics_step_replay_accounting COMMAND test_hierarchical_kinematics_step_replay_accounting)
  add_test(NAME apps.mcl_hierarchical_kinematics_step_execution_contract
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/execution_contract.py"
    --binary $<TARGET_FILE:mcl_hierarchical_kinematics_step>)
  add_executable(test_hierarchical_kinematics_step_tcp_mirror tests/tcp_mirror.cpp)
  target_link_libraries(test_hierarchical_kinematics_step_tcp_mirror PRIVATE motion_control_lab::hierarchical_kinematics_step_support)
  add_test(NAME apps.mcl_hierarchical_kinematics_step_tcp_mirror COMMAND test_hierarchical_kinematics_step_tcp_mirror)

  add_executable(test_hierarchical_kinematics_step_elbow_reference_visualization tests/elbow_reference_visualization.cpp)
  target_link_libraries(test_hierarchical_kinematics_step_elbow_reference_visualization PRIVATE motion_control_lab::hierarchical_kinematics_step_support)
  add_test(NAME apps.mcl_hierarchical_kinematics_step_elbow_reference_visualization COMMAND test_hierarchical_kinematics_step_elbow_reference_visualization)

  add_executable(test_hierarchical_kinematics_step_elbow_reference tests/elbow_reference.cpp)
  target_link_libraries(test_hierarchical_kinematics_step_elbow_reference PRIVATE motion_control_lab::hierarchical_kinematics_step_support)
  add_test(NAME apps.mcl_hierarchical_kinematics_step_elbow_geometry COMMAND test_hierarchical_kinematics_step_elbow_reference)
  add_executable(
    test_hierarchical_kinematics_step_app_options
    tests/app_options_contract.cpp
    options.cpp
    planning.cpp
  )
  target_link_libraries(
    test_hierarchical_kinematics_step_app_options
    PRIVATE
      motion_control_lab::input_contract
      motion_control_lab::scheduler
      motion_control_lab::replay
      motion_control_lab::tui
      motion_control_core::motion_control_core
  )
  target_compile_definitions(
    test_hierarchical_kinematics_step_app_options
    PRIVATE
      MCL_R1_MUJOCO_MODEL_PATH="${PROJECT_SOURCE_DIR}/assets/robots/r1/mujoco/mjcf/r1.xml"
  )
  add_test(
    NAME apps.mcl_hierarchical_kinematics_step_options
    COMMAND test_hierarchical_kinematics_step_app_options
  )

  add_executable(
    test_hierarchical_kinematics_step_admittance
    tests/admittance.cpp
    admittance.cpp
  )
  target_link_libraries(
    test_hierarchical_kinematics_step_admittance
    PRIVATE
      motion_control_core::motion_control_core
      motion_control_sim::mujoco
      motion_control_lab::replay
  )
  target_include_directories(
    test_hierarchical_kinematics_step_admittance
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}" "${PROJECT_SOURCE_DIR}"
  )
  add_test(
    NAME apps.mcl_hierarchical_kinematics_step_admittance
    COMMAND test_hierarchical_kinematics_step_admittance
  )

  add_executable(
    test_hierarchical_kinematics_step_r1_model_consistency
    tests/r1_model_consistency.cpp
  )
  target_link_libraries(
    test_hierarchical_kinematics_step_r1_model_consistency
    PRIVATE motion_control_sim::mujoco pinocchio::pinocchio
  )
  add_test(
    NAME apps.mcl_hierarchical_kinematics_step_r1_model_consistency
    COMMAND
      test_hierarchical_kinematics_step_r1_model_consistency
      "${PROJECT_SOURCE_DIR}/assets/robots/r1/mujoco/urdf/r1.urdf"
      "${PROJECT_SOURCE_DIR}/assets/robots/r1/mujoco/mjcf/r1.xml"
  )
  add_test(
    NAME apps.mcl_hierarchical_kinematics_step_r1_model_structure
    COMMAND "${Python3_EXECUTABLE}"
      "${PROJECT_SOURCE_DIR}/assets/robots/r1/mujoco/verify_structure.py"
      "${PROJECT_SOURCE_DIR}/assets/robots/r1/mujoco/mjcf/r1.xml"
  )

  add_executable(test_hierarchical_kinematics_step_solver tests/solver.cpp)
  target_link_libraries(
    test_hierarchical_kinematics_step_solver
    PRIVATE motion_control_lab::hierarchical_kinematics_step_support
  )
  add_test(
    NAME apps.mcl_hierarchical_kinematics_step_solver
    COMMAND
      test_hierarchical_kinematics_step_solver
      "/workspace/models/Psi_R1_visual_collision.urdf"
  )

  add_executable(
    test_hierarchical_kinematics_step_control
    tests/nullspace_control.cpp
  )
  target_link_libraries(
    test_hierarchical_kinematics_step_control
    PRIVATE motion_control_lab::hierarchical_kinematics_step_support
  )
  add_test(
    NAME apps.mcl_hierarchical_kinematics_step_control
    COMMAND test_hierarchical_kinematics_step_control
  )

  add_executable(test_hierarchical_kinematics_step_joint_target tests/joint_target_processing.cpp)
  target_link_libraries(
    test_hierarchical_kinematics_step_joint_target
    PRIVATE motion_control_lab::hierarchical_kinematics_step_support
  )
  add_test(
    NAME apps.mcl_hierarchical_kinematics_step_joint_target
    COMMAND test_hierarchical_kinematics_step_joint_target
  )

  add_executable(test_hierarchical_kinematics_step_retarget_clamp tests/retarget_state_clamp.cpp)
  target_link_libraries(
    test_hierarchical_kinematics_step_retarget_clamp
    PRIVATE motion_control_lab::hierarchical_kinematics_step_support
  )
  add_test(
    NAME apps.mcl_hierarchical_kinematics_step_retarget_clamp
    COMMAND test_hierarchical_kinematics_step_retarget_clamp
  )

  add_executable(test_hierarchical_kinematics_step_visualization tests/visualization.cpp)
  target_link_libraries(
    test_hierarchical_kinematics_step_visualization
    PRIVATE motion_control_lab::hierarchical_kinematics_step_support
  )
  add_test(
    NAME apps.mcl_hierarchical_kinematics_step_visualization
    COMMAND test_hierarchical_kinematics_step_visualization
  )

  add_executable(test_hierarchical_kinematics_step_telemetry tests/telemetry.cpp)
  target_link_libraries(
    test_hierarchical_kinematics_step_telemetry
    PRIVATE motion_control_lab::hierarchical_kinematics_step_support
  )
  add_test(
    NAME apps.mcl_hierarchical_kinematics_step_telemetry
    COMMAND test_hierarchical_kinematics_step_telemetry
  )

  add_executable(
    test_hierarchical_kinematics_step_tui_projection
    tests/tui_projection.cpp
  )
  target_link_libraries(
    test_hierarchical_kinematics_step_tui_projection
    PRIVATE motion_control_lab::hierarchical_kinematics_step_support
  )
  add_test(
    NAME apps.mcl_hierarchical_kinematics_step_tui_projection
    COMMAND test_hierarchical_kinematics_step_tui_projection
  )

  add_executable(
    test_hierarchical_kinematics_step_rejection_policy
    tests/rejection_policy.cpp
  )
  target_link_libraries(
    test_hierarchical_kinematics_step_rejection_policy
    PRIVATE motion_control_lab::hierarchical_kinematics_step_support
  )
  add_test(
    NAME apps.mcl_hierarchical_kinematics_step_rejection_policy
    COMMAND test_hierarchical_kinematics_step_rejection_policy
  )

  add_executable(
    test_hierarchical_kinematics_step_replay_pipeline_gate
    tests/replay_pipeline_gate.cpp
  )
  target_link_libraries(
    test_hierarchical_kinematics_step_replay_pipeline_gate
    PRIVATE motion_control_lab::hierarchical_kinematics_step_support
  )
  add_test(
    NAME apps.mcl_hierarchical_kinematics_step_replay_pipeline_gate
    COMMAND test_hierarchical_kinematics_step_replay_pipeline_gate
  )

endif()

if(MCL_BUILD_HARP)
  target_link_libraries(mcl_hierarchical_kinematics_step_support PRIVATE motion_control_core::posture_reference)
  target_compile_definitions(mcl_hierarchical_kinematics_step_support PRIVATE MCL_HAS_HARP=1)
  if(BUILD_TESTING)
    add_executable(test_hierarchical_kinematics_step_harp_lifecycle tests/harp_lifecycle.cpp)
    target_link_libraries(test_hierarchical_kinematics_step_harp_lifecycle
      PRIVATE motion_control_lab::hierarchical_kinematics_step_support)
    set(MCL_HARP_TEST_MODEL "" CACHE PATH "Optional model for native HARP lifecycle acceptance")
    set(MCL_HARP_TEST_DEVICE "cpu" CACHE STRING "Native HARP lifecycle device")
    if(MCL_HARP_TEST_MODEL)
      add_test(NAME apps.mcl_hierarchical_kinematics_step_harp_lifecycle
        COMMAND test_hierarchical_kinematics_step_harp_lifecycle "${MCL_HARP_TEST_MODEL}"
        "${CMAKE_CURRENT_BINARY_DIR}/harp_lifecycle.jsonl" "${MCL_HARP_TEST_DEVICE}")
    endif()
  endif()
endif()

if(BUILD_TESTING)
  if(TARGET mcl_hierarchical_kinematics_step)
    add_test(
      NAME scripts.hierarchical_kinematics_source_recipe_path_resolution
      COMMAND
        "${CMAKE_COMMAND}" -E env
        --unset=MCL_BINARY
        "PATH=$<TARGET_FILE_DIR:mcl_hierarchical_kinematics_step>:$ENV{PATH}"
        "MCL_CPU_SET="
        "MCL_RT_PRIORITY="
        "${PROJECT_SOURCE_DIR}/apps/hierarchical_kinematics_step/scripts/profiles/planned_otg_nullspace_admittance_kinematic_sim/run_keyboard.py"
        --help
    )
    add_test(
      NAME scripts.hierarchical_kinematics_mcap_preset_smoke
      COMMAND
        "${CMAKE_COMMAND}" -E env
        "MCL_BINARY=${MCL_TRUE_EXECUTABLE}"
        "MCL_CPU_SET="
        "MCL_RT_PRIORITY="
        "${PROJECT_SOURCE_DIR}/apps/hierarchical_kinematics_step/scripts/profiles/planned_otg_nullspace_admittance_kinematic_sim/run_mcap_interactive.py"
    )
    add_test(
      NAME scripts.hierarchical_kinematics_csv_preset_smoke
      COMMAND
        "${CMAKE_COMMAND}" -E env
        "MCL_BINARY=${MCL_TRUE_EXECUTABLE}"
        "MCL_CPU_SET="
        "MCL_RT_PRIORITY="
        "${PROJECT_SOURCE_DIR}/apps/hierarchical_kinematics_step/scripts/profiles/planned_otg_nullspace_admittance_kinematic_sim/run_csv_batch.py"
        --input /tmp/mcl-nullspace-admittance-csv-launcher-smoke.csv
    )
    if(MCL_SCRIPT_EXECUTABLE)
      add_test(
        NAME scripts.hierarchical_kinematics_keyboard_smoke
        COMMAND
          "${CMAKE_COMMAND}" -E env
          "MCL_BINARY=$<TARGET_FILE:mcl_hierarchical_kinematics_step>"
          "MCL_CPU_SET="
          "MCL_RT_PRIORITY="
          "${MCL_SCRIPT_EXECUTABLE}" -qec
          "${PROJECT_SOURCE_DIR}/apps/hierarchical_kinematics_step/scripts/profiles/planned_otg_nullspace_admittance_kinematic_sim/run_keyboard.py --no-mujoco-viewer --ui none --viz none --duration 0.05"
          /dev/null
      )
      set_tests_properties(
        scripts.hierarchical_kinematics_keyboard_smoke
        PROPERTIES TIMEOUT 15
      )
    endif()
  endif()
      if(MCL_ENABLE_TUI)
        add_test(
          NAME apps.mcl_hierarchical_kinematics_step_tui_pages_pty
          COMMAND
            "${Python3_EXECUTABLE}"
            "${PROJECT_SOURCE_DIR}/tests/run_planned_tui_pages_pty.py"
            "$<TARGET_FILE:mcl_hierarchical_kinematics_step>"
            6
            "/workspace/products/synrobot/modules/common/robot_description/psi_r1/urdf/Psi_R1_rev1.urdf"
            --profile planned-otg-nullspace
        )
        set_tests_properties(
          apps.mcl_hierarchical_kinematics_step_tui_pages_pty
          PROPERTIES TIMEOUT 20
        )
      endif()
endif()

install(DIRECTORY config/ DESTINATION "${CMAKE_INSTALL_LIBEXECDIR}/motion-control-lab/mcl_hierarchical_kinematics_step/config")
