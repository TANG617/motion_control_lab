add_library(
  mcl_hierarchical_inverse_dynamics_torque_sim_support
  solver.cpp
  loop.cpp
)
add_library(
  motion_control_lab::hierarchical_inverse_dynamics_torque_sim_support
  ALIAS mcl_hierarchical_inverse_dynamics_torque_sim_support
)
target_include_directories(
  mcl_hierarchical_inverse_dynamics_torque_sim_support
  PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>"
)
target_link_libraries(
  mcl_hierarchical_inverse_dynamics_torque_sim_support
  PUBLIC
    motion_control_core::motion_control_core
    motion_control_sim::mujoco
    motion_control_sim::viewer
    motion_control_lab::keyboard_input
    motion_control_lab::planned_grouped_tui
    motion_control_lab::preview_projection
    motion_control_lab::preview_transport
    motion_control_lab::r1_robot_config
    motion_control_lab::visualization_contracts
)

mcl_add_app(
  TARGET mcl_hierarchical_inverse_dynamics_torque_sim
  SOURCES main.cpp options.cpp
  LIBRARIES motion_control_lab::hierarchical_inverse_dynamics_torque_sim_support
)
target_compile_definitions(
  mcl_hierarchical_inverse_dynamics_torque_sim
  PRIVATE
    MCL_R1_MUJOCO_MODEL_PATH="${PROJECT_SOURCE_DIR}/assets/robots/r1/mujoco/mjcf/r1.xml"
    MCL_R1_MUJOCO_URDF_PATH="${PROJECT_SOURCE_DIR}/assets/robots/r1/mujoco/urdf/r1.urdf"
)

if(BUILD_TESTING)
  add_executable(
    test_hierarchical_inverse_dynamics_torque_sim_r1_model_consistency
    tests/r1_model_consistency.cpp
  )
  target_link_libraries(
    test_hierarchical_inverse_dynamics_torque_sim_r1_model_consistency
    PRIVATE
      motion_control_core::motion_control_core
      motion_control_sim::mujoco
      motion_control_lab::r1_robot_config
  )
  add_test(
    NAME apps.mcl_hierarchical_inverse_dynamics_torque_sim_r1_model_consistency
    COMMAND
      test_hierarchical_inverse_dynamics_torque_sim_r1_model_consistency
      "${PROJECT_SOURCE_DIR}/assets/robots/r1/mujoco/urdf/r1.urdf"
      "${PROJECT_SOURCE_DIR}/assets/robots/r1/mujoco/mjcf/r1.xml"
  )

  add_test(
    NAME apps.mcl_hierarchical_inverse_dynamics_torque_sim_headless
    COMMAND mcl_hierarchical_inverse_dynamics_torque_sim --duration 0.005 --retarget-x 0
      --ui none --viz none --no-mujoco-viewer
  )
  set_tests_properties(
    apps.mcl_hierarchical_inverse_dynamics_torque_sim_headless
    PROPERTIES TIMEOUT 20
  )
  add_test(
    NAME apps.mcl_hierarchical_inverse_dynamics_torque_sim_paused_single_step
    COMMAND mcl_hierarchical_inverse_dynamics_torque_sim --paused --single-steps 1 --retarget-x 0
      --ui none --viz none --no-mujoco-viewer
  )
  set_tests_properties(
    apps.mcl_hierarchical_inverse_dynamics_torque_sim_paused_single_step
    PROPERTIES TIMEOUT 20
  )

  find_program(MCL_HID_TORQUE_SCRIPT_EXECUTABLE script)
  if(MCL_HID_TORQUE_SCRIPT_EXECUTABLE)
    add_test(
      NAME apps.mcl_hierarchical_inverse_dynamics_torque_sim_keyboard
      COMMAND
        "${CMAKE_COMMAND}" -E env
        "MCL_BINARY=$<TARGET_FILE:mcl_hierarchical_inverse_dynamics_torque_sim>"
        "MCL_CPU_SET="
        "MCL_RT_PRIORITY="
        "MCL_URDF=${PROJECT_SOURCE_DIR}/assets/robots/r1/mujoco/urdf/r1.urdf"
        "MCL_MUJOCO_MODEL=${PROJECT_SOURCE_DIR}/assets/robots/r1/mujoco/mjcf/r1.xml"
        "MCL_UI=none"
        "MCL_VIZ=none"
        "MCL_MUJOCO_VIEWER=off"
        "${MCL_HID_TORQUE_SCRIPT_EXECUTABLE}" -qec
        "${CMAKE_CURRENT_SOURCE_DIR}/scripts/run_keyboard.sh --duration 0.005"
        /dev/null
    )
    set_tests_properties(
      apps.mcl_hierarchical_inverse_dynamics_torque_sim_keyboard
      PROPERTIES TIMEOUT 20
    )
  endif()

  if(MCL_ENABLE_TUI)
    add_test(
      NAME apps.mcl_hierarchical_inverse_dynamics_torque_sim_tui_pty
      COMMAND
        "${Python3_EXECUTABLE}"
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/run_tui_pty.py"
        "$<TARGET_FILE:mcl_hierarchical_inverse_dynamics_torque_sim>"
        "${PROJECT_SOURCE_DIR}/assets/robots/r1/mujoco/urdf/r1.urdf"
        "${PROJECT_SOURCE_DIR}/assets/robots/r1/mujoco/mjcf/r1.xml"
    )
    set_tests_properties(
      apps.mcl_hierarchical_inverse_dynamics_torque_sim_tui_pty
      PROPERTIES TIMEOUT 20
    )
  endif()
endif()
