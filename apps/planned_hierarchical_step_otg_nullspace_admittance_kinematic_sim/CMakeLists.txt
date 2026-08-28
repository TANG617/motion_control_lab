add_library(
  mcl_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_support
  admittance.cpp
  solver.cpp
  nullspace.cpp
  planning.cpp
  rejection_policy.cpp
  telemetry.cpp
  loop.cpp
)
add_library(
  motion_control_lab::planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_support
  ALIAS mcl_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_support
)
target_include_directories(
  mcl_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_support
  PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>"
)
target_link_libraries(
  mcl_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_support
  PUBLIC
    motion_control_core::motion_control_core
    motion_control_sim::mujoco
    motion_control_sim::viewer
    motion_control_lab::cpu_affinity
    motion_control_lab::app_helpers
    motion_control_lab::r1_robot_config
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
  TARGET mcl_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim
  SOURCES main.cpp options.cpp
  LIBRARIES motion_control_lab::planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_support
)
target_compile_definitions(
  mcl_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim
  PRIVATE
    MCL_R1_MUJOCO_MODEL_PATH="${PROJECT_SOURCE_DIR}/assets/robots/r1/mujoco/mjcf/r1.xml"
    MCL_R1_MUJOCO_URDF_PATH="${PROJECT_SOURCE_DIR}/assets/robots/r1/mujoco/urdf/r1.urdf"
)

if(BUILD_TESTING)
  add_executable(
    test_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_app_options
    tests/app_options_contract.cpp
    options.cpp
  )
  target_link_libraries(
    test_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_app_options
    PRIVATE
      motion_control_lab::input_contract
      motion_control_lab::scheduler
      motion_control_lab::replay
      motion_control_lab::tui
      motion_control_lab::r1_robot_config
      motion_control_core::motion_control_core
  )
  target_compile_definitions(
    test_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_app_options
    PRIVATE
      MCL_R1_MUJOCO_MODEL_PATH="${PROJECT_SOURCE_DIR}/assets/robots/r1/mujoco/mjcf/r1.xml"
      MCL_R1_MUJOCO_URDF_PATH="${PROJECT_SOURCE_DIR}/assets/robots/r1/mujoco/urdf/r1.urdf"
  )
  add_test(
    NAME apps.mcl_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_options
    COMMAND test_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_app_options
  )

  add_executable(
    test_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_admittance
    tests/admittance.cpp
    admittance.cpp
  )
  target_link_libraries(
    test_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_admittance
    PRIVATE
      motion_control_core::motion_control_core
      motion_control_sim::mujoco
  )
  target_include_directories(
    test_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_admittance
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}" "${PROJECT_SOURCE_DIR}"
  )
  add_test(
    NAME apps.mcl_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_admittance
    COMMAND test_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_admittance
  )

  add_executable(
    test_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_r1_model_consistency
    tests/r1_model_consistency.cpp
  )
  target_link_libraries(
    test_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_r1_model_consistency
    PRIVATE motion_control_sim::mujoco pinocchio::pinocchio
  )
  add_test(
    NAME apps.mcl_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_r1_model_consistency
    COMMAND
      test_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_r1_model_consistency
      "${PROJECT_SOURCE_DIR}/assets/robots/r1/mujoco/urdf/r1.urdf"
      "${PROJECT_SOURCE_DIR}/assets/robots/r1/mujoco/mjcf/r1.xml"
  )
  add_test(
    NAME apps.mcl_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_r1_model_structure
    COMMAND "${Python3_EXECUTABLE}"
      "${PROJECT_SOURCE_DIR}/assets/robots/r1/mujoco/verify_structure.py"
      "${PROJECT_SOURCE_DIR}/assets/robots/r1/mujoco/mjcf/r1.xml"
  )

  add_executable(test_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_solver tests/solver.cpp)
  target_link_libraries(
    test_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_solver
    PRIVATE motion_control_lab::planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_support
  )
  add_test(
    NAME apps.mcl_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_solver
    COMMAND
      test_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_solver
      "/workspace/products/synrobot/modules/common/robot_description/psi_r1/urdf/Psi_R1_rev1.urdf"
  )

  add_executable(
    test_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_control
    tests/nullspace_control.cpp
  )
  target_link_libraries(
    test_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_control
    PRIVATE motion_control_lab::planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_support
  )
  add_test(
    NAME apps.mcl_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_control
    COMMAND test_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_control
  )

  add_executable(test_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_joint_target tests/joint_target_processing.cpp)
  target_link_libraries(
    test_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_joint_target
    PRIVATE motion_control_lab::planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_support
  )
  add_test(
    NAME apps.mcl_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_joint_target
    COMMAND test_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_joint_target
  )

  add_executable(test_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_retarget_clamp tests/retarget_state_clamp.cpp)
  target_link_libraries(
    test_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_retarget_clamp
    PRIVATE motion_control_lab::planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_support
  )
  add_test(
    NAME apps.mcl_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_retarget_clamp
    COMMAND test_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_retarget_clamp
  )

  add_executable(test_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_visualization tests/visualization.cpp)
  target_link_libraries(
    test_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_visualization
    PRIVATE motion_control_lab::planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_support
  )
  add_test(
    NAME apps.mcl_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_visualization
    COMMAND test_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_visualization
  )

  add_executable(test_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_telemetry tests/telemetry.cpp)
  target_link_libraries(
    test_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_telemetry
    PRIVATE motion_control_lab::planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_support
  )
  add_test(
    NAME apps.mcl_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_telemetry
    COMMAND test_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_telemetry
  )

  add_executable(
    test_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_tui_projection
    tests/tui_projection.cpp
  )
  target_link_libraries(
    test_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_tui_projection
    PRIVATE motion_control_lab::planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_support
  )
  add_test(
    NAME apps.mcl_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_tui_projection
    COMMAND test_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_tui_projection
  )

  add_executable(
    test_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_rejection_policy
    tests/rejection_policy.cpp
  )
  target_link_libraries(
    test_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_rejection_policy
    PRIVATE motion_control_lab::planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_support
  )
  add_test(
    NAME apps.mcl_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_rejection_policy
    COMMAND test_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_rejection_policy
  )

  add_executable(
    test_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_replay_pipeline_gate
    tests/replay_pipeline_gate.cpp
  )
  target_link_libraries(
    test_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_replay_pipeline_gate
    PRIVATE motion_control_lab::planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_support
  )
  add_test(
    NAME apps.mcl_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_replay_pipeline_gate
    COMMAND test_planned_hierarchical_step_otg_nullspace_admittance_kinematic_sim_replay_pipeline_gate
  )

endif()
