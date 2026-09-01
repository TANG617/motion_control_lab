add_library(
  mcl_hierarchical_kinematics_step_support
  options.cpp
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
  DESTINATION
    "${CMAKE_INSTALL_LIBEXECDIR}/motion-control-lab/mcl_hierarchical_kinematics_step"
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING
    PATTERN "*.py"
    PATTERN "__pycache__" EXCLUDE
)

if(BUILD_TESTING)
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
  add_test(
    NAME apps.mcl_hierarchical_kinematics_step_python_launcher
    COMMAND
      "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/tests/python_launcher_contract.py"
      "$<TARGET_FILE:mcl_hierarchical_kinematics_step>"
      "${PROJECT_SOURCE_DIR}/assets/robots/r1/mujoco/urdf/r1.urdf"
      "${PROJECT_SOURCE_DIR}/assets/robots/r1/mujoco/mjcf/r1.xml"
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
