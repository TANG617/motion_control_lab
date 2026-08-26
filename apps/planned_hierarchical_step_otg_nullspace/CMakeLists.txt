add_library(
  mcl_planned_hierarchical_step_otg_nullspace_support
  solver.cpp
  nullspace.cpp
  planning.cpp
  telemetry.cpp
  loop.cpp
)
add_library(
  motion_control_lab::planned_hierarchical_step_otg_nullspace_support
  ALIAS mcl_planned_hierarchical_step_otg_nullspace_support
)
target_include_directories(
  mcl_planned_hierarchical_step_otg_nullspace_support
  PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>"
)
target_link_libraries(
  mcl_planned_hierarchical_step_otg_nullspace_support
  PUBLIC
    motion_control_core::motion_control_core
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
  TARGET mcl_planned_hierarchical_step_otg_nullspace
  SOURCES main.cpp options.cpp
  LIBRARIES motion_control_lab::planned_hierarchical_step_otg_nullspace_support
)

if(BUILD_TESTING)
  add_executable(
    test_planned_hierarchical_step_otg_nullspace_app_options
    tests/app_options_contract.cpp
    options.cpp
  )
  target_link_libraries(
    test_planned_hierarchical_step_otg_nullspace_app_options
    PRIVATE
      motion_control_lab::input_contract
      motion_control_lab::scheduler
      motion_control_lab::replay
      motion_control_lab::tui
      motion_control_lab::r1_robot_config
      motion_control_core::motion_control_core
  )
  add_test(
    NAME apps.mcl_planned_hierarchical_step_otg_nullspace_options
    COMMAND test_planned_hierarchical_step_otg_nullspace_app_options
  )

  add_executable(test_planned_hierarchical_step_otg_nullspace_solver tests/solver.cpp)
  target_link_libraries(
    test_planned_hierarchical_step_otg_nullspace_solver
    PRIVATE motion_control_lab::planned_hierarchical_step_otg_nullspace_support
  )
  add_test(
    NAME apps.mcl_planned_hierarchical_step_otg_nullspace_solver
    COMMAND
      test_planned_hierarchical_step_otg_nullspace_solver
      "/workspace/products/synrobot/modules/common/robot_description/psi_r1/urdf/Psi_R1_rev1.urdf"
  )

  add_executable(
    test_planned_hierarchical_step_otg_nullspace_control
    tests/nullspace_control.cpp
  )
  target_link_libraries(
    test_planned_hierarchical_step_otg_nullspace_control
    PRIVATE motion_control_lab::planned_hierarchical_step_otg_nullspace_support
  )
  add_test(
    NAME apps.mcl_planned_hierarchical_step_otg_nullspace_control
    COMMAND test_planned_hierarchical_step_otg_nullspace_control
  )

  add_executable(test_planned_hierarchical_step_otg_nullspace_joint_target tests/joint_target_processing.cpp)
  target_link_libraries(
    test_planned_hierarchical_step_otg_nullspace_joint_target
    PRIVATE motion_control_lab::planned_hierarchical_step_otg_nullspace_support
  )
  add_test(
    NAME apps.mcl_planned_hierarchical_step_otg_nullspace_joint_target
    COMMAND test_planned_hierarchical_step_otg_nullspace_joint_target
  )

  add_executable(test_planned_hierarchical_step_otg_nullspace_retarget_clamp tests/retarget_state_clamp.cpp)
  target_link_libraries(
    test_planned_hierarchical_step_otg_nullspace_retarget_clamp
    PRIVATE motion_control_lab::planned_hierarchical_step_otg_nullspace_support
  )
  add_test(
    NAME apps.mcl_planned_hierarchical_step_otg_nullspace_retarget_clamp
    COMMAND test_planned_hierarchical_step_otg_nullspace_retarget_clamp
  )

  add_executable(test_planned_hierarchical_step_otg_nullspace_visualization tests/visualization.cpp)
  target_link_libraries(
    test_planned_hierarchical_step_otg_nullspace_visualization
    PRIVATE motion_control_lab::planned_hierarchical_step_otg_nullspace_support
  )
  add_test(
    NAME apps.mcl_planned_hierarchical_step_otg_nullspace_visualization
    COMMAND test_planned_hierarchical_step_otg_nullspace_visualization
  )

  add_executable(test_planned_hierarchical_step_otg_nullspace_telemetry tests/telemetry.cpp)
  target_link_libraries(
    test_planned_hierarchical_step_otg_nullspace_telemetry
    PRIVATE motion_control_lab::planned_hierarchical_step_otg_nullspace_support
  )
  add_test(
    NAME apps.mcl_planned_hierarchical_step_otg_nullspace_telemetry
    COMMAND test_planned_hierarchical_step_otg_nullspace_telemetry
  )

  add_executable(
    test_planned_hierarchical_step_otg_nullspace_tui_projection
    tests/tui_projection.cpp
  )
  target_link_libraries(
    test_planned_hierarchical_step_otg_nullspace_tui_projection
    PRIVATE motion_control_lab::planned_hierarchical_step_otg_nullspace_support
  )
  add_test(
    NAME apps.mcl_planned_hierarchical_step_otg_nullspace_tui_projection
    COMMAND test_planned_hierarchical_step_otg_nullspace_tui_projection
  )

endif()
