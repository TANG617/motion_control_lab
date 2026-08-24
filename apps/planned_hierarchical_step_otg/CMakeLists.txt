add_library(
  mcl_planned_hierarchical_step_otg_support
  solver.cpp
  planning.cpp
  loop.cpp
)
add_library(
  motion_control_lab::planned_hierarchical_step_otg_support
  ALIAS mcl_planned_hierarchical_step_otg_support
)
target_include_directories(
  mcl_planned_hierarchical_step_otg_support
  PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>"
)
target_link_libraries(
  mcl_planned_hierarchical_step_otg_support
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
    motion_control_lab::replay
    Threads::Threads
)

mcl_add_app(
  TARGET mcl_planned_hierarchical_step_otg
  SOURCES main.cpp options.cpp
  LIBRARIES motion_control_lab::planned_hierarchical_step_otg_support
)

if(BUILD_TESTING)
  add_executable(
    test_planned_hierarchical_step_otg_app_options
    tests/app_options_contract.cpp
    options.cpp
  )
  target_link_libraries(
    test_planned_hierarchical_step_otg_app_options
    PRIVATE
      motion_control_lab::input_contract
      motion_control_lab::scheduler
      motion_control_lab::replay
      motion_control_lab::tui
      motion_control_lab::r1_robot_config
      motion_control_core::motion_control_core
  )
  add_test(
    NAME apps.mcl_planned_hierarchical_step_otg_options
    COMMAND test_planned_hierarchical_step_otg_app_options
  )

  add_executable(test_planned_hierarchical_step_otg_solver tests/solver.cpp)
  target_link_libraries(
    test_planned_hierarchical_step_otg_solver
    PRIVATE motion_control_lab::planned_hierarchical_step_otg_support
  )
  add_test(
    NAME apps.mcl_planned_hierarchical_step_otg_solver
    COMMAND test_planned_hierarchical_step_otg_solver
  )

  add_executable(test_planned_hierarchical_step_otg_joint_target tests/joint_target_processing.cpp)
  target_link_libraries(
    test_planned_hierarchical_step_otg_joint_target
    PRIVATE motion_control_lab::planned_hierarchical_step_otg_support
  )
  add_test(
    NAME apps.mcl_planned_hierarchical_step_otg_joint_target
    COMMAND test_planned_hierarchical_step_otg_joint_target
  )

  add_executable(test_planned_hierarchical_step_otg_retarget_clamp tests/retarget_state_clamp.cpp)
  target_link_libraries(
    test_planned_hierarchical_step_otg_retarget_clamp
    PRIVATE motion_control_lab::planned_hierarchical_step_otg_support
  )
  add_test(
    NAME apps.mcl_planned_hierarchical_step_otg_retarget_clamp
    COMMAND test_planned_hierarchical_step_otg_retarget_clamp
  )

  add_executable(test_planned_hierarchical_step_otg_visualization tests/visualization.cpp)
  target_link_libraries(
    test_planned_hierarchical_step_otg_visualization
    PRIVATE motion_control_lab::planned_hierarchical_step_otg_support
  )
  add_test(
    NAME apps.mcl_planned_hierarchical_step_otg_visualization
    COMMAND test_planned_hierarchical_step_otg_visualization
  )

  add_executable(
    test_planned_hierarchical_step_otg_tui_projection
    tests/tui_projection.cpp
  )
  target_link_libraries(
    test_planned_hierarchical_step_otg_tui_projection
    PRIVATE motion_control_lab::planned_grouped_tui
  )
  add_test(
    NAME apps.mcl_planned_hierarchical_step_otg_tui_projection
    COMMAND test_planned_hierarchical_step_otg_tui_projection
  )
endif()
