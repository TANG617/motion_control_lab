add_library(
  mcl_planned_grouped_step_otg_support
  joint_target_processing.cpp
  retarget_state_clamp.cpp
)
add_library(
  motion_control_lab::planned_grouped_step_otg_support
  ALIAS mcl_planned_grouped_step_otg_support
)
target_include_directories(
  mcl_planned_grouped_step_otg_support
  PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>"
)
target_link_libraries(
  mcl_planned_grouped_step_otg_support
  PUBLIC motion_control_core::motion_control_core
)

mcl_add_app(
  TARGET mcl_planned_grouped_step_otg
  SOURCES
    main.cpp app_options.cpp diagnostics_projection.cpp tui_projection.cpp
  LIBRARIES
    motion_control_lab::cpu_affinity
    motion_control_lab::app_helpers
    motion_control_lab::r1_robot_config
    motion_control_lab::planned_grouped_step_otg_support
    motion_control_lab::scheduler
    motion_control_lab::keyboard_input
    motion_control_lab::standard_ik_tui
    motion_control_lab::tui
    motion_control_lab::preview_transport
    motion_control_lab::preview_projection
    motion_control_lab::visualization_contracts
    motion_control_lab::replay
    Threads::Threads
)

if(BUILD_TESTING)
  add_executable(
    test_planned_grouped_step_otg_app_options
    tests/app_options_contract.cpp
    app_options.cpp
  )
  target_link_libraries(
    test_planned_grouped_step_otg_app_options
    PRIVATE
      motion_control_lab::input_contract
      motion_control_lab::scheduler
      motion_control_lab::replay
      motion_control_lab::tui
      motion_control_lab::r1_robot_config
  )
  add_test(
    NAME apps.mcl_planned_grouped_step_otg_options
    COMMAND test_planned_grouped_step_otg_app_options
  )

  add_executable(
    test_planned_grouped_step_otg_joint_target
    tests/joint_target_processing.cpp
  )
  target_link_libraries(
    test_planned_grouped_step_otg_joint_target
    PRIVATE motion_control_lab::planned_grouped_step_otg_support
  )
  add_test(
    NAME apps.mcl_planned_grouped_step_otg_joint_target
    COMMAND test_planned_grouped_step_otg_joint_target
  )

  add_executable(
    test_planned_grouped_step_otg_retarget_clamp
    tests/retarget_state_clamp.cpp
  )
  target_link_libraries(
    test_planned_grouped_step_otg_retarget_clamp
    PRIVATE motion_control_lab::planned_grouped_step_otg_support
  )
  add_test(
    NAME apps.mcl_planned_grouped_step_otg_retarget_clamp
    COMMAND test_planned_grouped_step_otg_retarget_clamp
  )

  add_executable(
    test_planned_grouped_step_otg_visualization
    tests/visualization.cpp
  )
  target_link_libraries(
    test_planned_grouped_step_otg_visualization
    PRIVATE
      motion_control_lab::preview_projection
      motion_control_lab::visualization_contracts
  )
  add_test(
    NAME apps.mcl_planned_grouped_step_otg_visualization
    COMMAND test_planned_grouped_step_otg_visualization
  )

  add_executable(
    test_planned_grouped_step_otg_tui_projection
    tests/tui_projection.cpp
    tui_projection.cpp
  )
  target_link_libraries(
    test_planned_grouped_step_otg_tui_projection
    PRIVATE motion_control_lab::standard_ik_tui
  )
  add_test(
    NAME apps.mcl_planned_grouped_step_otg_tui_projection
    COMMAND test_planned_grouped_step_otg_tui_projection
  )

endif()
