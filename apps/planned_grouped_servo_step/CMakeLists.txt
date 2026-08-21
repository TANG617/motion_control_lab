add_library(
  mcl_planned_grouped_servo_step_retarget_clamp
  retarget_state_clamp.cpp
)
add_library(
  motion_control_lab::planned_grouped_servo_step_retarget_clamp
  ALIAS mcl_planned_grouped_servo_step_retarget_clamp
)
target_include_directories(
  mcl_planned_grouped_servo_step_retarget_clamp
  PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>"
)
target_link_libraries(
  mcl_planned_grouped_servo_step_retarget_clamp
  PUBLIC motion_control_core::motion_control_core
)

mcl_add_app(
  TARGET mcl_planned_grouped_servo_step
  SOURCES
    main.cpp app_options.cpp diagnostics_projection.cpp
  LIBRARIES
    motion_control_lab::cpu_affinity
    motion_control_lab::app_helpers
    motion_control_lab::r1_robot_config
    motion_control_lab::planned_grouped_servo_step_retarget_clamp
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
    test_planned_grouped_servo_step_app_options
    tests/app_options_contract.cpp
    app_options.cpp
  )
  target_link_libraries(
    test_planned_grouped_servo_step_app_options
    PRIVATE
      motion_control_lab::input_contract
      motion_control_lab::scheduler
      motion_control_lab::replay
      motion_control_lab::tui
      motion_control_lab::r1_robot_config
  )
  add_test(
    NAME apps.mcl_planned_grouped_servo_step_options
    COMMAND test_planned_grouped_servo_step_app_options
  )

  add_executable(
    test_planned_grouped_servo_step_retarget_clamp
    "${PROJECT_SOURCE_DIR}/tests/planned_grouped_servo_step_retarget_clamp.cpp"
  )
  target_link_libraries(
    test_planned_grouped_servo_step_retarget_clamp
    PRIVATE motion_control_lab::planned_grouped_servo_step_retarget_clamp
  )
  add_test(
    NAME apps.mcl_planned_grouped_servo_step_retarget_clamp
    COMMAND test_planned_grouped_servo_step_retarget_clamp
  )

  add_executable(
    test_planned_grouped_servo_step_visualization
    "${PROJECT_SOURCE_DIR}/tests/planned_grouped_servo_step_visualization.cpp"
  )
  target_link_libraries(
    test_planned_grouped_servo_step_visualization
    PRIVATE
      motion_control_lab::preview_projection
      motion_control_lab::visualization_contracts
  )
  add_test(
    NAME apps.mcl_planned_grouped_servo_step_visualization
    COMMAND test_planned_grouped_servo_step_visualization
  )

endif()
