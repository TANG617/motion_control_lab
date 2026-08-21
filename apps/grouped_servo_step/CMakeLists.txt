mcl_add_app(
  TARGET mcl_grouped_servo_step
  SOURCES main.cpp app_options.cpp diagnostics_projection.cpp
  LIBRARIES
    motion_control_lab::cpu_affinity
    motion_control_lab::app_helpers
    motion_control_lab::r1_robot_config
    motion_control_core::motion_control_core
    motion_control_lab::scheduler
    motion_control_lab::keyboard_input
    motion_control_lab::standard_ik_tui
    motion_control_lab::tui
    motion_control_lab::preview_transport
    motion_control_lab::preview_projection
    motion_control_lab::replay
    Threads::Threads
)

if(BUILD_TESTING)
  add_executable(
    test_grouped_servo_step_app_options
    tests/app_options_contract.cpp
    app_options.cpp
  )
  target_link_libraries(
    test_grouped_servo_step_app_options
    PRIVATE
      motion_control_lab::input_contract
      motion_control_lab::scheduler
      motion_control_lab::replay
      motion_control_lab::tui
      motion_control_lab::r1_robot_config
  )
  add_test(
    NAME apps.mcl_grouped_servo_step_options
    COMMAND test_grouped_servo_step_app_options
  )

endif()
