mcl_add_app(
  TARGET mcl_servo_step
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
    motion_control_lab::run_artifacts
    libplaco
)
if(APPLE)
  set_property(
    TARGET mcl_servo_step
    PROPERTY INSTALL_RPATH "@loader_path/../${CMAKE_INSTALL_LIBDIR}"
  )
else()
  set_property(
    TARGET mcl_servo_step
    PROPERTY INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}"
  )
  target_link_options(
    mcl_servo_step
    PRIVATE "LINKER:--disable-new-dtags"
  )
endif()
if(BUILD_TESTING)
  add_executable(
    test_servo_step_app_options
    tests/app_options_contract.cpp
    app_options.cpp
  )
  target_link_libraries(
    test_servo_step_app_options
    PRIVATE
      motion_control_lab::input_contract
      motion_control_lab::replay
      motion_control_lab::tui
      motion_control_lab::r1_robot_config
  )
  add_test(
    NAME apps.mcl_servo_step_options
    COMMAND test_servo_step_app_options
  )

endif()
