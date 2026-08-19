add_executable(mcl_grouped_servo_step main.cpp app_options.cpp)
set_target_properties(
  mcl_grouped_servo_step
  PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
)
target_link_libraries(
  mcl_grouped_servo_step
  PRIVATE
    motion_control_lab::cpu_affinity
    motion_control_lab::app_common
    motion_control_core::motion_control_core
    motion_control_lab::interactive_runtime
    motion_control_lab::preview_transport
    motion_control_lab::ik_preview
    motion_control_lab::replay_support
    Threads::Threads
)
install(TARGETS mcl_grouped_servo_step RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

if(BUILD_TESTING)
  add_executable(
    test_grouped_servo_step_app_options
    tests/app_options_contract.cpp
    app_options.cpp
  )
  target_link_libraries(
    test_grouped_servo_step_app_options
    PRIVATE
      motion_control_lab::interactive_runtime
      motion_control_lab::replay_support
      motion_control_lab::r1_robot_config
  )
  add_test(
    NAME apps.mcl_grouped_servo_step_options
    COMMAND test_grouped_servo_step_app_options
  )

  add_test(
    NAME apps.mcl_grouped_servo_step_help
    COMMAND
      "${CMAKE_COMMAND}"
      "-DAPP=$<TARGET_FILE:mcl_grouped_servo_step>"
      -P "${PROJECT_SOURCE_DIR}/tests/check_app_help.cmake"
  )
endif()
