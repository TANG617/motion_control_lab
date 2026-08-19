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

add_executable(mcl_planned_grouped_servo_step main.cpp app_options.cpp)
set_target_properties(
  mcl_planned_grouped_servo_step
  PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
)
target_link_libraries(
  mcl_planned_grouped_servo_step
  PRIVATE
    motion_control_lab::cpu_affinity
    motion_control_lab::app_common
    motion_control_lab::planned_grouped_servo_step_retarget_clamp
    motion_control_lab::interactive_support
    motion_control_lab::replay_support
    Threads::Threads
)
install(TARGETS mcl_planned_grouped_servo_step RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

if(BUILD_TESTING)
  add_executable(
    test_planned_grouped_servo_step_app_options
    tests/app_options_contract.cpp
    app_options.cpp
  )
  target_link_libraries(
    test_planned_grouped_servo_step_app_options
    PRIVATE
      motion_control_lab::interactive_support
      motion_control_lab::replay_support
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
    PRIVATE motion_control_lab::interactive_support
  )
  add_test(
    NAME apps.mcl_planned_grouped_servo_step_visualization
    COMMAND test_planned_grouped_servo_step_visualization
  )

  add_test(
    NAME apps.mcl_planned_grouped_servo_step_help
    COMMAND
      "${CMAKE_COMMAND}"
      "-DAPP=$<TARGET_FILE:mcl_planned_grouped_servo_step>"
      -P "${PROJECT_SOURCE_DIR}/tests/check_app_help.cmake"
  )
endif()
