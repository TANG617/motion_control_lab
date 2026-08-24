add_library(
  mcl_single_arm_step_support
  solver.cpp
  loop.cpp
)
add_library(
  motion_control_lab::single_arm_step_support
  ALIAS mcl_single_arm_step_support
)
target_include_directories(
  mcl_single_arm_step_support
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(
  mcl_single_arm_step_support
  PUBLIC
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
)

mcl_add_app(
  TARGET mcl_single_arm_step
  SOURCES main.cpp options.cpp
  LIBRARIES motion_control_lab::single_arm_step_support
)

if(BUILD_TESTING)
  add_executable(
    test_single_arm_step_app_options
    tests/app_options_contract.cpp
    options.cpp
  )
  target_link_libraries(
    test_single_arm_step_app_options
    PRIVATE
      motion_control_lab::input_contract
      motion_control_lab::presentation_contract
      motion_control_lab::tui
      motion_control_lab::r1_robot_config
  )
  add_test(
    NAME apps.mcl_single_arm_step_options
    COMMAND test_single_arm_step_app_options
  )

endif()
