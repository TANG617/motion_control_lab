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

add_executable(mcl_planned_grouped_step_otg main.cpp app_options.cpp)
set_target_properties(
  mcl_planned_grouped_step_otg
  PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
)
target_link_libraries(
  mcl_planned_grouped_step_otg
  PRIVATE
    motion_control_lab::cpu_affinity
    motion_control_lab::app_common
    motion_control_lab::planned_grouped_step_otg_support
    motion_control_lab::interactive_runtime
    motion_control_lab::preview_transport
    motion_control_lab::ik_preview
    motion_control_lab::visualization_contracts
    motion_control_lab::replay_support
    Threads::Threads
)
install(TARGETS mcl_planned_grouped_step_otg RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

if(BUILD_TESTING)
  add_executable(
    test_planned_grouped_step_otg_app_options
    tests/app_options_contract.cpp
    app_options.cpp
  )
  target_link_libraries(
    test_planned_grouped_step_otg_app_options
    PRIVATE
      motion_control_lab::interactive_runtime
      motion_control_lab::replay_support
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
      motion_control_lab::ik_preview
      motion_control_lab::visualization_contracts
  )
  add_test(
    NAME apps.mcl_planned_grouped_step_otg_visualization
    COMMAND test_planned_grouped_step_otg_visualization
  )

  add_test(
    NAME apps.mcl_planned_grouped_step_otg_help
    COMMAND
      "${CMAKE_COMMAND}"
      "-DAPP=$<TARGET_FILE:mcl_planned_grouped_step_otg>"
      -P "${PROJECT_SOURCE_DIR}/tests/check_app_help.cmake"
  )
endif()
