add_executable(mcl_dual_arm_ik_target_solve main.cpp)
set_target_properties(
  mcl_dual_arm_ik_target_solve
  PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
)
target_link_libraries(
  mcl_dual_arm_ik_target_solve
  PRIVATE
    motion_control_lab::app_common
    motion_control_core::motion_control_core
    motion_control_lab::interactive_support
)
install(
  TARGETS mcl_dual_arm_ik_target_solve
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(BUILD_TESTING)
  add_test(
    NAME apps.mcl_dual_arm_ik_target_solve_help
    COMMAND
      "${CMAKE_COMMAND}"
      "-DAPP=$<TARGET_FILE:mcl_dual_arm_ik_target_solve>"
      -P "${PROJECT_SOURCE_DIR}/tests/check_app_help.cmake"
  )
endif()
