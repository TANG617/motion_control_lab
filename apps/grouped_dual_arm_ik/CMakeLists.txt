add_executable(mcl_grouped_dual_arm_ik main.cpp)
set_target_properties(
  mcl_grouped_dual_arm_ik
  PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
)
target_link_libraries(
  mcl_grouped_dual_arm_ik
  PRIVATE
    motion_control_lab::cpu_affinity
    motion_control_lab::app_common
    motion_control_core::motion_control_core
    motion_control_lab::interactive_support
    Threads::Threads
)
install(TARGETS mcl_grouped_dual_arm_ik RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

if(BUILD_TESTING)
  add_test(
    NAME apps.mcl_grouped_dual_arm_ik_help
    COMMAND
      "${CMAKE_COMMAND}"
      "-DAPP=$<TARGET_FILE:mcl_grouped_dual_arm_ik>"
      -P "${PROJECT_SOURCE_DIR}/tests/check_app_help.cmake"
  )
endif()
