add_executable(mcl_dual_arm_replay_ik main.cpp)
set_target_properties(
  mcl_dual_arm_replay_ik
  PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
)
target_link_libraries(
  mcl_dual_arm_replay_ik
  PRIVATE
    motion_control_lab::replay_support
    motion_control_core::motion_control_core
)
install(TARGETS mcl_dual_arm_replay_ik RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

if(BUILD_TESTING)
  add_test(NAME apps.mcl_dual_arm_replay_ik_help COMMAND mcl_dual_arm_replay_ik --help)
endif()
