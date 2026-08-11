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
target_include_directories(
  mcl_dual_arm_replay_ik
  PRIVATE "${PROJECT_BINARY_DIR}/generated"
)

if(MCL_BUILD_DUAL_ARM_REPLAY_VISUALIZATION)
  if(NOT TARGET motion_control_viz::foxglove_sink)
    message(
      FATAL_ERROR
      "MCL_BUILD_DUAL_ARM_REPLAY_VISUALIZATION requires motion_control_viz Foxglove support"
    )
  endif()
  target_link_libraries(
    mcl_dual_arm_replay_ik
    PRIVATE
      motion_control_viz::motion_control_viz
      motion_control_viz::foxglove_sink
  )
  target_compile_definitions(
    mcl_dual_arm_replay_ik
    PRIVATE MCL_WITH_REPLAY_VISUALIZATION=1
  )
else()
  target_compile_definitions(
    mcl_dual_arm_replay_ik
    PRIVATE MCL_WITH_REPLAY_VISUALIZATION=0
  )
endif()
install(TARGETS mcl_dual_arm_replay_ik RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

if(BUILD_TESTING)
  add_test(NAME apps.mcl_dual_arm_replay_ik_help COMMAND mcl_dual_arm_replay_ik --help)
endif()
