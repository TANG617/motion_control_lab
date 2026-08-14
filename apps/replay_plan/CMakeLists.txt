add_executable(mcl_replay_plan main.cpp replay_plan.cpp)
set_target_properties(
  mcl_replay_plan
  PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
)
target_include_directories(mcl_replay_plan PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(
  mcl_replay_plan
  PRIVATE
    motion_control_lab::data
    motion_control_lab::run_artifacts
    jsoncpp_lib
)
install(TARGETS mcl_replay_plan RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

if(BUILD_TESTING)
  add_test(NAME apps.mcl_replay_plan_help COMMAND mcl_replay_plan --help)
endif()
