add_library(
  mcl_e03_batch_support
  src/batch_replay.cpp
)
add_library(motion_control_lab::e03_batch_support ALIAS mcl_e03_batch_support)
target_include_directories(
  mcl_e03_batch_support
  PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>"
)
target_link_libraries(
  mcl_e03_batch_support
  PUBLIC motion_control_lab::run_artifacts
)

add_executable(
  mcl_e03_batch_replay_ik
  src/main.cpp
)
set_target_properties(
  mcl_e03_batch_replay_ik
  PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
)
target_include_directories(
  mcl_e03_batch_replay_ik
  PRIVATE "${PROJECT_BINARY_DIR}/generated"
)
target_link_libraries(
  mcl_e03_batch_replay_ik
  PRIVATE
    motion_control_lab::e03_batch_support
    motion_control_lab::e03_legacy_replay_ik_engine
    motion_control_lab::visualization_contracts
)

if(MCL_BUILD_E03_REPLAY_VISUALIZATION)
  if(NOT TARGET motion_control_viz::foxglove)
    message(
      FATAL_ERROR
      "E03 replay visualization requires motion_control_viz Foxglove support"
    )
  endif()
  target_link_libraries(
    mcl_e03_batch_replay_ik
    PRIVATE motion_control_viz::foxglove
  )
  target_compile_definitions(
    mcl_e03_batch_replay_ik
    PRIVATE MCL_WITH_REPLAY_VISUALIZATION=1
  )
else()
  target_compile_definitions(
    mcl_e03_batch_replay_ik
    PRIVATE MCL_WITH_REPLAY_VISUALIZATION=0
  )
endif()

install(TARGETS mcl_e03_batch_replay_ik RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  add_executable(
    test_e03_batch_replay
    tests/batch_replay.cpp
  )
  target_link_libraries(
    test_e03_batch_replay
    PRIVATE motion_control_lab::e03_batch_support
  )
  add_test(NAME experiments.e03_batch_replay_unit COMMAND test_e03_batch_replay)

  add_executable(
    test_e03_replay_ik_integration
    tests/replay_ik_integration.cpp
  )
  target_link_libraries(
    test_e03_replay_ik_integration
    PRIVATE motion_control_lab::e03_legacy_replay_ik_engine
  )
  add_test(
    NAME experiments.e03_replay_ik_integration
    COMMAND
      test_e03_replay_ik_integration
      "${MCL_WORKSPACE_DIRECTORY}/products/synrobot/modules/common/robot_description/psi_r1/urdf/Psi_R1_rev1.urdf"
      "$<TARGET_FILE:mcl_e03_batch_replay_ik>"
  )
  add_test(
    NAME experiments.e03_empty_library_manifest
    COMMAND
      "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/tests/validate_empty_run.py"
      --executable "$<TARGET_FILE:mcl_e03_batch_replay_ik>"
      --urdf "${MCL_WORKSPACE_DIRECTORY}/products/synrobot/modules/common/robot_description/psi_r1/urdf/Psi_R1_rev1.urdf"
      --work-root "${PROJECT_BINARY_DIR}/test-runs"
  )
  add_test(NAME apps.mcl_e03_batch_replay_ik_help COMMAND mcl_e03_batch_replay_ik --help)
endif()
