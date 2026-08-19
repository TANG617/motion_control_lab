add_library(
  mcl_cartesian_planning_support
  cartesian_planning.cpp
)
target_include_directories(
  mcl_cartesian_planning_support
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}"
  SYSTEM PRIVATE "${PROJECT_SOURCE_DIR}/third_party/matplotlib-cpp"
)
target_compile_definitions(
  mcl_cartesian_planning_support
  PRIVATE MATPLOTLIBCPP_PYTHON_EXECUTABLE="${Python3_EXECUTABLE}"
)
target_link_libraries(
  mcl_cartesian_planning_support
  PUBLIC
    motion_control_core::motion_control_core
    motion_control_lab::preview_transport
  PRIVATE
    eiquadprog::eiquadprog
    jsoncpp_lib
    Python3::Python
    Python3::NumPy
)

add_executable(mcl_cartesian_planning main.cpp)
set_target_properties(
  mcl_cartesian_planning
  PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
)
target_link_libraries(
  mcl_cartesian_planning
  PRIVATE mcl_cartesian_planning_support
)
install(TARGETS mcl_cartesian_planning RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

if(BUILD_TESTING)
  add_executable(
    test_cartesian_planning
    "${PROJECT_SOURCE_DIR}/tests/cartesian_planning.cpp"
  )
  target_link_libraries(
    test_cartesian_planning
    PRIVATE
      mcl_cartesian_planning_support
      jsoncpp_lib
  )
  add_test(
    NAME apps.cartesian_planning
    COMMAND
      test_cartesian_planning
      "${CMAKE_CURRENT_SOURCE_DIR}/example_request.json"
      "${PROJECT_BINARY_DIR}/test-cartesian-planning"
  )
  add_test(
    NAME apps.mcl_cartesian_planning_help
    COMMAND
      "${CMAKE_COMMAND}"
      "-DAPP=$<TARGET_FILE:mcl_cartesian_planning>"
      -P "${PROJECT_SOURCE_DIR}/tests/check_app_help.cmake"
  )
  add_test(
    NAME apps.mcl_cartesian_planning_smoke
    COMMAND
      mcl_cartesian_planning
      --request "${CMAKE_CURRENT_SOURCE_DIR}/example_request.json"
      --output-dir "${PROJECT_BINARY_DIR}/test-cartesian-planning-cli"
      --no-live
      --once
      --force
  )
endif()
