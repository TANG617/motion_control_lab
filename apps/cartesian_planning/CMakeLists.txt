add_library(
  mcl_cartesian_planning_support
  planning.cpp
  loop.cpp
)
add_library(
  motion_control_lab::cartesian_planning_support
  ALIAS mcl_cartesian_planning_support
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
    motion_control_lab::visualization_contracts
  PRIVATE
    eiquadprog::eiquadprog
    jsoncpp_lib
    Python3::Python
    Python3::NumPy
)

mcl_add_app(
  TARGET mcl_cartesian_planning
  SOURCES main.cpp options.cpp
  LIBRARIES motion_control_lab::cartesian_planning_support
)

if(BUILD_TESTING)
  add_executable(
    test_cartesian_planning
    "${PROJECT_SOURCE_DIR}/tests/cartesian_planning.cpp"
    options.cpp
  )
  target_link_libraries(
    test_cartesian_planning
    PRIVATE
      motion_control_lab::cartesian_planning_support
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
    NAME apps.mcl_cartesian_planning_smoke
    COMMAND
      mcl_cartesian_planning
      --request "${CMAKE_CURRENT_SOURCE_DIR}/example_request.json"
      --output-dir "${PROJECT_BINARY_DIR}/test-cartesian-planning-cli"
      --no-live
      --once
      --force
  )
  if(EXISTS "/usr/lib/python3/dist-packages/matplotlib")
    set_tests_properties(
      apps.mcl_cartesian_planning_smoke
      PROPERTIES ENVIRONMENT "PYTHONPATH=/usr/lib/python3/dist-packages"
    )
  endif()
endif()
