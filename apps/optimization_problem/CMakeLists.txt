add_library(mcl_optimization_problem_support solver.cpp options.cpp)
target_include_directories(mcl_optimization_problem_support PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${PROJECT_SOURCE_DIR}")
target_link_libraries(mcl_optimization_problem_support PUBLIC motion_control_core::motion_control_core motion_control_lab::run_artifacts libplaco)
mcl_add_app(TARGET mcl_optimization_problem SOURCES main.cpp LIBRARIES mcl_optimization_problem_support)

if(BUILD_TESTING)
  add_test(NAME apps.mcl_optimization_problem_public_execution
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/public_execution.py"
      "$<TARGET_FILE:mcl_optimization_problem>")
  set_tests_properties(apps.mcl_optimization_problem_public_execution PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE)
endif()
