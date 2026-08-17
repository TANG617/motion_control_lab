add_executable(mcl_target_solve main.cpp)
set_target_properties(
  mcl_target_solve
  PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
)
if(APPLE)
  set_property(
    TARGET mcl_target_solve
    PROPERTY INSTALL_RPATH "@loader_path/../${CMAKE_INSTALL_LIBDIR}"
  )
else()
  set_property(
    TARGET mcl_target_solve
    PROPERTY INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}"
  )
  target_link_options(
    mcl_target_solve
    PRIVATE "LINKER:--disable-new-dtags"
  )
endif()
target_link_libraries(
  mcl_target_solve
  PRIVATE
    motion_control_lab::cpu_affinity
    motion_control_lab::app_common
    motion_control_core::motion_control_core
    motion_control_lab::interactive_support
    libplaco
)
install(
  TARGETS mcl_target_solve
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(BUILD_TESTING)
  add_test(
    NAME apps.mcl_target_solve_help
    COMMAND
      "${CMAKE_COMMAND}"
      "-DAPP=$<TARGET_FILE:mcl_target_solve>"
      -P "${PROJECT_SOURCE_DIR}/tests/check_app_help.cmake"
  )
endif()
