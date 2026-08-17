add_executable(mcl_dual_arm_ik_servo_step main.cpp)
set_target_properties(
  mcl_dual_arm_ik_servo_step
  PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
)
if(APPLE)
  set_property(
    TARGET mcl_dual_arm_ik_servo_step
    PROPERTY INSTALL_RPATH "@loader_path/../${CMAKE_INSTALL_LIBDIR}"
  )
else()
  set_property(
    TARGET mcl_dual_arm_ik_servo_step
    PROPERTY INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}"
  )
  target_link_options(
    mcl_dual_arm_ik_servo_step
    PRIVATE "LINKER:--disable-new-dtags"
  )
endif()
target_link_libraries(
  mcl_dual_arm_ik_servo_step
  PRIVATE
    motion_control_lab::cpu_affinity
    motion_control_lab::app_common
    motion_control_core::motion_control_core
    motion_control_lab::interactive_support
    libplaco
)
install(
  TARGETS mcl_dual_arm_ik_servo_step
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(BUILD_TESTING)
  add_test(
    NAME apps.mcl_dual_arm_ik_servo_step_help
    COMMAND
      "${CMAKE_COMMAND}"
      "-DAPP=$<TARGET_FILE:mcl_dual_arm_ik_servo_step>"
      -P "${PROJECT_SOURCE_DIR}/tests/check_app_help.cmake"
  )
endif()
