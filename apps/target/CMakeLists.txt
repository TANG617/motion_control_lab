add_library(
  mcl_target_support
  solver.cpp
  loop.cpp
)
add_library(
  motion_control_lab::target_support
  ALIAS mcl_target_support
)
target_include_directories(
  mcl_target_support
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(
  mcl_target_support
  PUBLIC
    motion_control_lab::cpu_affinity
    motion_control_lab::app_helpers
    motion_control_lab::r1_robot_config
    motion_control_core::motion_control_core
    motion_control_lab::scheduler
    motion_control_lab::keyboard_input
    motion_control_lab::standard_ik_tui
    motion_control_lab::tui
    motion_control_lab::preview_transport
    motion_control_lab::preview_projection
    libplaco
)

mcl_add_app(
  TARGET mcl_target
  SOURCES main.cpp options.cpp
  LIBRARIES motion_control_lab::target_support
)
if(APPLE)
  set_property(
    TARGET mcl_target
    PROPERTY INSTALL_RPATH "@loader_path/../${CMAKE_INSTALL_LIBDIR}"
  )
else()
  set_property(
    TARGET mcl_target
    PROPERTY INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}"
  )
  target_link_options(
    mcl_target
    PRIVATE "LINKER:--disable-new-dtags"
  )
endif()
if(BUILD_TESTING)
  add_executable(
    test_target_app_options
    tests/app_options_contract.cpp
    options.cpp
  )
  target_link_libraries(
    test_target_app_options
    PRIVATE
      motion_control_lab::input_contract
      motion_control_lab::tui
      motion_control_lab::r1_robot_config
  )
  add_test(
    NAME apps.mcl_target_options
    COMMAND test_target_app_options
  )

endif()
