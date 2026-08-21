add_library(mcl_plot_core_planning_support planning.cpp)
add_library(
  motion_control_lab::plot_core_planning_support
  ALIAS mcl_plot_core_planning_support
)
target_include_directories(
  mcl_plot_core_planning_support
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}"
  SYSTEM PRIVATE "${PROJECT_SOURCE_DIR}/third_party/matplotlib-cpp"
)
target_compile_definitions(
  mcl_plot_core_planning_support
  PRIVATE MATPLOTLIBCPP_PYTHON_EXECUTABLE="${Python3_EXECUTABLE}"
)
target_link_libraries(
  mcl_plot_core_planning_support
  PUBLIC
    motion_control_lab::r1_robot_config
    motion_control_core::motion_control_core
  PRIVATE
    Python3::Python
    Python3::NumPy
)

add_executable(motion_control_lab_plot_core_planning main.cpp options.cpp)
target_link_libraries(
  motion_control_lab_plot_core_planning
  PRIVATE motion_control_lab::plot_core_planning_support
)

if(BUILD_TESTING)
  add_test(
    NAME apps.motion_control_lab_plot_core_planning_help
    COMMAND motion_control_lab_plot_core_planning --help
  )
endif()
