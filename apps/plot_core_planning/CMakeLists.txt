add_executable(motion_control_lab_plot_core_planning main.cpp)
target_include_directories(
  motion_control_lab_plot_core_planning
  SYSTEM PRIVATE "${PROJECT_SOURCE_DIR}/third_party/matplotlib-cpp"
)
target_compile_definitions(
  motion_control_lab_plot_core_planning
  PRIVATE MATPLOTLIBCPP_PYTHON_EXECUTABLE="${Python3_EXECUTABLE}"
)
target_link_libraries(
  motion_control_lab_plot_core_planning
  PRIVATE
    motion_control_lab::r1_robot_config
    motion_control_core::motion_control_core
    Python3::Python
    Python3::NumPy
)
