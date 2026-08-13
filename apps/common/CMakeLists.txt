add_library(
  mcl_app_common
  dual_arm_ik_app.cpp
  ik_app_utils.cpp
  r1_robot_config.cpp
)
add_library(motion_control_lab::app_common ALIAS mcl_app_common)
target_include_directories(
  mcl_app_common
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(
  mcl_app_common
  PUBLIC
    motion_control_core::motion_control_core
    motion_control_lab::interactive_support
)
