add_library(
  mcl_r1_robot_config
  r1_robot_config.cpp
)
add_library(motion_control_lab::r1_robot_config ALIAS mcl_r1_robot_config)
target_include_directories(
  mcl_r1_robot_config
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(mcl_r1_robot_config PUBLIC Eigen3::Eigen)

if(NOT MCL_BUILD_ANY_INTERACTIVE_IK)
  return()
endif()

add_library(
  mcl_app_common
  ik_app_utils.cpp
  r1_interactive_config.cpp
)
add_library(motion_control_lab::app_common ALIAS mcl_app_common)
target_include_directories(
  mcl_app_common
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(
  mcl_app_common
  PUBLIC
    motion_control_lab::r1_robot_config
    motion_control_core::motion_control_core
    motion_control_lab::interactive_support
)
