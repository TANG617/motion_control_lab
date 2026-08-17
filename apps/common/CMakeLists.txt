add_library(
  mcl_cpu_affinity
  cpu_affinity.cpp
)
add_library(motion_control_lab::cpu_affinity ALIAS mcl_cpu_affinity)
target_include_directories(
  mcl_cpu_affinity
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}"
)
target_compile_definitions(
  mcl_cpu_affinity
  PUBLIC MCL_ENABLE_CPU_AFFINITY=$<BOOL:${MCL_ENABLE_CPU_AFFINITY}>
)
target_link_libraries(mcl_cpu_affinity PRIVATE Threads::Threads)

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
