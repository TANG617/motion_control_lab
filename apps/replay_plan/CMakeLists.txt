add_library(mcl_replay_plan_support loop.cpp)
add_library(
  motion_control_lab::replay_plan_support
  ALIAS mcl_replay_plan_support
)
target_include_directories(
  mcl_replay_plan_support
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(
  mcl_replay_plan_support
  PUBLIC motion_control_lab::replay
)

mcl_add_app(
  TARGET mcl_replay_plan
  SOURCES main.cpp options.cpp
  LIBRARIES motion_control_lab::replay_plan_support
)
