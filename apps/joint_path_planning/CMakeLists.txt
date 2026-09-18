add_library(mcl_joint_path_planning_support options.cpp solver.cpp planning.cpp loop.cpp tui_projection.cpp visualization.cpp)
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/foxglove/SceneUpdate.json" MCL_SCENE_SCHEMA)
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/foxglove/status.schema.json" MCL_STATUS_SCHEMA)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS foxglove/SceneUpdate.json foxglove/status.schema.json)
configure_file(scene_schema.hpp.in scene_schema.hpp @ONLY)
target_include_directories(mcl_joint_path_planning_support PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")
target_include_directories(mcl_joint_path_planning_support PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(mcl_joint_path_planning_support PRIVATE MCL_JOINT_PATH_SHARE="${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATADIR}/motion-control-lab")
target_link_libraries(mcl_joint_path_planning_support PUBLIC
  motion_control_core::joint_planning motion_control_viz::render motion_control_viz::foxglove
  motion_control_lab::visualization_contracts
  motion_control_lab::r1_robot_config motion_control_lab::keyboard_teleop motion_control_lab::cartesian_teleop
  motion_control_lab::terminal_frontend motion_control_lab::tui motion_control_lab::run_artifacts Threads::Threads jsoncpp_lib)
mcl_add_app(TARGET mcl_joint_path_planning SOURCES main.cpp LIBRARIES mcl_joint_path_planning_support)
# Keep the explicitly selected Core component and base library from the same install.
# The workspace may also contain a different Core with the same SONAME.
set_property(TARGET mcl_joint_path_planning PROPERTY INSTALL_RPATH
  "$<TARGET_FILE_DIR:motion_control_core::motion_control_core>;${CMAKE_INSTALL_RPATH}")
install(DIRECTORY configs foxglove DESTINATION ${CMAKE_INSTALL_DATADIR}/motion-control-lab/apps/joint_path_planning)
install(PROGRAMS scripts/run_keyboard.py DESTINATION ${CMAKE_INSTALL_LIBEXECDIR}/motion-control-lab/mcl_joint_path_planning)

if(BUILD_TESTING)
  set(MCL_JOINT_PATH_TEST_URDF "/workspace/models/Psi_R1_visual_collision.urdf" CACHE FILEPATH "R1 model for joint planning integration tests")
  if(EXISTS "${MCL_JOINT_PATH_TEST_URDF}")
    add_executable(test_mcl_joint_path_planning_visualization tests/visualization.cpp)
    target_link_libraries(test_mcl_joint_path_planning_visualization PRIVATE mcl_joint_path_planning_support)
    add_test(NAME mcl_joint_path_planning_visualization COMMAND test_mcl_joint_path_planning_visualization
      "${MCL_JOINT_PATH_TEST_URDF}" "${CMAKE_CURRENT_SOURCE_DIR}/configs/cube.json")
    add_executable(test_mcl_joint_path_planning_solver tests/solver.cpp)
    target_link_libraries(test_mcl_joint_path_planning_solver PRIVATE mcl_joint_path_planning_support)
    add_test(NAME mcl_joint_path_planning_solver COMMAND test_mcl_joint_path_planning_solver
      "${MCL_JOINT_PATH_TEST_URDF}" "${CMAKE_CURRENT_SOURCE_DIR}/configs/free.json")
    add_test(NAME mcl_joint_path_planning_contract COMMAND ${Python3_EXECUTABLE}
      "${CMAKE_CURRENT_SOURCE_DIR}/tests/contract.py" --binary $<TARGET_FILE:mcl_joint_path_planning>
      --urdf "${MCL_JOINT_PATH_TEST_URDF}" --app "${CMAKE_CURRENT_SOURCE_DIR}"
      --output "${CMAKE_CURRENT_BINARY_DIR}/test-runs")
    add_test(NAME mcl_joint_path_planning_subscription COMMAND ${Python3_EXECUTABLE}
      "${CMAKE_CURRENT_SOURCE_DIR}/tests/subscription.py" --binary $<TARGET_FILE:mcl_joint_path_planning>
      --urdf "${MCL_JOINT_PATH_TEST_URDF}" --app "${CMAKE_CURRENT_SOURCE_DIR}"
      --output "${CMAKE_CURRENT_BINARY_DIR}/test-runs")
    add_test(NAME mcl_joint_path_planning_smooth_validation COMMAND ${Python3_EXECUTABLE}
      "${CMAKE_CURRENT_SOURCE_DIR}/tests/smooth_validation.py" --binary $<TARGET_FILE:mcl_joint_path_planning>
      --urdf "${MCL_JOINT_PATH_TEST_URDF}" --app "${CMAKE_CURRENT_SOURCE_DIR}"
      --output "${CMAKE_CURRENT_BINARY_DIR}/test-runs")
    set_tests_properties(mcl_joint_path_planning_smooth_validation PROPERTIES TIMEOUT 120)
    set_tests_properties(mcl_joint_path_planning_contract PROPERTIES TIMEOUT 120)
    set_tests_properties(mcl_joint_path_planning_subscription PROPERTIES TIMEOUT 45)
  endif()
endif()
