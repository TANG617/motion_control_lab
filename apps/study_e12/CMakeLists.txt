add_library(mcl_study_e12_support options.cpp solver.cpp planning.cpp loop.cpp)
target_include_directories(mcl_study_e12_support PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(mcl_study_e12_support PUBLIC motion_control_core::motion_control_core libplaco jsoncpp_lib eiquadprog::eiquadprog)
mcl_add_app(TARGET mcl_study_e12 SOURCES main.cpp LIBRARIES mcl_study_e12_support)
