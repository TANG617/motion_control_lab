add_library(mcl_study_e07_support options.cpp solver.cpp loop.cpp)
target_include_directories(mcl_study_e07_support PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(mcl_study_e07_support PUBLIC motion_control_core::motion_control_core libplaco jsoncpp_lib)
mcl_add_app(TARGET mcl_study_e07 SOURCES main.cpp LIBRARIES mcl_study_e07_support)
