add_library(mcl_study_e10_support options.cpp solver.cpp loop.cpp)
target_link_libraries(mcl_study_e10_support PUBLIC motion_control_core::motion_control_core libplaco jsoncpp_lib Threads::Threads)
mcl_add_app(TARGET mcl_study_e10 SOURCES main.cpp LIBRARIES mcl_study_e10_support)
