add_library(mcl_study_e09_support options.cpp solver.cpp loop.cpp allocation.cpp)
target_link_libraries(mcl_study_e09_support PUBLIC motion_control_core::motion_control_core libplaco jsoncpp_lib Threads::Threads)
mcl_add_app(TARGET mcl_study_e09 SOURCES main.cpp LIBRARIES mcl_study_e09_support)

if(BUILD_TESTING)
 add_executable(test_study_e09_allocation tests/allocation.cpp allocation.cpp)
 add_test(NAME apps.mcl_study_e09_allocation COMMAND test_study_e09_allocation)
endif()
