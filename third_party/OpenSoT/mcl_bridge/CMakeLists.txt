add_library(mcl_opensot_bridge SHARED src/mcl_opensot_bridge.cpp)

target_include_directories(
    mcl_opensot_bridge
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(
    mcl_opensot_bridge
    PRIVATE
        OpenSoT
        OpenSotBackEndOSQP
)
target_compile_features(mcl_opensot_bridge PRIVATE cxx_std_20)
if(UNIX AND NOT APPLE)
    # Keep the private dependency prefixes usable for OpenSoT's transitive
    # XBot2/ROS libraries after the bridge is relocated by the Lab install.
    target_link_options(mcl_opensot_bridge PRIVATE "LINKER:--disable-new-dtags")
endif()
set_target_properties(
    mcl_opensot_bridge
    PROPERTIES
        OUTPUT_NAME mcl_opensot_bridge
        INSTALL_RPATH "$ORIGIN;/opt/motion-control-lab/opensot-deps/lib;/opt/openrobots/lib;/opt/ros/jazzy/lib"
)

install(
    TARGETS mcl_opensot_bridge
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(
    FILES include/mcl_opensot_bridge.h
    DESTINATION include
)
