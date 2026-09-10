set(MCL_PSIBOT_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/psi_sdk" CACHE PATH
    "Complete Psi SDK package (paired include and lib/<arch> directories)")
set(psibot_arch "${CMAKE_SYSTEM_PROCESSOR}")
if(psibot_arch MATCHES "^(aarch64|arm64)$")
  set(psibot_arch aarch64)
elseif(psibot_arch MATCHES "^(x86_64|amd64|AMD64)$")
  set(psibot_arch x86_64)
else()
  message(FATAL_ERROR "Psi SDK has no package for ${CMAKE_SYSTEM_PROCESSOR}")
endif()
set(psibot_libdir "${MCL_PSIBOT_SDK_ROOT}/lib/${psibot_arch}")
add_library(mcl_psibot_sdk SHARED IMPORTED)
set_target_properties(mcl_psibot_sdk PROPERTIES
  IMPORTED_LOCATION "${psibot_libdir}/libpsi_sdk.so"
  INTERFACE_INCLUDE_DIRECTORIES "${MCL_PSIBOT_SDK_ROOT}/include")
add_library(mcl_psibot_teleop_support
  options.cpp client.cpp loop.cpp tui_projection.cpp)
add_library(motion_control_lab::psibot_teleop_support ALIAS mcl_psibot_teleop_support)
target_include_directories(mcl_psibot_teleop_support PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(mcl_psibot_teleop_support PRIVATE
  MCL_PSIBOT_SDK_ORIGIN="${MCL_PSIBOT_SDK_ROOT}")
target_link_libraries(mcl_psibot_teleop_support PUBLIC mcl_psibot_sdk
  motion_control_lab::keyboard_teleop motion_control_lab::cartesian_teleop
  motion_control_lab::terminal_frontend motion_control_lab::tui
  motion_control_lab::scheduler jsoncpp_lib)
mcl_add_app(TARGET mcl_psibot_teleop SOURCES main.cpp LIBRARIES mcl_psibot_teleop_support)
set_target_properties(mcl_psibot_teleop PROPERTIES
  BUILD_RPATH "${psibot_libdir}"
  INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}/mcl_psibot_teleop"
  INSTALL_RPATH_USE_LINK_PATH FALSE)
install(DIRECTORY "${psibot_libdir}/" DESTINATION "${CMAKE_INSTALL_LIBDIR}/mcl_psibot_teleop"
  FILES_MATCHING PATTERN "*.so*")
install(FILES README.md DESTINATION "${CMAKE_INSTALL_DATADIR}/motion-control-lab/psibot_teleop")
install(DIRECTORY foxglove/ DESTINATION "${CMAKE_INSTALL_DATADIR}/motion-control-lab/psibot_teleop/foxglove"
  FILES_MATCHING PATTERN "*.json" PATTERN "*.md" PATTERN "*.py")
install(DIRECTORY "${MCL_PSIBOT_SDK_ROOT}/" DESTINATION "${CMAKE_INSTALL_DATADIR}/motion-control-lab/psibot_teleop/sdk"
  FILES_MATCHING PATTERN "*.md" PATTERN "*.json" PATTERN "SHA256SUMS" PATTERN "licenses/*")
if(BUILD_TESTING)
  add_executable(test_psibot_teleop tests/unit.cpp)
  target_link_libraries(test_psibot_teleop PRIVATE mcl_psibot_teleop_support)
  set_target_properties(test_psibot_teleop PROPERTIES BUILD_RPATH "${psibot_libdir}")
  add_test(NAME apps.psibot_teleop_unit COMMAND test_psibot_teleop)
  add_test(NAME apps.psibot_teleop_sdk COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/sdk_package.py" "${MCL_PSIBOT_SDK_ROOT}")
  if(MCL_ENABLE_TUI)
    add_test(NAME apps.psibot_teleop_pty COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/pty_session.py" "$<TARGET_FILE:mcl_psibot_teleop>")
    set_tests_properties(apps.psibot_teleop_pty PROPERTIES TIMEOUT 90)
  endif()
endif()
