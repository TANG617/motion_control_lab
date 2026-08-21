function(mcl_add_app)
  set(options NO_INSTALL NO_HELP_SMOKE)
  set(one_value_args TARGET)
  set(multi_value_args SOURCES LIBRARIES INCLUDE_DIRECTORIES)
  cmake_parse_arguments(MCL_APP "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})
  if(NOT MCL_APP_TARGET)
    message(FATAL_ERROR "mcl_add_app requires TARGET")
  endif()
  add_executable(${MCL_APP_TARGET} ${MCL_APP_SOURCES})
  set_target_properties(
    ${MCL_APP_TARGET}
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
  )
  if(MCL_APP_INCLUDE_DIRECTORIES)
    target_include_directories(${MCL_APP_TARGET} PRIVATE ${MCL_APP_INCLUDE_DIRECTORIES})
  endif()
  if(MCL_APP_LIBRARIES)
    target_link_libraries(${MCL_APP_TARGET} PRIVATE ${MCL_APP_LIBRARIES})
  endif()
  if(NOT MCL_APP_NO_INSTALL)
    install(TARGETS ${MCL_APP_TARGET} RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
    file(GLOB MCL_APP_SCRIPTS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.sh")
    if(MCL_APP_SCRIPTS)
      install(
        PROGRAMS ${MCL_APP_SCRIPTS}
        DESTINATION "${CMAKE_INSTALL_LIBEXECDIR}/motion-control-lab/${MCL_APP_TARGET}"
      )
    endif()
  endif()
  if(BUILD_TESTING AND NOT MCL_APP_NO_HELP_SMOKE)
    add_test(
      NAME apps.${MCL_APP_TARGET}_help
      COMMAND
        "${CMAKE_COMMAND}"
        "-DAPP=$<TARGET_FILE:${MCL_APP_TARGET}>"
        -P "${PROJECT_SOURCE_DIR}/tests/check_app_help.cmake"
    )
  endif()
endfunction()
