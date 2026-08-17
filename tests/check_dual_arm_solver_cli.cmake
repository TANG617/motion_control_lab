if(NOT DEFINED APP)
  message(FATAL_ERROR "APP is required")
endif()

set(APP_COMMAND "${APP}")
if(DEFINED SUBCOMMAND AND NOT SUBCOMMAND STREQUAL "")
  list(APPEND APP_COMMAND "${SUBCOMMAND}")
endif()

foreach(SOLVER IN ITEMS mcc placo)
  foreach(BACKEND IN ITEMS proxqp eiquadprog)
    execute_process(
      COMMAND ${APP_COMMAND} --solver "${SOLVER}" --backend "${BACKEND}" --help
      RESULT_VARIABLE HELP_RESULT
      OUTPUT_VARIABLE HELP_OUTPUT
      ERROR_VARIABLE HELP_ERROR
    )
    if(NOT HELP_RESULT EQUAL 0 OR
       NOT "${HELP_OUTPUT}${HELP_ERROR}" MATCHES "--solver <mcc\\|placo>" OR
       NOT "${HELP_OUTPUT}${HELP_ERROR}" MATCHES "--backend <proxqp\\|eiquadprog>")
      message(
        FATAL_ERROR
        "${APP} --solver ${SOLVER} --backend ${BACKEND} --help failed:\n${HELP_OUTPUT}${HELP_ERROR}"
      )
    endif()
  endforeach()
endforeach()

execute_process(
  COMMAND ${APP_COMMAND} --solver
  RESULT_VARIABLE MISSING_RESULT
  OUTPUT_VARIABLE MISSING_OUTPUT
  ERROR_VARIABLE MISSING_ERROR
)
if(MISSING_RESULT EQUAL 0 OR
   NOT "${MISSING_OUTPUT}${MISSING_ERROR}" MATCHES "--solver requires a value")
  message(FATAL_ERROR "${APP} did not reject a missing --solver value")
endif()

execute_process(
  COMMAND ${APP_COMMAND} --solver invalid
  RESULT_VARIABLE INVALID_RESULT
  OUTPUT_VARIABLE INVALID_OUTPUT
  ERROR_VARIABLE INVALID_ERROR
)
if(INVALID_RESULT EQUAL 0 OR
   NOT "${INVALID_OUTPUT}${INVALID_ERROR}" MATCHES "--solver must be either 'mcc' or 'placo'")
  message(FATAL_ERROR "${APP} did not reject an invalid --solver value")
endif()

execute_process(
  COMMAND ${APP_COMMAND} --backend
  RESULT_VARIABLE MISSING_BACKEND_RESULT
  OUTPUT_VARIABLE MISSING_BACKEND_OUTPUT
  ERROR_VARIABLE MISSING_BACKEND_ERROR
)
if(MISSING_BACKEND_RESULT EQUAL 0 OR
   NOT "${MISSING_BACKEND_OUTPUT}${MISSING_BACKEND_ERROR}" MATCHES "--backend requires a value")
  message(FATAL_ERROR "${APP} did not reject a missing --backend value")
endif()

execute_process(
  COMMAND ${APP_COMMAND} --backend invalid
  RESULT_VARIABLE INVALID_BACKEND_RESULT
  OUTPUT_VARIABLE INVALID_BACKEND_OUTPUT
  ERROR_VARIABLE INVALID_BACKEND_ERROR
)
if(INVALID_BACKEND_RESULT EQUAL 0 OR
   NOT "${INVALID_BACKEND_OUTPUT}${INVALID_BACKEND_ERROR}" MATCHES
       "--backend must be either 'proxqp' or 'eiquadprog'")
  message(FATAL_ERROR "${APP} did not reject an invalid --backend value")
endif()
