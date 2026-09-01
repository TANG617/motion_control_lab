if(NOT DEFINED APP)
  message(FATAL_ERROR "APP is required")
endif()

set(APP_COMMAND "${APP}")
if(DEFINED PROFILE AND NOT PROFILE STREQUAL "")
  list(APPEND APP_COMMAND --profile "${PROFILE}")
endif()

execute_process(
  COMMAND ${APP_COMMAND} teleop --help
  RESULT_VARIABLE TELEOP_RESULT
  OUTPUT_VARIABLE TELEOP_OUTPUT
  ERROR_VARIABLE TELEOP_ERROR
)
if(NOT TELEOP_RESULT EQUAL 0 OR NOT "${TELEOP_OUTPUT}${TELEOP_ERROR}" MATCHES "--ui <tui\\|none>")
  message(FATAL_ERROR "${APP} teleop --help failed:\n${TELEOP_OUTPUT}${TELEOP_ERROR}")
endif()

execute_process(
  COMMAND ${APP_COMMAND} replay --help
  RESULT_VARIABLE REPLAY_RESULT
  OUTPUT_VARIABLE REPLAY_OUTPUT
  ERROR_VARIABLE REPLAY_ERROR
)
if(NOT REPLAY_RESULT EQUAL 0 OR
   NOT "${REPLAY_OUTPUT}${REPLAY_ERROR}" MATCHES "--target-period-ms")
  message(FATAL_ERROR "${APP} replay --help failed:\n${REPLAY_OUTPUT}${REPLAY_ERROR}")
endif()

execute_process(
  COMMAND ${APP_COMMAND} replay
    --urdf /tmp/missing.urdf
    --input /tmp/missing.mcap
    --left-stream left
    --right-stream right
    --ui none
  RESULT_VARIABLE PERIOD_RESULT
  OUTPUT_VARIABLE PERIOD_OUTPUT
  ERROR_VARIABLE PERIOD_ERROR
)
if(PERIOD_RESULT EQUAL 0 OR
   NOT "${PERIOD_OUTPUT}${PERIOD_ERROR}" MATCHES "--target-period-ms is required")
  message(FATAL_ERROR "${APP} did not require --target-period-ms")
endif()

if(DEFINED PLANNED AND PLANNED)
  if(NOT "${TELEOP_OUTPUT}${TELEOP_ERROR}" MATCHES "--max-linear-jerk-mps3" OR
     NOT "${TELEOP_OUTPUT}${TELEOP_ERROR}" MATCHES "--max-angular-jerk-rps3")
    message(FATAL_ERROR "${APP} planned limit help is incomplete")
  endif()
endif()
