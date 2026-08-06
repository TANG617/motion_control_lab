if(NOT DEFINED APP)
  message(FATAL_ERROR "APP is required")
endif()

execute_process(
  COMMAND "${APP}" --help
  RESULT_VARIABLE APP_RESULT
  OUTPUT_VARIABLE APP_OUTPUT
  ERROR_VARIABLE APP_ERROR
)
if(NOT APP_RESULT EQUAL 0)
  message(FATAL_ERROR "${APP} --help failed:\n${APP_OUTPUT}${APP_ERROR}")
endif()

set(APP_HELP "${APP_OUTPUT}${APP_ERROR}")
if(APP_HELP MATCHES "[Rr]1")
  message(FATAL_ERROR "${APP} --help exposes the robot model:\n${APP_HELP}")
endif()
