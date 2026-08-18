if(NOT DEFINED APP)
  message(FATAL_ERROR "APP is required")
endif()

execute_process(
  COMMAND "${APP}" teleop --help
  RESULT_VARIABLE TELEOP_RESULT
  OUTPUT_VARIABLE TELEOP_OUTPUT
  ERROR_VARIABLE TELEOP_ERROR
)
if(NOT TELEOP_RESULT EQUAL 0 OR
   NOT "${TELEOP_OUTPUT}${TELEOP_ERROR}" MATCHES "fixed at 100 Hz")
  message(FATAL_ERROR "baseline teleop help failed:\n${TELEOP_OUTPUT}${TELEOP_ERROR}")
endif()

execute_process(
  COMMAND "${APP}" replay --help
  RESULT_VARIABLE REPLAY_RESULT
  OUTPUT_VARIABLE REPLAY_OUTPUT
  ERROR_VARIABLE REPLAY_ERROR
)
if(NOT REPLAY_RESULT EQUAL 0 OR
   NOT "${REPLAY_OUTPUT}${REPLAY_ERROR}" MATCHES "--target-period-ms 10")
  message(FATAL_ERROR "baseline replay help failed:\n${REPLAY_OUTPUT}${REPLAY_ERROR}")
endif()

execute_process(
  COMMAND "${APP}" teleop --rate 20
  RESULT_VARIABLE RATE_RESULT
  OUTPUT_VARIABLE RATE_OUTPUT
  ERROR_VARIABLE RATE_ERROR
)
if(RATE_RESULT EQUAL 0 OR
   NOT "${RATE_OUTPUT}${RATE_ERROR}" MATCHES "fixed at 100 Hz")
  message(FATAL_ERROR "baseline accepted a teleop rate override")
endif()

execute_process(
  COMMAND "${APP}" teleop --solver placo
  RESULT_VARIABLE SOLVER_RESULT
  OUTPUT_VARIABLE SOLVER_OUTPUT
  ERROR_VARIABLE SOLVER_ERROR
)
if(SOLVER_RESULT EQUAL 0 OR
   NOT "${SOLVER_OUTPUT}${SOLVER_ERROR}" MATCHES "unknown option: --solver")
  message(FATAL_ERROR "baseline accepted --solver")
endif()

execute_process(
  COMMAND "${APP}" replay
    --urdf /tmp/missing.urdf
    --input /tmp/missing.csv
    --input-format csv
    --left-stream left
    --right-stream right
    --timestamp-source csv_timestamp
    --target-period-ms 20
    --ui none
    --output-dir /tmp/mcl-baseline-missing
  RESULT_VARIABLE PERIOD_RESULT
  OUTPUT_VARIABLE PERIOD_OUTPUT
  ERROR_VARIABLE PERIOD_ERROR
)
if(PERIOD_RESULT EQUAL 0 OR
   NOT "${PERIOD_OUTPUT}${PERIOD_ERROR}" MATCHES "must be exactly 10")
  message(FATAL_ERROR "baseline accepted a non-production replay period")
endif()
