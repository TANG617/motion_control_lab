#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct mcl_opensot_request
{
  const char* urdf_path;
  const char* base_frame;
  const char* end_effector_frame;
  const char* const* joint_names;
  const double* initial_positions;
  const double* goal_positions;
  size_t joint_count;
  uint32_t maximum_iterations;
  double position_tolerance_m;
  double cartesian_lambda;
  double postural_lambda;
  double regularization;
  double velocity_limit_period_s;
} mcl_opensot_request;

typedef struct mcl_opensot_trace_sample
{
  uint32_t iteration;
  double target_position_m[3];
  double achieved_position_m[3];
  double position_error_m;
  double solve_time_us;
  const double* joint_positions;
  size_t joint_count;
} mcl_opensot_trace_sample;

typedef void (*mcl_opensot_trace_callback)(
  void* user_data,
  const mcl_opensot_trace_sample* sample);

typedef struct mcl_opensot_result
{
  uint32_t converged;
  uint32_t iterations;
  double initial_position_error_m;
  double final_position_error_m;
  double mean_solve_time_us;
  double max_solve_time_us;
  uint32_t joint_limit_violation_count;
} mcl_opensot_result;

int mcl_opensot_solve_position_ik(
  const mcl_opensot_request* request,
  mcl_opensot_trace_callback trace_callback,
  void* trace_user_data,
  mcl_opensot_result* result,
  char* error_message,
  size_t error_message_capacity);

const char* mcl_opensot_upstream_revision(void);

#ifdef __cplusplus
}
#endif
