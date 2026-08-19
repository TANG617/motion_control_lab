export LD_LIBRARY_PATH="/workspace/install/algorithm/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

chrt -f 60 taskset -c 5-7 \
  /workspace/install/algorithm/bin/mcl_planned_grouped_servo_step replay \
  --urdf /workspace/products/synrobot/modules/common/robot_description/psi_r1/urdf/Psi_R1_rev1.urdf \
  --input /workspace/fixtures/raw/sliced_RW1AZHYCSEFT5_RW1AZHYCSEFT5260310002_20260731015829_0.mcap \
  --input-format mcap \
  --left-stream /hal/tracker/htc/left/calib_target_pose \
  --right-stream /hal/tracker/htc/right/calib_target_pose \
  --initial-joint-state-stream /mc/ik/joint_states \
  --timestamp-source header_stamp \
  --target-period-ms 10 \
  --pairing-policy nearest \
  --nearest-tolerance-ms 5 \
  --unmatched-policy drop_with_diagnostics \
  --execution-mode realtime \
  --playback-rate 1 \
  --red-rate 1000 \
  --yellow-rate 100 \
  --deadline-policy monitor \
  --no-mcap \
  --output-root /workspace/runs/mcl_planned_grouped_servo_step \
  --ui tui \
  --port 8765 \
--max-linear-velocity-mps 0.9 \
--max-linear-acceleration-mps2 5.0 \
--max-linear-jerk-mps3 80.0 \
--max-angular-velocity-rps 3.0 \
--max-angular-acceleration-rps2 20.0 \
--max-angular-jerk-rps3 150.0 \
--start-paused \
# --mcap /workspace/labs/motion-control-lab/experiments/E05/runs/test.mcap