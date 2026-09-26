#!/bin/bash
# E1: dose-response of Z support vs robot_cmd_rate_hz, interleaved
for rep in 1 2 3 4; do
  for r in 100 10 50 20 5; do
    pkill -f pr2_mujoco_sim 2>/dev/null; sleep 1
    /workspace/scripts/verify_run.sh E1_r${r}_${rep} robot_cmd_rate_hz:=$r
  done
done
echo SWEEP1_DONE
