#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
BAG_PATH="/workspace/artifacts/so101_pick_bag_$(date +%Y%m%d_%H%M%S)"
ros2 bag record -o "$BAG_PATH" /joint_states /so101_arm_controller/joint_trajectory /so101_gripper_controller/joint_trajectory /model/eraser/pose /clock > /workspace/artifacts/so101_pick_record.log 2>&1 &
RECORDER_PID=$!
trap 'kill -INT "$RECORDER_PID" 2>/dev/null || true' EXIT
sleep 5
python3 /workspace/so101/pick_material.py
sleep 1
kill -INT "$RECORDER_PID"
wait "$RECORDER_PID" || true
trap - EXIT
ros2 bag info "$BAG_PATH" > /workspace/artifacts/so101_pick_bag_info.txt
