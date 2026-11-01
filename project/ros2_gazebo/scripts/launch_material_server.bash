#!/usr/bin/env bash
set -euo pipefail
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export OPENBLAS_NUM_THREADS=1
source /opt/ros/humble/setup.bash
exec python3 /workspace/so101/task_server.py
