#!/usr/bin/env bash
set -eo pipefail
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp OPENBLAS_NUM_THREADS=1
source /opt/ros/humble/setup.bash
exec python3 /workspace/so101/scattered_task_server.py
