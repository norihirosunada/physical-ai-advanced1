#!/usr/bin/env bash
set -eo pipefail
export SO101_WORLD=/workspace/worlds/so101_pick.sdf
exec bash /workspace/scripts/launch_so101.bash
