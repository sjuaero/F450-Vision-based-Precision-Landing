#!/bin/bash
set -e

PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}
PX4_BIN=$PX4_DIR/build/px4_sitl_default/bin/px4

export GZ_SIM_RESOURCE_PATH=$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds
export PX4_GZ_MODELS=$PX4_DIR/Tools/simulation/gz/models
export PX4_GZ_WORLDS=$PX4_DIR/Tools/simulation/gz/worlds
export PX4_GCS_HOST=${PX4_GCS_HOST:-172.18.144.1}

mkdir -p /tmp/px4_drone
cd /tmp/px4_drone

echo "[drone] attaching F450 to existing farmland world"
PX4_SYS_AUTOSTART=4022 \
PX4_SIM_MODEL=f450 \
PX4_GZ_WORLD=farmland \
PX4_GZ_MODEL_POSE="${PX4_GZ_MODEL_POSE:-0,0,0.295,0,0,0}" \
PX4_GZ_STANDALONE=1 \
"$PX4_BIN" -i 1