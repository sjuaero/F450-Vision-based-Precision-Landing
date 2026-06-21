#!/bin/bash
set -e

PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}
PX4_BIN=$PX4_DIR/build/px4_sitl_default/bin/px4
DRONE_START_POSE=${DRONE_START_POSE:-0,0,0.17,0,0,0}

export GZ_SIM_RESOURCE_PATH=$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds
export PX4_GZ_MODELS=$PX4_DIR/Tools/simulation/gz/models
export PX4_GZ_WORLDS=$PX4_DIR/Tools/simulation/gz/worlds
export PX4_GCS_HOST=${PX4_GCS_HOST:-172.18.144.1}

echo "[drone-landing] stopping old PX4/Gazebo processes..."
pkill -f "bin/px4"  2>/dev/null || true
pkill -f "gz sim"   2>/dev/null || true
pkill -f "gz-sim"   2>/dev/null || true
pkill -f "gz_bridge" 2>/dev/null || true

for _ in {1..20}; do
    timeout 1s gz topic -l 2>/dev/null | grep -q '^/world/' || break
    sleep 0.5
done

mkdir -p /tmp/px4_drone_landing
cd /tmp/px4_drone_landing

echo "[drone-landing] starting F450 + f450_landing world"
echo "[drone-landing] landing pad center: 0,0"
echo "[drone-landing] drone start pose: $DRONE_START_POSE"
PX4_SYS_AUTOSTART=4022 \
PX4_SIM_MODEL=f450 \
PX4_GZ_WORLD=f450_landing \
PX4_GZ_MODEL_POSE="$DRONE_START_POSE" \
PX4_UXRCE_DDS_NS="" \
exec "$PX4_BIN" -i 1
