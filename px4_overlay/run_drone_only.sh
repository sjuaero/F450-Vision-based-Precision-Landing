#!/bin/bash
set -e

PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}
PX4_BIN=$PX4_DIR/build/px4_sitl_default/bin/px4

export GZ_SIM_RESOURCE_PATH=$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds
export PX4_GZ_MODELS=$PX4_DIR/Tools/simulation/gz/models
export PX4_GZ_WORLDS=$PX4_DIR/Tools/simulation/gz/worlds
export PX4_GCS_HOST=${PX4_GCS_HOST:-172.18.144.1}

echo "[drone-only] stopping old PX4/Gazebo processes..."
pkill -f "bin/px4" 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true
pkill -f "gz-sim" 2>/dev/null || true
pkill -f "ruby.*gz" 2>/dev/null || true

for _ in {1..20}; do
    if ! gz topic -l 2>/dev/null | grep -q '^/world/'; then
        break
    fi
    sleep 0.5
done

mkdir -p /tmp/px4_drone_only
cd /tmp/px4_drone_only

echo "[drone-only] starting farmland world and F450 drone (instance 1, port 14541)"
# PX4_GZ_STANDALONE 없음 → px4-rc.gzsim이 Gazebo 월드를 직접 시작
PX4_SYS_AUTOSTART=4022 \
PX4_SIM_MODEL=f450 \
PX4_GZ_WORLD=farmland \
PX4_GZ_MODEL_POSE="${PX4_GZ_MODEL_POSE:-0,0,0.295,0,0,0}" \
"$PX4_BIN" -i 1
