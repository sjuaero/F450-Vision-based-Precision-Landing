#!/bin/bash
set -e

PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}
PX4_BIN=$PX4_DIR/build/px4_sitl_default/bin/px4

export GZ_SIM_RESOURCE_PATH=$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds
export PX4_GZ_MODELS=$PX4_DIR/Tools/simulation/gz/models
export PX4_GZ_WORLDS=$PX4_DIR/Tools/simulation/gz/worlds
export PX4_GCS_HOST=${PX4_GCS_HOST:-172.18.144.1}

echo "[rover] stopping old PX4/Gazebo processes..."
pkill -f "bin/px4" 2>/dev/null || true
pkill -f "px4.*-i" 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true
pkill -f "gz-sim" 2>/dev/null || true
pkill -f "ruby.*gz" 2>/dev/null || true

for _ in {1..20}; do
	if ! gz topic -l 2>/dev/null | grep -q '^/world/'; then
		break
	fi
	sleep 0.5
done

mkdir -p /tmp/px4_rover
cd /tmp/px4_rover

echo "[rover] starting farmland world and R1 rover"
PX4_SYS_AUTOSTART=4009 \
PX4_SIM_MODEL=r1_rover_clean \
PX4_GZ_WORLD=farmland \
PX4_GZ_MODEL_POSE="0,0,0,0,0,0" \
"$PX4_BIN" -i 0