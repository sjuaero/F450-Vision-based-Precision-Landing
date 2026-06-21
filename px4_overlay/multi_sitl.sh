#!/bin/bash
# ============================================================
# Multi-Vehicle SITL: r1_rover + F450 drone (farmland world)
#
# 포트 구성:
#   Rover (SYSID=1) : QGC UDP 14550  |  Offboard local UDP 14580
#   Drone (SYSID=2) : QGC UDP 14560  |  Offboard local UDP 14581
# ============================================================

PX4_DIR=$HOME/PX4-Autopilot
BUILD=$PX4_DIR/build/px4_sitl_default/bin/px4

export GZ_SIM_RESOURCE_PATH=$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds

echo "[1/3] 기존 px4/gz 프로세스 종료..."
pkill -f "bin/px4" 2>/dev/null
pkill -f "gz sim"  2>/dev/null
sleep 2

# --- 로버 (instance 0) : Gazebo + farmland 월드를 직접 시작 ---
echo "[2/3] 로버 + Gazebo 시작 (SYSID=1)..."
mkdir -p /tmp/px4_rover
cd /tmp/px4_rover
PX4_SYS_AUTOSTART=4009 \
PX4_GZ_WORLD=farmland \
PX4_GZ_MODEL=r1_rover \
PX4_GZ_MODEL_POSE="0,0,0,0,0,0" \
$BUILD -i 0 -d > /tmp/px4_rover.log 2>&1 &
ROVER_PID=$!

echo "  Gazebo 및 로버 초기화 대기중 (15초)..."
sleep 15

# --- 드론 (instance 1) : 이미 실행 중인 Gazebo에 합류 ---
echo "[3/3] F450 드론 시작 - 기존 Gazebo에 합류 (SYSID=2)..."
mkdir -p /tmp/px4_drone
cd /tmp/px4_drone
PX4_SYS_AUTOSTART=4022 \
PX4_GZ_MODEL=f450 \
PX4_GZ_MODEL_POSE="0,0,0.3,0,0,0" \
PX4_GZ_STANDALONE=1 \
$BUILD -i 1 -d > /tmp/px4_drone.log 2>&1 &
DRONE_PID=$!

echo ""
echo "=========================================="
echo "  Multi-Vehicle SITL 실행 완료!"
echo "=========================================="
echo ""
echo "  [로버  SYSID=1]"
echo "    QGC 수신       : UDP 14550"
echo "    PX4 GCS local  : UDP 18570"
echo "    Offboard local : UDP 14580"
echo ""
echo "  [드론  SYSID=2]"
echo "    QGC 수신       : UDP 14560"
echo "    PX4 GCS local  : UDP 18571"
echo "    Offboard local : UDP 14581"
echo ""
echo "  QGroundControl 연결:"
echo "    설정 -> 통신링크 -> 추가 -> UDP 14550  (로버)"
echo "    설정 -> 통신링크 -> 추가 -> UDP 14560  (드론)"
echo ""
echo "  로그 확인:"
echo "    tail -f /tmp/px4_rover.log"
echo "    tail -f /tmp/px4_drone.log"
echo ""
echo "  종료: Ctrl+C"

trap "echo '종료 중...'; kill $ROVER_PID $DRONE_PID 2>/dev/null; pkill -f 'bin/px4'; pkill -f 'gz sim'; exit 0" INT TERM
wait
