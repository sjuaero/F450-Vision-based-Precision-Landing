"""
spray_mission.py (원래 px4-drone-rover-sim의 drone_mission.py)
Full spray mission: takeoff → lawnmower spray → return → precision land
"""

import asyncio
import math
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

# ─────────────────────────────────────────
# MISSION PARAMETERS
# ─────────────────────────────────────────

# Farmland boundaries in Gazebo ENU (meters from world origin)
FARMLANDS = {
    "A": {"cx": -28.1, "cy":  14.55, "w": 52.2, "h": 25.1},
    "B": {"cx":  28.1, "cy":  14.55, "w": 52.2, "h": 25.1},
    "C": {"cx": -28.1, "cy": -14.55, "w": 52.2, "h": 25.1},
    "D": {"cx":  28.1, "cy": -14.55, "w": 52.2, "h": 25.1},
}

TARGET_FARM    = "A"    # 방역 대상 농경지
SPRAY_ALT      = 3.0    # 방역 비행 고도 (m)
TRANSIT_ALT    = 8.0    # 이동 고도 (m, 장애물 회피)
LINE_SPACING   = 3.0    # 방역 줄 간격 = 살포 폭 (m)
MARGIN         = 1.0    # 농경지 경계 안쪽 여유 (m)
ARRIVAL_RADIUS = 2.0    # waypoint 도착 판정 반경 (m)
WP_TIMEOUT     = 90     # waypoint 이동 제한 시간 (s)

# 로버 정차 위치 (Gazebo ENU) — 스크립트 실행 전 로버가 여기 있어야 함
ROVER_ENU_X = 0.0
ROVER_ENU_Y = 0.0
HOVER_ALT   = 3.0       # 로버 위 대기 고도 (ArUco 탐색 시작 높이)

# ─────────────────────────────────────────
# 좌표 변환: Gazebo ENU → MAVSDK NED
# Gazebo: X=East, Y=North, Z=Up
# NED:    North, East, Down
# ─────────────────────────────────────────
def enu_to_ned(x, y, z=0.0):
    return y, x, -z  # (north, east, down)


# ─────────────────────────────────────────
# 로즈모어(boustrophedon) 경로 생성
# ─────────────────────────────────────────
def gen_lawnmower(farm_key, alt, spacing=LINE_SPACING, margin=MARGIN):
    f = FARMLANDS[farm_key]
    cx, cy, w, h = f["cx"], f["cy"], f["w"], f["h"]

    x_min = cx - w / 2 + margin
    x_max = cx + w / 2 - margin
    y_min = cy - h / 2 + margin
    y_max = cy + h / 2 - margin

    waypoints = []
    y = y_min
    left_to_right = True
    while y <= y_max + 0.01:
        xs = (x_min, x_max) if left_to_right else (x_max, x_min)
        waypoints.append((xs[0], y, alt))
        waypoints.append((xs[1], y, alt))
        y += spacing
        left_to_right = not left_to_right
    return waypoints


# ─────────────────────────────────────────
# 드론 연결
# ─────────────────────────────────────────
async def connect(address="udp://:14541"):
    drone = System()
    await drone.connect(system_address=address)
    print(f"[연결] {address} 대기 중...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("[연결] 완료.")
            break
    return drone


async def wait_ready(drone):
    print("[EKF2] 수렴 대기 중...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_local_position_ok:
            print("[EKF2] 완료.")
            break


# ─────────────────────────────────────────
# 비행 제어
# ─────────────────────────────────────────
async def arm_and_takeoff(drone, alt):
    print("[ARM] 시동 중...")
    await drone.action.arm()
    await drone.action.set_takeoff_altitude(alt)
    print(f"[이륙] {alt}m 목표...")
    await drone.action.takeoff()
    await asyncio.sleep(6)
    print("[이륙] 완료.")


async def start_offboard(drone, north, east, down):
    await drone.offboard.set_position_ned(PositionNedYaw(north, east, down, 0.0))
    try:
        await drone.offboard.start()
        print("[Offboard] 시작.")
    except OffboardError as e:
        print(f"[Offboard] 실패: {e}")
        raise


async def fly_to(drone, north, east, down, yaw=0.0, label=""):
    tag = f"  → {label}" if label else f"  → N={north:+.1f} E={east:+.1f} D={down:+.1f}"
    print(tag)
    await drone.offboard.set_position_ned(PositionNedYaw(north, east, down, yaw))


async def wait_arrival(drone, n_tgt, e_tgt, radius=ARRIVAL_RADIUS, timeout=WP_TIMEOUT):
    deadline = asyncio.get_event_loop().time() + timeout
    async for pos in drone.telemetry.position_velocity_ned():
        dn = pos.position.north_m - n_tgt
        de = pos.position.east_m  - e_tgt
        if math.hypot(dn, de) < radius:
            return True
        if asyncio.get_event_loop().time() > deadline:
            print(f"  [timeout] 거리={math.hypot(dn,de):.1f}m")
            return False


# ─────────────────────────────────────────
# ArUco 정밀 착륙 (Phase 2 stub)
# ─────────────────────────────────────────
async def aruco_precision_land(drone):
    """
    Phase 2에서 구현 예정:
    - /aruco_pose 토픽 구독 (ROS2 노드가 마커 오프셋 발행)
    - 오프셋 보정하며 속도 제어로 마커 중앙 정렬
    - 마커 추적하며 단계적 하강
    """
    print("[착륙] ArUco 정밀 착륙 (Phase 2 미구현 → 직접 착륙)")
    await drone.action.land()


# ─────────────────────────────────────────
# 메인 미션
# ─────────────────────────────────────────
async def main():
    drone = await connect()
    await wait_ready(drone)

    # ── 이륙 ──────────────────────────────
    await arm_and_takeoff(drone, TRANSIT_ALT)

    # ── 경로 생성 ─────────────────────────
    waypoints = gen_lawnmower(TARGET_FARM, SPRAY_ALT)
    print(f"\n[경로] farmland_{TARGET_FARM}: {len(waypoints)} waypoints")

    # ── 농경지 진입 (이동 고도) ──────────
    n0, e0, _ = enu_to_ned(*waypoints[0])
    await start_offboard(drone, n0, e0, -TRANSIT_ALT)
    await fly_to(drone, n0, e0, -TRANSIT_ALT, label=f"farmland_{TARGET_FARM} 진입 이동")
    await wait_arrival(drone, n0, e0, radius=2.0, timeout=120)

    # ── 방역 고도로 하강 ──────────────────
    await fly_to(drone, n0, e0, -SPRAY_ALT, label=f"방역 고도 {SPRAY_ALT}m 하강")
    await asyncio.sleep(3)

    # ── 방역 비행 ─────────────────────────
    print(f"\n[방역 시작] farmland_{TARGET_FARM}")
    for i, (x, y, z) in enumerate(waypoints):
        north, east, down = enu_to_ned(x, y, z)
        await fly_to(drone, north, east, -z,
                     label=f"WP {i+1}/{len(waypoints)}  ({x:.1f}, {y:.1f})")
        await wait_arrival(drone, north, east, radius=ARRIVAL_RADIUS, timeout=WP_TIMEOUT)
    print("[방역] 완료.")

    # ── 로버로 복귀 ───────────────────────
    rover_n, rover_e, _ = enu_to_ned(ROVER_ENU_X, ROVER_ENU_Y)
    last_x, last_y = waypoints[-1][0], waypoints[-1][1]
    cur_n, cur_e, _ = enu_to_ned(last_x, last_y)

    print(f"\n[복귀] 로버 위치로 이동 N={rover_n:.1f} E={rover_e:.1f}")
    await fly_to(drone, cur_n, cur_e, -TRANSIT_ALT, label="복귀 고도 상승")
    await asyncio.sleep(3)

    await fly_to(drone, rover_n, rover_e, -TRANSIT_ALT, label="로버 위치로 이동")
    await wait_arrival(drone, rover_n, rover_e, radius=1.5, timeout=150)

    await fly_to(drone, rover_n, rover_e, -HOVER_ALT, label=f"로버 위 {HOVER_ALT}m 호버링")
    await asyncio.sleep(4)

    # ── 정밀 착륙 ─────────────────────────
    print("\n[착륙] 정밀 착륙 시작")
    await aruco_precision_land(drone)
    print("\n[미션 완료]")


asyncio.run(main())
