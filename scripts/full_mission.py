"""
full_mission.py
로버 이동 → 정지 → 드론 이륙 → 방역 → 복귀 → 착륙
"""

import asyncio
import math
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
from mavsdk.offboard import OffboardError, PositionNedYaw

# ─────────────────────────────────────────
# 설정
# ─────────────────────────────────────────

# PX4 SITL 기본 홈 GPS (Gazebo 원점)
HOME_LAT = 47.397742
HOME_LON = 8.545594

# 농경지 정보 (Gazebo ENU, 미터)
FARMLANDS = {
    "A": {"cx": -28.1, "cy":  14.55, "w": 52.2, "h": 25.1},
    "B": {"cx":  28.1, "cy":  14.55, "w": 52.2, "h": 25.1},
    "C": {"cx": -28.1, "cy": -14.55, "w": 52.2, "h": 25.1},
    "D": {"cx":  28.1, "cy": -14.55, "w": 52.2, "h": 25.1},
}

TARGET_FARM    = "A"    # 방역 대상 농경지
ROVER_SPEED    = 2.0    # 로버 이동 속도 (m/s)
ROVER_RADIUS   = 3.0    # 로버 목적지 도착 판정 반경 (m)

SPRAY_ALT      = 3.0    # 방역 고도 (m)
TRANSIT_ALT    = 8.0    # 이동 고도 (m)
LINE_SPACING   = 3.0    # 방역 줄 간격 (m)
MARGIN         = 1.0    # 농경지 경계 여유 (m)
ARRIVAL_RADIUS = 2.0    # 드론 waypoint 도착 판정 반경 (m)
WP_TIMEOUT     = 90     # waypoint 제한 시간 (s)
HOVER_ALT      = 3.0    # 로버 위 대기 고도 (m)

# ─────────────────────────────────────────
# 좌표 변환
# ─────────────────────────────────────────

def enu_to_ned(x, y, z=0.0):
    """Gazebo ENU → MAVSDK NED"""
    return y, x, -z

def ned_to_gps(north, east):
    """NED 오프셋(m) → GPS 좌표 (로버 미션용)"""
    lat = HOME_LAT + north / 111320.0
    lon = HOME_LON + east / (111320.0 * math.cos(math.radians(HOME_LAT)))
    return lat, lon

# ─────────────────────────────────────────
# 연결
# ─────────────────────────────────────────

async def connect(address, label):
    sys = System()
    await sys.connect(system_address=address)
    print(f"[{label}] 연결 대기 중...")
    async for state in sys.core.connection_state():
        if state.is_connected:
            print(f"[{label}] 연결 완료.")
            break
    return sys

# ─────────────────────────────────────────
# 로버 미션
# ─────────────────────────────────────────

async def wait_rover_ready(rover):
    print("[로버] EKF2 대기...")
    async for health in rover.telemetry.health():
        if health.is_local_position_ok:
            print("[로버] EKF2 준비.")
            break

async def drive_to_farmland(rover, farm_key):
    """로버를 농경지 입구(도로변)까지 이동"""
    farm = FARMLANDS[farm_key]

    # 목적지: 농경지 남쪽 도로변 (NS 도로 끝, 농경지 Y 중심)
    # farmland_A의 경우 도로 x=-2 지점, y=farm cy
    entry_x = -2.0 if farm["cx"] < 0 else 2.0
    entry_y = farm["cy"]
    target_n, target_e, _ = enu_to_ned(entry_x, entry_y)
    lat, lon = ned_to_gps(target_n, target_e)

    print(f"[로버] farmland_{farm_key} 입구로 이동 (N={target_n:.1f}, E={target_e:.1f})")

    # 미션 waypoint 1개 업로드
    wp = MissionItem(
        lat, lon,
        0.0,           # 지상 고도
        ROVER_SPEED,
        True,          # fly_through
        float("nan"), float("nan"),
        MissionItem.CameraAction.NONE,
        float("nan"), float("nan"),
        2.0,           # acceptance_radius
        float("nan"), float("nan"),
    )
    plan = MissionPlan([wp])
    await rover.mission.set_return_to_launch_after_mission(False)
    await rover.mission.upload_mission(plan)

    await rover.action.arm()
    await rover.mission.start_mission()

    # 도착 대기
    async for pos in rover.telemetry.position_velocity_ned():
        dn = pos.position.north_m - target_n
        de = pos.position.east_m  - target_e
        dist = math.hypot(dn, de)
        print(f"  [로버] 남은 거리: {dist:.1f}m", end="\r")
        if dist < ROVER_RADIUS:
            break

    print(f"\n[로버] 목적지 도착. 정지.")
    await rover.action.hold()
    await asyncio.sleep(2)

    # 정지 후 정확한 위치 반환 (드론 복귀 기준)
    async for pos in rover.telemetry.position_velocity_ned():
        rover_n = pos.position.north_m
        rover_e = pos.position.east_m
        break

    print(f"[로버] 최종 위치: N={rover_n:.1f} E={rover_e:.1f}")
    return rover_n, rover_e

# ─────────────────────────────────────────
# 드론 미션
# ─────────────────────────────────────────

async def wait_drone_ready(drone):
    print("[드론] EKF2 대기...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_local_position_ok:
            print("[드론] EKF2 준비.")
            break

async def arm_and_takeoff(drone, alt):
    print("[드론] ARM 중...")
    await drone.action.arm()
    await drone.action.set_takeoff_altitude(alt)
    print(f"[드론] 이륙 → {alt}m")
    await drone.action.takeoff()
    await asyncio.sleep(6)

async def start_offboard(drone, n, e, d):
    await drone.offboard.set_position_ned(PositionNedYaw(n, e, d, 0.0))
    try:
        await drone.offboard.start()
        print("[드론] Offboard 시작.")
    except OffboardError as err:
        print(f"[드론] Offboard 실패: {err}")
        raise

async def fly_to(drone, n, e, d, yaw=0.0, label=""):
    if label:
        print(f"  [드론] {label}")
    await drone.offboard.set_position_ned(PositionNedYaw(n, e, d, yaw))

async def wait_arrival(drone, n_tgt, e_tgt, radius=ARRIVAL_RADIUS, timeout=WP_TIMEOUT):
    deadline = asyncio.get_event_loop().time() + timeout
    async for pos in drone.telemetry.position_velocity_ned():
        dist = math.hypot(
            pos.position.north_m - n_tgt,
            pos.position.east_m  - e_tgt
        )
        if dist < radius:
            return True
        if asyncio.get_event_loop().time() > deadline:
            print(f"  [드론] timeout (dist={dist:.1f}m)")
            return False

def gen_lawnmower(farm_key):
    f = FARMLANDS[farm_key]
    cx, cy, w, h = f["cx"], f["cy"], f["w"], f["h"]
    x_min = cx - w / 2 + MARGIN
    x_max = cx + w / 2 - MARGIN
    y_min = cy - h / 2 + MARGIN
    y_max = cy + h / 2 - MARGIN

    wps, y, left = [], y_min, True
    while y <= y_max + 0.01:
        xs = (x_min, x_max) if left else (x_max, x_min)
        wps.append((xs[0], y, SPRAY_ALT))
        wps.append((xs[1], y, SPRAY_ALT))
        y += LINE_SPACING
        left = not left
    return wps

async def spray_mission(drone, rover_n, rover_e):
    waypoints = gen_lawnmower(TARGET_FARM)
    print(f"\n[드론] farmland_{TARGET_FARM} 방역 시작 ({len(waypoints)} waypoints)")

    # 농경지 진입 (이동 고도)
    n0, e0, _ = enu_to_ned(*waypoints[0])
    await start_offboard(drone, n0, e0, -TRANSIT_ALT)
    await fly_to(drone, n0, e0, -TRANSIT_ALT, label=f"farmland_{TARGET_FARM} 이동")
    await wait_arrival(drone, n0, e0, radius=2.0, timeout=120)

    # 방역 고도 하강
    await fly_to(drone, n0, e0, -SPRAY_ALT, label=f"방역 고도 {SPRAY_ALT}m 하강")
    await asyncio.sleep(3)

    # 방역 경로 비행
    for i, (x, y, z) in enumerate(waypoints):
        n, e, _ = enu_to_ned(x, y)
        await fly_to(drone, n, e, -z, label=f"WP {i+1}/{len(waypoints)}")
        await wait_arrival(drone, n, e)

    print("[드론] 방역 완료.")

    # 로버 위치로 복귀
    last_n, last_e, _ = enu_to_ned(waypoints[-1][0], waypoints[-1][1])
    print(f"\n[드론] 로버 위치로 복귀 (N={rover_n:.1f} E={rover_e:.1f})")
    await fly_to(drone, last_n, last_e, -TRANSIT_ALT, label="복귀 고도 상승")
    await asyncio.sleep(3)
    await fly_to(drone, rover_n, rover_e, -TRANSIT_ALT, label="로버 위치로 이동")
    await wait_arrival(drone, rover_n, rover_e, radius=2.0, timeout=150)

    # 로버 위 호버링 후 착륙
    await fly_to(drone, rover_n, rover_e, -HOVER_ALT, label=f"로버 위 {HOVER_ALT}m 호버링")
    await asyncio.sleep(4)

    print("[드론] 착륙")
    await drone.offboard.stop()
    await drone.action.land()

# ─────────────────────────────────────────
# 메인
# ─────────────────────────────────────────

async def main():
    # 로버(14540)와 드론(14541) 동시 연결
    rover, drone = await asyncio.gather(
        connect("udp://:14540", "로버"),
        connect("udp://:14541", "드론"),
    )

    await asyncio.gather(
        wait_rover_ready(rover),
        wait_drone_ready(drone),
    )

    # 1. 로버 이동
    rover_n, rover_e = await drive_to_farmland(rover, TARGET_FARM)

    # 2. 드론 이륙 + 방역 + 복귀 + 착륙
    await arm_and_takeoff(drone, TRANSIT_ALT)
    await spray_mission(drone, rover_n, rover_e)

    print("\n[미션 완료]")


asyncio.run(main())