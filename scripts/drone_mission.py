"""
drone_mission.py (ROS 2 오프셋 연동 정밀착륙 버전)
"""
import asyncio
import math
import threading
import time
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw, PositionNedYaw

# ROS 2 Python 라이브러리
import rclpy
from geometry_msgs.msg import Point

# 전역 변수 (카메라 노드가 보내주는 데이터를 실시간 저장)
target_offset_x = 0.0
target_offset_y = 0.0
estimated_alt   = 0.0
data_received   = False

# ── ROS 2 Subscriber 스레드 함수 ──────────────────────────
def ros2_thread_entry():
    global target_offset_x, target_offset_y, estimated_alt, data_received
    
    rclpy.init()
    node = rclpy.create_node('mission_bridge_node')
    
    def callback(msg):
        global target_offset_x, target_offset_y, estimated_alt, data_received
        target_offset_x = msg.x
        target_offset_y = msg.y
        estimated_alt   = msg.z
        data_received   = True

    node.create_subscription(Point, '/landing/offset', callback, 10)
    print("[ROS2 브리지] /landing/offset 토픽 구독 중...")
    rclpy.spin(node)

# ROS 2를 백그라운드 스레드로 실행
threading.Thread(target=ros2_thread_entry, daemon=True).start()


# ── MAVSDK 드론 미션 및 제어 ──────────────────────────────
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

async def arm_and_takeoff(drone, alt):
    print("[ARM] 시동 중...")
    await drone.action.arm()
    await drone.action.set_takeoff_altitude(alt)
    print(f"[이륙] {alt}m 목표...")
    await drone.action.takeoff()
    
    arrival_margin = 0.85  # 목표 고도의 85% 도달 시 다음 단계로 진행 (완전히 안 채워도 됨)
    last_print = 0.0
    async for position in drone.telemetry.position():
        cur_alt = position.relative_altitude_m
        now = time.monotonic()
        if now - last_print >= 1.0:   # 콘솔 스팸 방지: 1초에 한 번만 출력
            print(f"[이륙] 현재 고도: {cur_alt:.2f}m / 목표: {alt:.1f}m")
            last_print = now
        if cur_alt >= (alt * arrival_margin):
            break
    print("[이륙] 완료.")

async def start_offboard_with_retry(drone, hold_setpoint, retries=5):
    """오프보드 진입 전 setpoint를 충분히 흘려주고, NO_SETPOINT_SET 등으로
    실패하면 setpoint를 다시 흘리고 재시도한다 (시뮬레이터 부하로 실시간성이
    떨어질 때 한 번에 안 들어가는 경우가 있어 바로 죽지 않게 방어)."""
    for attempt in range(1, retries + 1):
        for _ in range(20):
            await drone.offboard.set_position_ned(hold_setpoint)
            await asyncio.sleep(0.05)
        try:
            await drone.offboard.start()
            return
        except OffboardError as e:
            print(f"[Offboard] 진입 실패({attempt}/{retries}): {e}. setpoint 재전송 후 재시도...")
    raise RuntimeError(f"[Offboard] {retries}회 재시도에도 진입 실패")

# ─────────────────────────────────────────
# 드론 비행 제어 시스템과 연동된 정밀 착륙 알고리즘
# ─────────────────────────────────────────
async def aruco_precision_land(drone):
    global target_offset_x, target_offset_y, estimated_alt, data_received
    print("\n[정밀착륙] 알고리즘 연동 제어 시작...")

    # 초기 오프보드 속도 모드 셋팅 (정지 상태)
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
    try:
        await drone.offboard.start()
        print("[Offboard] 속도 제어 모드 진입 성공.")
    except OffboardError as e:
        print(f"[Offboard] 실패: {e}, 일반 강제 착륙합니다.")
        await drone.action.land()
        return

    current_drone_alt = 10.0
    position_stream = drone.telemetry.position()  # 루프마다 새로 구독하지 않고 같은 스트림을 계속 읽음

    while True:
        # 드론 현재 고도 확인
        pos = await position_stream.__anext__()
        current_drone_alt = pos.relative_altitude_m

        # 고도가 0.5m 이하로 내려오면 정밀제어를 멈추고 최종 착륙터치업
        if current_drone_alt < 0.5:
            print("[정밀착륙] 지면 근접, 최종 착륙 명령 수행.")
            break

        if not data_received:
            # 카메라 노드에서 토픽이 안 올 때: 제자리 호버링하며 대기
            await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
            await asyncio.sleep(0.1)
            continue

        # 피드백 제어 (픽셀 오프셋을 드론의 속도(m/s) 명령으로 변환)
        # target_offset_x/y = "타겟 중심 - 영상 중심" 이므로, 이 값은 그대로
        # "드론이 타겟에 다가가기 위해 움직여야 하는 방향"과 같은 부호다.
        # (예전 코드는 이 값을 한번 더 반전시켜서 타겟에서 점점 멀어지는
        #  양성 피드백(발산)이 발생했었다 — 부호 반전 제거가 핵심 수정.)
        # 카메라 영상 좌표계: +X가 오른쪽 → NED +East, +Y가 아래쪽 → NED +North
        gain = 0.001           # 드론 반응 속도 조절용 가중치 (기체 움직임 보고 조절 가능)
        max_horiz_vel = 1.0    # m/s — 오프셋이 클 때 과도하게 튀어나가지 않도록 속도 제한

        vel_east  = max(-max_horiz_vel, min(max_horiz_vel, target_offset_x * gain))
        vel_north = max(-max_horiz_vel, min(max_horiz_vel, target_offset_y * gain))

        offset_dist = math.hypot(target_offset_x, target_offset_y)

        # 하강 속도: 고도 기준 기본값에서 x/y 정렬 품질에 따라 매끄럽게 줄여,
        # "정렬이 좋아질수록 더 빨리 내려간다"가 되도록 하강/정렬을 함께 묶는다.
        base_descent = 0.4 if current_drone_alt > 3.0 else 0.15
        align_factor = max(0.0, 1.0 - offset_dist / 80.0)  # 오프셋 80px 이상이면 하강 0
        vel_down = base_descent * align_factor

        if offset_dist > 40.0:
            print(f"[정밀착륙] 정렬 중... 오차 오프셋: {offset_dist:.1f} px (하강속도 {vel_down:.2f} m/s)")

        # 드론에게 속도 제어 명령 전달
        await drone.offboard.set_velocity_ned(VelocityNedYaw(vel_north, vel_east, vel_down, 0.0))

        data_received = False # 다음 주기 루프 확인을 위해 플래그 초기화
        await asyncio.sleep(0.05) # 20Hz 제어 루프

    # 하강 완료 후 착륙 명령
    try:
        await drone.offboard.stop()
    except:
        pass
    await drone.action.land()
    print("[정밀착륙] 미션 완료.")

# ── 메인 미션 시퀀스 ─────────────────────────────────────
async def main():
    drone = await connect()
    await wait_ready(drone)

    TAKEOFF_ALT = 10.0
    await arm_and_takeoff(drone, TAKEOFF_ALT)

    # 오프보드 정밀 착륙 테스트를 위해 원점 호버링 상태 진입
    hold_setpoint = PositionNedYaw(0.0, 0.0, -TAKEOFF_ALT, 0.0)
    await start_offboard_with_retry(drone, hold_setpoint)

    print("[미션] 5초간 제자리 호버링 후 정밀 착륙을 시작합니다.")
    await asyncio.sleep(5)

    # 연동 정밀 착륙 함수 호출!
    await aruco_precision_land(drone)

if __name__ == "__main__":
    asyncio.run(main())