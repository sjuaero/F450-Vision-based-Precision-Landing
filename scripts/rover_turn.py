import asyncio, math
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

TURN_DEGREES = 90   # 왼쪽으로 몇 도 선회할지
ARC_RADIUS   = 2.0  # 선회 반경 (m)
ARRIVAL_DIST = 0.5  # 도착 판정 거리 (m)

async def main():
    rover = System()
    await rover.connect(system_address="udpin://0.0.0.0:14540")
    print("연결 대기...")
    async for state in rover.core.connection_state():
        if state.is_connected:
            print("연결 완료")
            break

    async for health in rover.telemetry.health():
        if health.is_local_position_ok:
            print("EKF2 준비")
            break

    async for pos in rover.telemetry.position_velocity_ned():
        cur_n = pos.position.north_m
        cur_e = pos.position.east_m
        break

    async for att in rover.telemetry.attitude_euler():
        cur_yaw_deg = att.yaw_deg
        break

    print("현재 위치: N=%.2f E=%.2f" % (cur_n, cur_e))
    print("현재 heading: %.1f deg" % cur_yaw_deg)

    # 표준 NED: forward=(cos,sin), left=(sin,-cos)
    # 90도 좌회전 호 끝점 = forward+left → 45도 전방-좌측 (PX4가 전진 방향으로 인식)
    yaw_rad = math.radians(cur_yaw_deg)

    target_n = cur_n + ARC_RADIUS * (math.cos(yaw_rad) + math.sin(yaw_rad))
    target_e = cur_e + ARC_RADIUS * (math.sin(yaw_rad) - math.cos(yaw_rad))
    target_yaw_deg = cur_yaw_deg - TURN_DEGREES

    print("목표 위치: N=%.2f E=%.2f  heading: %.1f deg" % (target_n, target_e, target_yaw_deg))

    await rover.param.set_param_int("BAT1_SOURCE", 0)
    await rover.param.set_param_int("COM_RC_IN_MODE", 4)
    await rover.param.set_param_int("CBRK_SUPPLY_CHK", 894281)
    await asyncio.sleep(0.5)

    async for armed in rover.telemetry.armed():
        already_armed = armed
        break

    if not already_armed:
        print("ARM 시도 (shell commander arm -f)...")
        await rover.shell.send("commander arm -f\n")
        await asyncio.sleep(2.0)
        async for armed in rover.telemetry.armed():
            already_armed = armed
            break
        if not already_armed:
            print("ARM 실패 - PX4 콘솔에서 수동 입력 필요:")
            print("  param set BAT1_SOURCE 0")
            print("  param set CBRK_SUPPLY_CHK 894281")
            print("  param set COM_RC_IN_MODE 4")
            print("  commander arm -f")
            return
        print("ARM 완료")
    else:
        print("이미 ARM 상태")
    await asyncio.sleep(1)

    await rover.offboard.set_position_ned(
        PositionNedYaw(target_n, target_e, 0.0, target_yaw_deg)
    )
    try:
        await rover.offboard.start()
        print("선회 시작")
    except OffboardError as e:
        print("Offboard 실패: %s" % e)
        return

    # offboard 모드 진입 확인
    await asyncio.sleep(1)
    async for mode in rover.telemetry.flight_mode():
        print("현재 모드: %s" % mode)
        break

    async for pos in rover.telemetry.position_velocity_ned():
        dn = pos.position.north_m - target_n
        de = pos.position.east_m  - target_e
        dist = math.hypot(dn, de)
        print("  남은 거리: %.2fm" % dist, end="\r")
        if dist < ARRIVAL_DIST:
            break

    print("\n선회 완료!")
    await rover.offboard.stop()
    await rover.action.hold()

asyncio.run(main())
