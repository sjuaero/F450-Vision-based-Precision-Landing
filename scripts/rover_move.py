import asyncio, math
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

ROVER_RADIUS = 0.5

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

    print("현재 위치: N=%.2f E=%.2f" % (cur_n, cur_e))

    target_n = cur_n + 0.0
    target_e = cur_e - 2.0
    print("목표 위치: N=%.2f E=%.2f" % (target_n, target_e))

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

    await rover.offboard.set_position_ned(PositionNedYaw(target_n, target_e, 0.0, 0.0))
    try:
        await rover.offboard.start()
        print("이동 시작")
    except OffboardError as e:
        print("Offboard 실패: %s" % e)
        return

    async for pos in rover.telemetry.position_velocity_ned():
        dn = pos.position.north_m - target_n
        de = pos.position.east_m  - target_e
        dist = math.hypot(dn, de)
        print("  남은 거리: %.2fm" % dist, end="\r")
        if dist < ROVER_RADIUS:
            break

    print("\n도착!")
    await rover.offboard.stop()
    await rover.action.hold()

asyncio.run(main())
