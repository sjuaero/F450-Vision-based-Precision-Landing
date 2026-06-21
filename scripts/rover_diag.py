import asyncio, math
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw

async def main():
    rover = System()
    await rover.connect(system_address="udpin://0.0.0.0:14540")
    async for state in rover.core.connection_state():
        if state.is_connected:
            print("연결 완료")
            break

    async for health in rover.telemetry.health():
        if health.is_local_position_ok:
            break

    # 현재 상태 출력
    async for armed in rover.telemetry.armed():
        print("Armed:", armed)
        break
    async for mode in rover.telemetry.flight_mode():
        print("현재 모드:", mode)
        break
    async for pos in rover.telemetry.position_velocity_ned():
        print("위치: N=%.2f E=%.2f" % (pos.position.north_m, pos.position.east_m))
        break

    # shell로 arm
    print("\n--- shell arm 시도 ---")
    await rover.shell.send("param set COM_RC_IN_MODE 4\n")
    await asyncio.sleep(0.5)
    await rover.shell.send("commander arm -f\n")
    await asyncio.sleep(1.5)
    async for armed in rover.telemetry.armed():
        print("Armed 후:", armed)
        break

    # offboard 시작
    print("\n--- offboard 시도 ---")
    async for pos in rover.telemetry.position_velocity_ned():
        cur_n = pos.position.north_m
        cur_e = pos.position.east_m
        break

    # 우선 PositionNedYaw 시도
    await rover.offboard.set_position_ned(PositionNedYaw(cur_n, cur_e - 2.0, 0.0, 0.0))
    try:
        await rover.offboard.start()
        print("offboard.start() 성공")
    except OffboardError as e:
        print("offboard.start() 실패:", e)
        return

    await asyncio.sleep(1)
    async for mode in rover.telemetry.flight_mode():
        print("offboard 후 모드:", mode)
        break

    # 3초 동안 위치 모니터링
    print("3초 동안 위치 모니터링...")
    t0 = asyncio.get_event_loop().time()
    async for pos in rover.telemetry.position_velocity_ned():
        print("  N=%.2f E=%.2f vN=%.2f vE=%.2f" % (
            pos.position.north_m, pos.position.east_m,
            pos.velocity.north_m_s, pos.velocity.east_m_s))
        if asyncio.get_event_loop().time() - t0 > 3:
            break

    print("\n--- VelocityNedYaw 시도 (전진 속도) ---")
    # 앞으로 = east -1 m/s
    await rover.offboard.set_velocity_ned(VelocityNedYaw(0.0, -1.0, 0.0, 0.0))
    await asyncio.sleep(0.5)
    async for mode in rover.telemetry.flight_mode():
        print("velocity 후 모드:", mode)
        break

    await asyncio.sleep(3)
    async for pos in rover.telemetry.position_velocity_ned():
        print("최종 위치: N=%.2f E=%.2f" % (pos.position.north_m, pos.position.east_m))
        break

    await rover.offboard.stop()
    await rover.action.hold()

asyncio.run(main())
