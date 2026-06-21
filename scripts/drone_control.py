import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw


TAKEOFF_ALT = 2.5   # meters above ground
MOVE_DIST   = 1.0   # meters per step
HOLD_SEC    = 3.0   # seconds to hold each position


async def connect(address="udp://:14541"):
    drone = System()
    await drone.connect(system_address=address)
    print(f"Connecting to {address} ...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected.")
            break
    return drone


async def wait_ready(drone):
    print("Waiting for EKF2 ...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_local_position_ok:
            print("EKF2 ready.")
            break


async def takeoff(drone, alt=TAKEOFF_ALT):
    print("Arming ...")
    await drone.action.arm()
    await drone.action.set_takeoff_altitude(alt)
    print(f"Taking off to {alt} m ...")
    await drone.action.takeoff()
    await asyncio.sleep(5)
    print("Airborne.")


async def goto_ned(drone, north, east, down, yaw=0.0, hold=HOLD_SEC, label=""):
    if label:
        print(f"  -> {label}  N={north:+.1f}  E={east:+.1f}  D={down:+.1f}")
    await drone.offboard.set_position_ned(PositionNedYaw(north, east, down, yaw))
    await asyncio.sleep(hold)


async def run_mission(drone):
    alt = -TAKEOFF_ALT   # NED: negative = up

    # start offboard from current position (hover)
    await drone.offboard.set_position_ned(PositionNedYaw(0, 0, alt, 0))
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"Offboard failed: {e}")
        await drone.action.land()
        return

    await asyncio.sleep(2)

    d = MOVE_DIST
    waypoints = [
        ( d,  0, alt, 0, "Forward"),
        ( 0,  0, alt, 0, "Center"),
        (-d,  0, alt, 0, "Backward"),
        ( 0,  0, alt, 0, "Center"),
        ( 0,  d, alt, 0, "Right"),
        ( 0,  0, alt, 0, "Center"),
        ( 0, -d, alt, 0, "Left"),
        ( 0,  0, alt, 0, "Center"),
    ]

    for n, e, dz, yaw, label in waypoints:
        await goto_ned(drone, n, e, dz, yaw, label=label)

    print("Mission complete. Landing ...")
    await drone.offboard.stop()
    await drone.action.land()
    print("Done.")


async def main():
    drone = await connect()
    await wait_ready(drone)
    await takeoff(drone)
    await run_mission(drone)


asyncio.run(main())
