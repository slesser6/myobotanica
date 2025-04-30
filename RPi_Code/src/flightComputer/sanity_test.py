import asyncio
from mavsdk import System

async def main():
    drone = System()
    await drone.connect(system_address="udp://:14540")   # ok for both A & B
    print("Waiting for position …")
    async for pos in drone.telemetry.position():
        print(pos)          # should print within ~3 s
        break

asyncio.run(main())
