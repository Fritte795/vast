from bus import VASTBus
import asyncio

vast = VASTBus()


async def start():
    print("VAST DB App started")
    while True:
        await asyncio.sleep(1)
