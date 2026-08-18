import asyncio
from bleak import BleakScanner

async def main():
    devices = await BleakScanner.discover(
        timeout=10.0,
        return_adv=True,
    )

    print(f"Found {len(devices)} devices")

    for address, (device, adv) in devices.items():
        print()
        print("device.name:", device.name)
        print("address/id:", address)
        print("local_name:", adv.local_name)
        print("service UUIDs:", adv.service_uuids)
        print("RSSI:", adv.rssi)

asyncio.run(main())