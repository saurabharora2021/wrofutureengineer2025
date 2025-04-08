import asyncio
from bleak import BleakScanner

class BluetoothScanner:
    def __init__(self):
        pass

    async def discover_devices(self):
        try:
            devices = await BleakScanner.discover()
            return devices
        except Exception as e:
            print(f"Bluetooth error: {e}")
            return []

if __name__ == "__main__":
    scanner = BluetoothScanner()

    async def main():
        print("Scanning for Bluetooth devices...")
        nearby_devices = await scanner.discover_devices()
        
        if nearby_devices:
            print("Found Bluetooth devices:")
            for device in nearby_devices:
                print(f"{device.name} - {device.address}")
        else:
            print("No Bluetooth devices found.")

    asyncio.run(main())