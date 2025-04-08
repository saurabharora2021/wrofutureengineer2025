from bluetooth import discover_devices, BluetoothError

class BluetoothScanner:
    def __init__(self):
        pass

    def discover_devices(self):
        try:
            devices = discover_devices(lookup_names=True)
            return devices
        except BluetoothError as e:
            print(f"Bluetooth error: {e}")
            return []

if __name__ == "__main__":
    scanner = BluetoothScanner()
    nearby_devices = scanner.discover_devices()
    
    if nearby_devices:
        print("Found Bluetooth devices:")
        for addr, name in nearby_devices:
            print(f"{name} - {addr}")
    else:
        print("No Bluetooth devices found.")