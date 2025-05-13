from pybricksdev.connections.pybricks import PybricksHubUSB
from pybricksdev.connections.pybricks import PybricksHub

# Initialize the USB connection
usb = PybricksHubUSB(PybricksHub())


async def hello_world():
    
    # Connect to the hub
    usb.connect()

    # Send a command to the hub
    usb.write_line(b"Hello Hub!")

    # Read a response from the hub
    response = usb.read_line()
    print(f"Response from hub: {response}")

    # Disconnect the USB connection
    usb.disconnect()

if __name__ == "__main__":
    import asyncio
    loop = asyncio.get_event_loop()
    loop.run_until_complete(hello_world())
    loop.close()