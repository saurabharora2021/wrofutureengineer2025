from pybricksdev.connections.pybricks import PybricksHubUSB
from pybricksdev.connections.pybricks import PybricksHub

# Initialize the USB connection
usb = PybricksHubUSB(PybricksHub())

# Connect to the hub
usb.connect()

# Send a command to the hub
usb.write_line(b"Hello Hub!")

# Read a response from the hub
response = usb.read_line()
print(f"Response from hub: {response}")

# Disconnect the USB connection
usb.disconnect()