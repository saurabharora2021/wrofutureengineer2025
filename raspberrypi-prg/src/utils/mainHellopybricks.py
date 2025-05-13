from pybricksdev.usb import PybricksHubUSB

# Initialize the USB connection
usb = PybricksHubUSB()

# Connect to the hub
usb.connect()

# Send a command to the hub
usb.write(b"Hello Hub!")

# Read a response from the hub
response = usb.read()
print(f"Response from hub: {response}")

# Disconnect the USB connection
usb.disconnect()