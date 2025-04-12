from pybricks.pupdevices import Motor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.hubs import PrimeHub
from pybricks.parameters import Color
from pybricksdev.usb import PybricksHubUSB

# Standard MicroPython modules
from usys import stdin, stdout
from uselect import poll

hub = PrimeHub();
hub.light.on(Color.GREEN)

# This is useful for USB communication.
usb = USB()
# This is the USB port for communication.
usb.start()
usb.set_baudrate(115200)
usb.set_timeout(1)
usb.set_buffer_size(64)
usb._connect()
data = usb._read_usb()


# Optional: Register stdin for polling. This allows
# you to wait for incoming data without blocking.
keyboard = poll()
keyboard.register(stdin)


while True:
    # Read a command from USB
    command = stdin.read(3)

    if command == "fwd":
        hub.light.on("green")
        print("Moving forward")
    elif command == "rev":
        hub.light.on("red")
        print("Moving backward")
    elif command == "bye":
        hub.light.on("off")
        print("Stopping")
        break

    wait(100)