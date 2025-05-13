from pybricksdev.connections.pybricks import PybricksHubUSB
from usb.core import find as find_usb
import asyncio
from pybricksdev.usb import (
                LEGO_USB_VID,
                MINDSTORMS_INVENTOR_USB_PID,
                SPIKE_ESSENTIAL_USB_PID,
                SPIKE_PRIME_USB_PID,
            )

import usb.backend.libusb1
backend = usb.backend.libusb1.get_backend()
if backend is None:
    print("No libusb backend found!")
else:
    print("libusb backend is available.")
    

def is_pybricks_usb(dev):
                return (
                    (dev.idVendor == LEGO_USB_VID)
                    and (
                        dev.idProduct
                        in [
                            SPIKE_PRIME_USB_PID,
                            SPIKE_ESSENTIAL_USB_PID,
                            MINDSTORMS_INVENTOR_USB_PID,
                        ]
                    )
                    and dev.product.endswith("Pybricks")
                )

#Find the USB device
device_or_address = find_usb(custom_match=is_pybricks_usb)
if device_or_address is None:
    raise RuntimeError("No Pybricks USB device found")

# Initialize the USB connection
usb = PybricksHubUSB(PybricksHub(device_or_address))


def hello_world(loop: asyncio.AbstractEventLoop):

    # Connect to the hub
    loop.run_until_complete(usb.connect())

    # Send a command to the hub
    loop.run_until_complete(usb.write_line(b"Hello Hub!"))

    # Read a response from the hub
    response = usb.read_line()
    print(f"Response from hub: {response}")

    # Disconnect the USB connection
    usb.disconnect()




if __name__ == "__main__":
    
    loop = asyncio.get_event_loop()
    hello_world(loop)
    loop.close()