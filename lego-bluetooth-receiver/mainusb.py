import serial
import time

# Replace with the correct port for your LEGO Spike Prime hub
# On Windows, it might look like "COM3", "COM4", etc.
# On macOS/Linux, it might look like "/dev/tty.usbmodemXXXX" or "/dev/ttyACM0"
USB_PORT = "COM3"
BAUD_RATE = 115200  # Default baud rate for LEGO Spike Prime

def main():
    try:
        # Open the serial connection
        with serial.Serial(USB_PORT, BAUD_RATE, timeout=1) as ser:
            print(f"Connected to {USB_PORT} at {BAUD_RATE} baud.")

            # Wait for the hub to initialize
            time.sleep(2)

            # Send a command to the hub
            ser.write(b"fwd\n")  # Example command to move forward
            print("Sent: fwd")

            # Wait for a response
            response = ser.readline().decode("utf-8").strip()
            print(f"Received: {response}")

            # Send another command
            ser.write(b"rev\n")  # Example command to move backward
            print("Sent: rev")

            # Wait for a response
            response = ser.readline().decode("utf-8").strip()
            print(f"Received: {response}")

            # Send a stop command
            ser.write(b"bye\n")
            print("Sent: bye")

    except serial.SerialException as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        print("Program interrupted by user.")

if __name__ == "__main__":
    main()