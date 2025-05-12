from spike import PrimeHub
import sys
import time

# Initialize the hub
hub = PrimeHub()

# Function to read data from USB
def read_usb_data():
    try:
        # Read a line of input from the USB port
        data = sys.stdin.readline().strip()
        return data
    except Exception as e:
        return f"Error: {e}"

# Main loop
while True:
    # Read data from USB
    usb_data = read_usb_data()
    
    # Display the data on the hub's screen
    hub.light_matrix.write(usb_data)
    
    # Wait for a short time before reading again
    time.sleep(0.5) 