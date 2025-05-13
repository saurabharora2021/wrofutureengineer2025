from pybricks.hubs import PrimeHub
import sys

# Initialize the hub
hub = PrimeHub()

# Indicate that the hub is ready
hub.light.on("green")
print("Hub is ready to receive data.")

while True:
    # Read data from USB (stdin)
    data = sys.stdin.read(11)  # Adjust the number of bytes to match the expected input
    if data:
        print(f"Received: {data}")  # Send a response back via USB (stdout)

        # Perform an action based on the received data
        if data == "Hello Hub!":
            hub.light.on("blue")  # Change the light color to blue
            print("Acknowledged: Hello Hub!")
        elif data == "bye":
            hub.light.off()  # Turn off the light
            print("Goodbye!")
            break

