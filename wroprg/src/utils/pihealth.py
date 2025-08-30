""" Utililies function to prevent throattling , can be called after 1 round. to check."""
import time

class PiHealth:
    """Class to monitor and manage Raspberry Pi health."""
    def __init__(self, duration_seconds, threshold=65):
        self.duration_seconds = duration_seconds
        self.threshold = threshold

    def is_temperature_high(self):
        """
        Checks if the current CPU temperature exceeds the threshold.
        """
        with open("/sys/class/thermal/thermal_zone0/temp", "r", encoding="utf-8") as f:
            temp = int(f.readline()) / 1000.0

        print("Current CPU Temperature: %.2f°C" % temp)
        return temp > self.threshold

    def monitor_temperature(self):
        """
        Monitors CPU temperature for the duration specified in the instance.
        """
        end_time = time.time() + self.duration_seconds

        while time.time() < end_time:

            if self.is_temperature_high():
                print("High CPU temperature detected!")
                time.sleep(1)  # check every second


#main method to test
if __name__ == "__main__":
    pi_health = PiHealth(duration_seconds=5, threshold=65)
    pi_health.monitor_temperature()
