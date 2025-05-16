import time
from datetime import datetime

log_file = "timelog.txt"

with open(log_file, "a") as f:
    while True:
        now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        f.write(f"{now}\n")
        f.flush()  # Ensure data is written to disk immediately
        print(f"Logged: {now}")
        time.sleep(60)