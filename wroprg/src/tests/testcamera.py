# file: cam_save_native.py
import cv2
import os
from picamera2 import Picamera2
from libcamera import controls

# 1) Create output folder if not exists
output_dir = "output"
os.makedirs(output_dir, exist_ok=True)

# 2) Init camera
picam2 = Picamera2()

# Configure for native 1332x990 (binned) mode
video_config = picam2.create_video_configuration(
    main={"size": (1332, 990), "format": "RGB888"},
    buffer_count=3,
    controls={
        # Target 4 FPS => frame duration ~250,000 microseconds
        "FrameDurationLimits": (250000, 250000),
    },
)
picam2.configure(video_config)

# Optional autofocus / exposure control
try:
    # Use string key to avoid ControlId objects that some versions treat as strings
    picam2.set_controls({
        "AfMode": controls.AfModeEnum.Continuous,  # Ignore if fixed-focus
    })
except Exception as e:
    print(f"Warning: set_controls(AfMode) failed: {e}")

# Ensure 4 FPS if not set in config (some platforms require this after configure)
try:
    picam2.set_controls({"FrameDurationLimits": (250000, 250000)})
except Exception as e:
    print(f"Warning: could not set FrameDurationLimits: {e}")

# 3) Start streaming
picam2.start()

frame_count = 0
target_period = 0.25  # seconds per frame for 4 FPS
next_time = None
try:
    while True:
        frame_rgb = picam2.capture_array()         # Capture frame (RGB888)
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

        # Save frame to output folder
        filename = os.path.join(output_dir, f"frame_{frame_count:05d}.jpg")
        cv2.imwrite(filename, frame_bgr)
        print(f"Saved {filename}")
        frame_count += 1

        # Show preview (optional, can remove)
        # cv2.imshow("Camera", frame_bgr)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC key to stop
            break

        # Throttle to ~4 FPS regardless of processing speed
        if next_time is None:
            next_time = cv2.getTickCount() / cv2.getTickFrequency() + target_period
        else:
            now = cv2.getTickCount() / cv2.getTickFrequency()
            sleep_sec = next_time - now
            if sleep_sec > 0:
                # Use OpenCV waitKey timing granularity for portability
                cv2.waitKey(int(sleep_sec * 1000))
            next_time += target_period
finally:
    cv2.destroyAllWindows()
    picam2.stop()
