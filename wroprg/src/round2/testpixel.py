from picamera2 import Picamera2
import cv2
import numpy as np
import os

FRAME_WIDTH_PX = 640

# Known parameters for distance calculation
KNOWN_OBJECT_HEIGHT = 10.0  # cm (100 mm)
KNOWN_FOCAL_LENGTH = 2075.0 # pixels (from calibration)

os.makedirs("outputs", exist_ok=True)

picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (FRAME_WIDTH_PX, int(FRAME_WIDTH_PX * 3 / 4))})
picam2.configure(config)
picam2.start()

for frame_count in range(10):
    frame = picam2.capture_array()
    if frame is None:
        print("Error: Could not read frame.")
        break

    frame = cv2.flip(frame, 1)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_green = np.array([40, 40, 40])
    upper_green = np.array([80, 255, 255])
    mask = cv2.inRange(hsv, lower_green, upper_green)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest)
        block_mask = mask[y:y+h, x:x+w]
        block_green_pixel_count = cv2.countNonZero(block_mask)
        print(f"Frame {frame_count}: Green pixel count in block = {block_green_pixel_count}")

        # Calculate distance: D = (H_real * f) / h
        distance = (KNOWN_OBJECT_HEIGHT * KNOWN_FOCAL_LENGTH) / h
        print(f"Frame {frame_count}: Estimated distance to object = {distance:.2f} cm")

        # Crop block region but from y to bottom of frame, full width
        H, W = frame.shape[:2]
        y1 = y
        y2 = H
        x1 = 0
        x2 = W
        block_img = frame[y1:y2, x1:x2]

        # Draw rectangle around detected block on cropped image
        rect_x1 = x - x1
        rect_y1 = 0
        rect_x2 = rect_x1 + w
        rect_y2 = h
        cv2.rectangle(block_img, (rect_x1, rect_y1), (rect_x2, rect_y2), (0, 255, 0), 3)

        cv2.imwrite(f"outputs/block_{frame_count:05d}.jpg", block_img)
    else:
        print(f"Frame {frame_count}: No green block detected.")
        cv2.imwrite(f"outputs/block_{frame_count:05d}.jpg", frame)

picam2.stop()
