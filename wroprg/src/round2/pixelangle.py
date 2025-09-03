from picamera2 import Picamera2
import cv2
import numpy as np
import os

FRAME_WIDTH_PX = 640

# Known actual parameters
KNOWN_OBJECT_HEIGHT = 10.0  # cm (100 mm)
KNOWN_FOCAL_LENGTH = 970.0  # pixels (from calibration)

os.makedirs("outputs", exist_ok=True)

picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (FRAME_WIDTH_PX, int(FRAME_WIDTH_PX * 3 / 4))})
picam2.configure(config)
picam2.start()

num_rows = 10
num_cols = 10

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
        # Get largest green block
        largest = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest)

        block_mask = mask[y:y+h, x:x+w]
        block_green_pixel_count = cv2.countNonZero(block_mask)
        print(f"Frame {frame_count}: Green pixel count in block = {block_green_pixel_count}")

        # Calculate distance
        distance = (KNOWN_OBJECT_HEIGHT * KNOWN_FOCAL_LENGTH) / h
        print(f"Frame {frame_count}: Estimated distance to object = {distance:.2f} cm")

        H, W = frame.shape[:2]

        # Initialize grid counts for green pixels
        grid_counts = np.zeros((num_rows, num_cols), dtype=int)

        # Calculate sizes of each grid cell
        cell_height = H // num_rows
        cell_width = W // num_cols

        # Calculate green pixel counts per quadrant in full frame mask
        for i in range(num_rows):
            for j in range(num_cols):
                y_start = i * cell_height
                y_end = (i + 1) * cell_height if i < num_rows - 1 else H
                x_start = j * cell_width
                x_end = (j + 1) * cell_width if j < num_cols - 1 else W

                # Extract mask slice
                quadrant_mask = mask[y_start:y_end, x_start:x_end]
                green_count = cv2.countNonZero(quadrant_mask)
                grid_counts[i, j] = green_count

        # Find the quadrant with maximum green pixels
        max_idx = np.unravel_index(np.argmax(grid_counts), grid_counts.shape)
        max_row, max_col = max_idx
        print(f"Frame {frame_count}: Quadrant with max green pixels is row {max_row+1}, column {max_col+1}")

        # Draw the grid rectangles and highlight max quadrant
        for i in range(num_rows):
            for j in range(num_cols):
                p1 = (j * cell_width, i * cell_height)
                p2 = ((j + 1) * cell_width, (i + 1) * cell_height)
                color = (0, 255, 0) if (i, j) == max_idx else (255, 0, 0)
                thickness = 3 if (i, j) == max_idx else 1
                cv2.rectangle(frame, p1, p2, color, thickness)

        # Draw rectangle around detected block
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 3)

        # Save frame with grids and block
        cv2.imwrite(f"outputs/frame_{frame_count:05d}.jpg", frame)
    else:
        print(f"Frame {frame_count}: No green block detected.")
        cv2.imwrite(f"outputs/frame_{frame_count:05d}.jpg", frame)

picam2.stop()
