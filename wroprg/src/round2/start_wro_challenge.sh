#!/bin/bash
# WRO Future Engineers 2025 Startup Script

echo "=== WRO Future Engineers 2025 Challenge ==="
echo "Starting autonomous vehicle system..."

# Check if running on Raspberry Pi
if ! grep -q "Raspberry Pi" /proc/cpuinfo; then
    echo "Warning: Not running on Raspberry Pi"
fi

# Check Python dependencies
echo "Checking dependencies..."
python3 -c "import cv2, numpy, picamera2, buildhat, gpiozero" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "Error: Missing required Python packages"
    echo "Please install: opencv-python numpy picamera2 buildhat gpiozero"
    exit 1
fi

# Set environment variables
export PYTHONPATH="${PYTHONPATH}:/home/pi/wrofutureengineer2025/wroprg/src"

# Navigate to source directory
cd /home/pi/wrofutureengineer2025/wroprg/src

# Run the WRO challenge
echo "Launching WRO Future Engineers challenge..."
python3 wro_future_engineers_main.py --logfile wro_challenge_$(date +%Y%m%d_%H%M%S).log

echo "Challenge completed. Check logs for details."
