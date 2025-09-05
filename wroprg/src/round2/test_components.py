#!/usr/bin/env python3
"""
WRO Future Engineers 2025 - Component Test Script
Use this script to test individual components of the system
"""

import argparse
import logging
import time
from utils.helpers import HelperFunctions

def test_hardware_basic(hardware_interface):
    """Test basic hardware functionality"""
    print("=== Testing Basic Hardware ===")

    # Test LEDs
    print("Testing LEDs...")
    hardware_interface.led1_red()
    time.sleep(0.5)
    hardware_interface.led1_green()
    time.sleep(0.5)
    hardware_interface.led1_blue()
    time.sleep(0.5)
    hardware_interface.led1_off()

    # Test buzzer
    print("Testing buzzer...")
    hardware_interface.buzzer_beep(0.3)

    # Test button
    print("Press action button to continue...")
    hardware_interface.wait_for_action()
    print("Button test passed!")

def test_sensors(hardware_interface):
    """Test all sensors"""
    print("=== Testing Sensors ===")

    for i in range(10):
        state = hardware_interface.read_state()
        print(f"Reading {i+1}:")
        print(f"  Front: {state.front:.1f}cm")
        print(f"  Left:  {state.left:.1f}cm") 
        print(f"  Right: {state.right:.1f}cm")
        print(f"  Yaw:   {state.yaw:.1f}°")
        print(f"  Camera Front: {state.camera_front:.1f}cm")
        time.sleep(0.5)

def test_motors(hardware_interface):
    """Test motor movements"""
    print("=== Testing Motors ===")

    # Test drive motor
    print("Testing drive motor...")
    hardware_interface.drive_forward(20)
    time.sleep(1.0)
    hardware_interface.drive_backward(20)
    time.sleep(1.0)
    hardware_interface.drive_stop()

    # Test steering motor
    print("Testing steering motor...")
    hardware_interface.turn_steering(30)
    time.sleep(1.0)
    hardware_interface.turn_steering(-30)
    time.sleep(1.0)
    hardware_interface.turn_steering(0)

    print("Motor tests completed")

def test_camera(hardware_interface):
    """Test camera and color detection"""
    print("=== Testing Camera ===")

    try:
        # Import WRO bot for color detection
        import sys
        sys.path.append('.')
        from wro_future_engineers_main import WROFutureEngineersBot

        wro_bot = WROFutureEngineersBot(hardware_interface)

        print("Capturing frames and detecting colors...")
        for i in range(10):
            frame = hardware_interface.camera.capture()
            if frame is not None:
                color, bbox = wro_bot.detect_obstacle_color(frame)
                if color.value != "unknown":
                    distance = wro_bot.calculate_obstacle_distance(bbox)
                    print(f"Frame {i+1}: Detected {color.value} obstacle at {distance:.1f}cm")
                else:
                    print(f"Frame {i+1}: No obstacles detected")
            time.sleep(0.5)

    except Exception as e:
        print(f"Camera test failed: {e}")

def test_gyro_calibration(hardware_interface):
    """Test gyro calibration and readings"""
    print("=== Testing Gyro Calibration ===")

    print("Place robot on level surface...")
    time.sleep(2)

    # Reset gyro
    hardware_interface.reset_gyro()
    print("Gyro reset to zero")

    # Take readings
    print("Taking yaw readings for 10 seconds...")
    start_time = time.time()
    while time.time() - start_time < 10:
        yaw = hardware_interface.get_yaw()
        print(f"Current yaw: {yaw:.2f}°")
        time.sleep(0.5)

def main():
    parser = argparse.ArgumentParser(description="WRO Future Engineers Component Tester")
    parser.add_argument('--test', choices=['basic', 'sensors', 'motors', 'camera', 'gyro', 'all'],
                       default='all', help='Which component to test')
    parser.add_argument('--logfile', type=str, default='component_test.log', help='Log file path')

    args = parser.parse_args()

    # Initialize hardware
    print("Initializing hardware...")
    helper = HelperFunctions(args.logfile, debug=True, screen_logger=True)
    hardware_interface = helper.get_pi_interface()
    hardware_interface.wait_for_ready()

    try:
        if args.test == 'basic' or args.test == 'all':
            test_hardware_basic(hardware_interface)

        if args.test == 'sensors' or args.test == 'all':
            test_sensors(hardware_interface)

        if args.test == 'motors' or args.test == 'all':
            test_motors(hardware_interface)

        if args.test == 'camera' or args.test == 'all':
            test_camera(hardware_interface)

        if args.test == 'gyro' or args.test == 'all':
            test_gyro_calibration(hardware_interface)

        print("\n=== All Tests Completed ===")
        hardware_interface.led1_green()
        hardware_interface.buzzer_beep(1.0)

    except Exception as e:
        print(f"Test failed: {e}")
        hardware_interface.led1_red()
        hardware_interface.buzzer_beep(0.3)

    finally:
        helper.shutdown_all()

if __name__ == "__main__":
    main()
