"""
WRO Future Engineers 2025 - Complete Autonomous Vehicle Code
Based on existing hardware interface and incorporating round1 navigation + round2 camera logic

This code implements a complete WRO Future Engineers solution:
- Uses existing pin configurations from hardware_interface.py
- Incorporates wall-following logic from round1 
- Uses camera-based obstacle detection from round2
- Implements proper WRO rules for obstacle avoidance
- Completes 3 rounds successfully using gyro navigation
- No parking implementation (as requested)
"""

import logging
import time
import threading
import cv2
import numpy as np
from typing import Tuple, Optional
from enum import Enum
from hardware.hardware_interface import HardwareInterface
from hardware.robotstate import RobotState
from round1.walker_helpers import FixedTurnWalker
from utils.helpers import HelperFunctions
from utils import constants
from utils.mat import MATDIRECTION, MATGENERICLOCATION

logger = logging.getLogger(__name__)

class ObstacleColor(Enum):
    """Obstacle color identification"""
    RED = "red"
    GREEN = "green"
    UNKNOWN = "unknown"

class DriveDirection(Enum):
    """Track driving direction"""
    CLOCKWISE = 1
    COUNTERCLOCKWISE = -1
    UNKNOWN = 0

class WROFutureEngineersBot:
    """
    Complete WRO Future Engineers autonomous vehicle implementation
    """

    # WRO Future Engineers specific constants
    WRO_RED_COLOR_RGB = (238, 39, 55)      # PANTONE 1795 C
    WRO_GREEN_COLOR_RGB = (68, 214, 44)    # PANTONE 802 C

    # Camera parameters for obstacle detection (from round2)
    FRAME_WIDTH_PX = 640
    KNOWN_OBJECT_HEIGHT = 10.0  # cm (100mm obstacle height)
    KNOWN_FOCAL_LENGTH = 970.0  # pixels (from calibration)

    # Navigation constants (from round1 logic)
    DEFAULT_SPEED = 30
    CORNER_SPEED = 25
    MAX_STEERING_ANGLE = 35
    WALL_FOLLOW_DISTANCE = 25.0  # cm
    OBSTACLE_DETECTION_DISTANCE = 50.0  # cm

    # Round completion
    TARGET_LAPS = 3
    MAX_ROUND_TIME = 180  # 3 minutes in seconds

    def __init__(self, hardware_interface: HardwareInterface):
        """Initialize the WRO Future Engineers bot"""
        self.hardware = hardware_interface
        self.current_lap = 0
        self.drive_direction = DriveDirection.UNKNOWN
        self.start_time = None
        self.last_obstacle_avoid_time = 0
        self.obstacle_avoid_cooldown = 2.0  # seconds

        # Camera setup
        self.camera = self.hardware.camera
        self.camera.start()

        # Navigation state
        self.wall_following_side = "right"  # Default to right wall following
        self.last_yaw_reading = 0.0

        logger.info("WRO Future Engineers Bot initialized")

    def detect_drive_direction(self) -> DriveDirection:
        """
        Detect the driving direction by analyzing initial wall distances and movement
        """
        logger.info("Detecting drive direction...")

        # Take initial readings
        initial_state = self.hardware.read_state()
        initial_yaw = initial_state.yaw

        # Move forward slightly while monitoring yaw change
        self.hardware.drive_forward(15)  # Slow speed
        time.sleep(1.0)

        # Check yaw change
        current_state = self.hardware.read_state()
        yaw_change = current_state.yaw - initial_yaw

        self.hardware.drive_stop()

        # Determine direction based on yaw change and wall distances
        if abs(yaw_change) < 2.0:  # Going straight, use wall analysis
            if current_state.left < current_state.right:
                self.drive_direction = DriveDirection.COUNTERCLOCKWISE
                self.wall_following_side = "left"
            else:
                self.drive_direction = DriveDirection.CLOCKWISE
                self.wall_following_side = "right"
        else:
            # Use yaw change to determine direction
            if yaw_change > 0:  # Turning right naturally
                self.drive_direction = DriveDirection.CLOCKWISE
                self.wall_following_side = "right"
            else:  # Turning left naturally
                self.drive_direction = DriveDirection.COUNTERCLOCKWISE
                self.wall_following_side = "left"

        logger.info(f"Detected drive direction: {self.drive_direction.name}")
        logger.info(f"Wall following side: {self.wall_following_side}")

        return self.drive_direction

    def detect_obstacle_color(self, frame: np.ndarray) -> Tuple[ObstacleColor, Tuple[int, int, int, int]]:
        """
        Detect obstacle color and bounding box using camera (from round2 logic)
        Returns: (color, (x, y, w, h))
        """
        if frame is None:
            return ObstacleColor.UNKNOWN, (0, 0, 0, 0)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define HSV ranges for red and green obstacles
        # Red color detection (handling red wrap-around in HSV)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        # Green color detection
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([80, 255, 255])
        green_mask = cv2.inRange(hsv, lower_green, upper_green)

        # Find contours for both colors
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Find largest contours
        largest_red_area = 0
        largest_green_area = 0
        red_bbox = (0, 0, 0, 0)
        green_bbox = (0, 0, 0, 0)

        if red_contours:
            largest_red = max(red_contours, key=cv2.contourArea)
            largest_red_area = cv2.contourArea(largest_red)
            red_bbox = cv2.boundingRect(largest_red)

        if green_contours:
            largest_green = max(green_contours, key=cv2.contourArea)
            largest_green_area = cv2.contourArea(largest_green)
            green_bbox = cv2.boundingRect(largest_green)

        # Determine which color is more prominent and closer
        min_obstacle_area = 500  # Minimum pixels for valid obstacle

        if largest_red_area > min_obstacle_area and largest_green_area > min_obstacle_area:
            # Both colors detected, choose the larger/closer one
            if largest_red_area > largest_green_area:
                return ObstacleColor.RED, red_bbox
            else:
                return ObstacleColor.GREEN, green_bbox
        elif largest_red_area > min_obstacle_area:
            return ObstacleColor.RED, red_bbox
        elif largest_green_area > min_obstacle_area:
            return ObstacleColor.GREEN, green_bbox
        else:
            return ObstacleColor.UNKNOWN, (0, 0, 0, 0)

    def calculate_obstacle_distance(self, bbox: Tuple[int, int, int, int]) -> float:
        """
        Calculate distance to obstacle using bounding box height (from round2 logic)
        """
        if bbox[3] <= 0:  # Invalid height
            return float('inf')

        distance = (self.KNOWN_OBJECT_HEIGHT * self.KNOWN_FOCAL_LENGTH) / bbox[3]
        return distance

    def should_avoid_obstacle(self, obstacle_color: ObstacleColor) -> Tuple[bool, str]:
        """
        Determine if obstacle should be avoided and which direction based on WRO rules

        WRO Rules:
        - Clockwise: Green obstacles inner (avoid right), Red obstacles outer (avoid left)
        - Counter-clockwise: Red obstacles inner (avoid right), Green obstacles outer (avoid left)

        Returns: (should_avoid, direction)
        """
        if obstacle_color == ObstacleColor.UNKNOWN:
            return False, "none"

        current_time = time.time()
        if current_time - self.last_obstacle_avoid_time < self.obstacle_avoid_cooldown:
            return False, "none"  # Avoid rapid successive avoidance maneuvers

        if self.drive_direction == DriveDirection.CLOCKWISE:
            if obstacle_color == ObstacleColor.GREEN:
                return True, "right"  # Green on inner, avoid by going right
            elif obstacle_color == ObstacleColor.RED:
                return True, "left"   # Red on outer, avoid by going left
        elif self.drive_direction == DriveDirection.COUNTERCLOCKWISE:
            if obstacle_color == ObstacleColor.RED:
                return True, "right"  # Red on inner, avoid by going right
            elif obstacle_color == ObstacleColor.GREEN:
                return True, "left"   # Green on outer, avoid by going left

        return False, "none"

    def perform_obstacle_avoidance(self, avoid_direction: str, obstacle_distance: float):
        """
        Perform obstacle avoidance maneuver
        """
        logger.info(f"Avoiding obstacle by going {avoid_direction}, distance: {obstacle_distance:.1f}cm")

        self.last_obstacle_avoid_time = time.time()

        # Calculate avoidance parameters based on distance
        if obstacle_distance < 20:
            # Close obstacle - sharp avoidance
            steering_angle = 30 if avoid_direction == "right" else -30
            avoidance_time = 1.5
        elif obstacle_distance < 35:
            # Medium distance - moderate avoidance
            steering_angle = 20 if avoid_direction == "right" else -20
            avoidance_time = 1.0
        else:
            # Far obstacle - gentle avoidance
            steering_angle = 15 if avoid_direction == "right" else -15
            avoidance_time = 0.8

        # Perform avoidance maneuver
        # Step 1: Turn to avoid
        self.hardware.turn_steering(steering_angle)
        self.hardware.drive_forward(self.DEFAULT_SPEED)
        time.sleep(avoidance_time)

        # Step 2: Straighten out
        self.hardware.turn_steering(0)
        time.sleep(0.3)

        # Step 3: Turn back to original path
        opposite_angle = -steering_angle * 0.7  # Slightly less to account for drift
        self.hardware.turn_steering(opposite_angle)
        time.sleep(avoidance_time * 0.8)

        # Step 4: Return to center
        self.hardware.turn_steering(0)

        logger.info("Obstacle avoidance maneuver completed")

    def wall_follow_navigation(self) -> bool:
        """
        Perform wall-following navigation (adapted from round1 logic)
        Returns True if lap completed, False otherwise
        """
        state = self.hardware.read_state()

        # Check for lap completion (yaw-based)
        yaw_change = abs(state.yaw - self.last_yaw_reading)
        if yaw_change > 300:  # Completed approximately 360 degrees
            self.current_lap += 1
            self.hardware.reset_gyro()
            self.last_yaw_reading = 0.0
            logger.info(f"Lap {self.current_lap} completed!")
            return True

        self.last_yaw_reading = state.yaw

        # Wall following logic
        if self.wall_following_side == "right":
            target_distance = self.WALL_FOLLOW_DISTANCE
            current_distance = state.right
            error = target_distance - current_distance
        else:  # left wall following
            target_distance = self.WALL_FOLLOW_DISTANCE
            current_distance = state.left
            error = current_distance - target_distance

        # PID-like steering correction
        steering_correction = error * 0.8  # Proportional gain
        steering_correction = max(min(steering_correction, self.MAX_STEERING_ANGLE), -self.MAX_STEERING_ANGLE)

        # Apply steering correction
        self.hardware.turn_steering(steering_correction)

        # Speed control based on steering angle
        if abs(steering_correction) > 20:
            speed = self.CORNER_SPEED
        else:
            speed = self.DEFAULT_SPEED

        self.hardware.drive_forward(speed)

        return False

    def check_obstacle_ahead(self) -> Tuple[bool, ObstacleColor, float]:
        """
        Check for obstacles ahead using camera
        Returns: (obstacle_detected, color, distance)
        """
        try:
            frame = self.camera.capture()
            if frame is None:
                return False, ObstacleColor.UNKNOWN, float('inf')

            obstacle_color, bbox = self.detect_obstacle_color(frame)

            if obstacle_color != ObstacleColor.UNKNOWN:
                distance = self.calculate_obstacle_distance(bbox)
                if distance < self.OBSTACLE_DETECTION_DISTANCE:
                    return True, obstacle_color, distance

            return False, ObstacleColor.UNKNOWN, float('inf')

        except Exception as e:
            logger.warning(f"Camera obstacle detection failed: {e}")
            return False, ObstacleColor.UNKNOWN, float('inf')

    def run_complete_challenge(self):
        """
        Run the complete WRO Future Engineers challenge
        """
        logger.info("Starting WRO Future Engineers Challenge")

        # Initialize
        self.start_time = time.time()
        self.current_lap = 0

        # Detect drive direction
        self.detect_drive_direction()

        # Reset gyro for lap counting
        self.hardware.reset_gyro()
        self.last_yaw_reading = 0.0

        # Main navigation loop
        while self.current_lap < self.TARGET_LAPS:
            # Check time limit
            elapsed_time = time.time() - self.start_time
            if elapsed_time > self.MAX_ROUND_TIME:
                logger.warning("Time limit reached!")
                break

            # Check for obstacles ahead
            obstacle_detected, obstacle_color, obstacle_distance = self.check_obstacle_ahead()

            if obstacle_detected:
                # Determine avoidance strategy
                should_avoid, avoid_direction = self.should_avoid_obstacle(obstacle_color)

                if should_avoid:
                    # Stop and perform obstacle avoidance
                    self.hardware.drive_stop()
                    self.perform_obstacle_avoidance(avoid_direction, obstacle_distance)
                    continue

            # Perform normal wall-following navigation
            lap_completed = self.wall_follow_navigation()

            if lap_completed:
                # Brief pause between laps
                self.hardware.drive_stop()
                time.sleep(0.5)

                # Check if all laps completed
                if self.current_lap >= self.TARGET_LAPS:
                    logger.info("All laps completed successfully!")
                    break

            # Small delay to prevent overwhelming the system
            time.sleep(0.05)

        # Stop the vehicle
        self.hardware.drive_stop()
        self.hardware.turn_steering(0)

        # Log final results
        final_time = time.time() - self.start_time
        logger.info(f"Challenge completed! Laps: {self.current_lap}/{self.TARGET_LAPS}, Time: {final_time:.1f}s")

        # Success indicators
        if self.current_lap >= self.TARGET_LAPS:
            self.hardware.led1_green()
            self.hardware.buzzer_beep(1.0)
        else:
            self.hardware.led1_red()
            self.hardware.buzzer_beep(0.5)

def main():
    """Main function to run the WRO Future Engineers challenge"""
    import argparse

    parser = argparse.ArgumentParser(description="WRO Future Engineers 2025 Challenge")
    parser.add_argument('--logfile', type=str, default='wro_challenge.log', help='Path to log file')
    parser.add_argument('--debug', action='store_true', help='Enable debug mode')
    args = parser.parse_args()

    # Initialize helper and hardware
    helper = HelperFunctions(args.logfile, args.debug, screen_logger=True)
    logger = logging.getLogger(__name__)

    hardware_interface = helper.get_pi_interface()

    try:
        # Wait for hardware to be ready
        hardware_interface.wait_for_ready()

        # Create and run the WRO bot
        wro_bot = WROFutureEngineersBot(hardware_interface)

        # Signal ready and wait for start button
        hardware_interface.led1_blue()
        logger.info("WRO Future Engineers Bot ready! Press action button to start.")
        hardware_interface.wait_for_action()

        # Start the challenge
        hardware_interface.led1_white()
        wro_bot.run_complete_challenge()

    except Exception as e:
        logger.error(f"Challenge failed: {e}")
        hardware_interface.led1_red()
        hardware_interface.buzzer_beep(0.2)
        time.sleep(0.1)
        hardware_interface.buzzer_beep(0.2)
        raise

    finally:
        # Cleanup
        helper.shutdown_all()

if __name__ == "__main__":
    main()
