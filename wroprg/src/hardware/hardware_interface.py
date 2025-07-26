"""Unified Hardware Interface for WRO Future Engineer 2025 project."""

import logging
from typing import Optional
from base.shutdown_handling import ShutdownInterface
from hardware.hardwareconfig import HardwareConfig
from hardware.legodriver import BuildHatDriveBase
from hardware.rpi_interface import RpiInterface

logger = logging.getLogger(__name__)

class HardwareInterface(ShutdownInterface):
    """
    Provides unified access to all hardware components.
    Includes methods for both LEGO driver and Raspberry Pi interface.
    """

    def __init__(self,stabilize:bool) -> None:
        self._rpi = RpiInterface(stabilize)
        self._lego_drive_base: Optional[BuildHatDriveBase] = None

    def full_initialization(self) -> None:
        """Initialize all hardware components."""
        try:
            if HardwareConfig.CHASSIS_VERSION == 1:
                self._lego_drive_base = BuildHatDriveBase(front_motor_port='D', back_motor_port='A',
                                               bottom_color_sensor_port='C',
                                               front_distance_sensor_port='B')
            elif HardwareConfig.CHASSIS_VERSION == 2:
                self._lego_drive_base = BuildHatDriveBase(front_motor_port='D', back_motor_port='A',
                                               bottom_color_sensor_port='C',
                                               front_distance_sensor_port=None)
        except Exception as e:
            logger.error("Failed to initialize drive base: %s", e)
            raise RuntimeError(f"Drive base initialization failed: {e}") from e


    # --- Raspberry Pi Interface Methods ---
    def buzzer_beep(self, timer: float = 0.5) -> None:
        """Turn on the buzzer."""
        self._rpi.buzzer_beep(timer)

    def led1_green(self) -> None:
        """Turn on the LED1 green."""
        self._rpi.led1_green()

    def led1_red(self) -> None:
        """Turn on the LED1 red."""
        self._rpi.led1_red()

    def led1_blue(self) -> None:
        """Turn on the LED1 blue."""
        self._rpi.led1_blue()

    def led1_white(self) -> None:
        """Turn on the LED1 white."""
        self._rpi.led1_white()

    def led1_off(self) -> None:
        """Turn off the LED1."""
        self._rpi.led1_off()

    def wait_for_action(self) -> None:
        """Wait for the action button to be pressed."""
        self._rpi.wait_for_action()

    def get_right_distance(self) -> float:
        """Get the distance from the right distance sensor."""
        return self._rpi.get_right_distance()

    def get_right_distance_max(self) -> float:
        """Get the maximum distance from the right distance sensor."""
        return self._rpi.get_right_distance_max()

    def get_left_distance(self) -> float:
        """Get the distance from the left distance sensor."""
        return self._rpi.get_left_distance()

    def get_left_distance_max(self) -> float:
        """Get the maximum distance from the left distance sensor."""
        return self._rpi.get_left_distance_max()

    def display_message(self, message: str, forceflush: bool = False) -> None:
        """
        Display a message on the OLED screen.

        Only the last 5 messages are shown on the display.
        """
        self._rpi.display_message(message, forceflush)

    def force_flush_messages(self) -> None:
        """Force flush the messages on the OLED screen."""
        self._rpi.force_flush_messages()

    def get_jumper_state(self) -> bool:
        """Get the state of the jumper pin."""        
        return self._rpi.get_jumper_state()

    def shutdown(self) -> None:
        """Shutdown the hardware interface."""
        if self._lego_drive_base is None:
            logger.warning("LEGO Drive Base not initialized, skipping shutdown.")
        else:
            self._lego_drive_base.shutdown()

        self._rpi.shutdown()

    # --- LEGO Driver Methods ---
    def drive_forward(self, speed: float) -> None:
        """Run the drive base forward at the specified speed."""
        if self._lego_drive_base is None:
            raise RuntimeError("LEGO Drive Base not initialized. Call full_initialization() first.")
        self._lego_drive_base.run_front(speed)

    def turn_steering(self, degrees: float,steering_speed:float=10) -> None:
        """
        Turn the steering by the specified degrees.
        Positive degrees turn right, negative turn left.
        Limits steering to +/-38 degrees.
        """
        if self._lego_drive_base is None:
            raise RuntimeError("LEGO Drive Base not initialized. Call full_initialization() first.")
        self._lego_drive_base.turn_steering(degrees, steering_speed)

    def drive_stop(self) -> None:
        """Stop the drive base."""
        if self._lego_drive_base is None:
            raise RuntimeError("LEGO Drive Base not initialized. Call full_initialization() first.")
        self._lego_drive_base.stop()

    def get_bottom_color(self) -> str:
        """Get the color detected by the bottom sensor."""
        if self._lego_drive_base is None:
            raise RuntimeError("LEGO Drive Base not initialized. Call full_initialization() first.")
        return self._lego_drive_base.get_bottom_color()

    def get_bottom_color_rgbi(self) -> list[float]:
        """Get the RGB values detected by the bottom sensor."""
        if self._lego_drive_base is None:
            raise RuntimeError("LEGO Drive Base not initialized. Call full_initialization() first.")
        return self._lego_drive_base.get_bottom_color_rgbi()

    def get_front_distance(self) -> float:
        """Get the distance to the front obstacle in centimeter."""
        if HardwareConfig.CHASSIS_VERSION == 1:
            if self._lego_drive_base is None:
                raise RuntimeError("LEGO Drive Base not initialized. Call full_initialization()" \
                " first.")
            return self._lego_drive_base.get_front_distance()
        elif HardwareConfig.CHASSIS_VERSION == 2:
            return self._rpi.get_front_distance()

    ## End of LEGO Driver Methods
