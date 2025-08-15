"""Screen logger to write on the screen"""

import logging
from PIL import Image, ImageDraw, ImageFont
from hardware.pin_config import PinConfig
from hardware.rpi_interface import RpiInterface


logger = logging.getLogger(__name__)

class ScreenLogger:
    """Class to log messages to the screen."""
    def __init__(self,rpi:RpiInterface):
        self.image: Image.Image = Image.new("1", (PinConfig.SCREEN_WIDTH, PinConfig.SCREEN_HEIGHT))
        self.draw: ImageDraw.ImageDraw = ImageDraw.Draw(self.image)
        self.font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
                                        RpiInterface.FONT_SIZE)
        self.rpi = rpi

    def log_message(self, front: float, left: float, right: float, current_yaw: float,
                                                            current_steering: float):
        """Log the message to the screen."""
        logger.info("Front: %.2f, Left: %.2f, Right: %.2f, Yaw: %.2f, Steering: %.2f",
                                             front, left, right, current_yaw, current_steering)
        self.draw.rectangle((0, 0, PinConfig.SCREEN_WIDTH, PinConfig.SCREEN_HEIGHT),
                            outline=0, fill=0)

        # Draw left sensor value
        self.draw.text((5, 5), f"Left: {left:.2f}", font=self.font, fill=1, align="center")

        # Draw right sensor value
        self.draw.text((5, 15), f"Right: {right:.2f}", font=self.font, fill=1, align="center")

        # Draw front sensor value
        self.draw.text((5, 25), f"Front: {front:.2f}", font=self.font, fill=1, align="center")

        # Draw yaw value
        self.draw.text((5, 35), f"Yaw: {current_yaw:.2f}", font=self.font, fill=1, align="center")

        # Draw steering value
        self.draw.text((5, 45), f"Steering: {current_steering:.2f}", font=self.font,
                                                         fill=1, align="center")

        self.rpi.paint_display(self.image)
