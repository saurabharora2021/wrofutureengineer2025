"""Screen logger to write on the screen"""

import logging
from typing import List
from PIL import Image, ImageDraw, ImageFont
from hardware.pin_config import PinConfig


logger = logging.getLogger(__name__)

class ScreenLogger:
    """Class to log messages to the screen."""
    FONT_SIZE = 10 # font size for messages
    LINE_HEIGHT = 10  # pixels per line
    MAX_MESSAGES = 2  # maximum number of messages to display

    def __init__(self):
        self.image: Image.Image = Image.new("1", (PinConfig.SCREEN_WIDTH, PinConfig.SCREEN_HEIGHT))
        self.draw: ImageDraw.ImageDraw = ImageDraw.Draw(self.image)
        self.font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
                                        self.FONT_SIZE)
        self.messages: List[str] = []  # Initialize messages list

    def log_message(self, front: float, left: float, right: float, current_yaw: float,
                                                            current_steering: float)->Image.Image:
        """Log the message to the screen."""
        logger.info("Front: %.2f, Left: %.2f, Right: %.2f, Yaw: %.2f, Steering: %.2f",
                                             front, left, right, current_yaw, current_steering)
        self.draw.rectangle((0, 0, PinConfig.SCREEN_WIDTH, PinConfig.SCREEN_HEIGHT),
                            outline=0, fill=0)

        cursor = 5
        # Draw left sensor value
        self.draw.text((5, cursor), f"L: {left:.2f}  R: {right:.2f}", font=self.font, fill=1,
                        align="center")

        cursor += self.LINE_HEIGHT

        # Draw front sensor value
        self.draw.text((5, cursor), f"Front: {front:.2f}", font=self.font, fill=1,
                        align="center")

        cursor += self.LINE_HEIGHT

        # Draw yaw value
        self.draw.text((5, cursor), f"Y: {current_yaw:.2f}   S: {current_steering:.2f}",
                       font=self.font, fill=1, align="center")
        cursor += self.LINE_HEIGHT

        for msg in enumerate(self.messages):
            # Use constant for line height spacing
            self.draw.text((0, cursor), msg, font=self.font, fill=255)
            cursor += self.LINE_HEIGHT

        return self.image

    def add_message(self, messages: List[str]) -> None:
        """Add messages to the screen logger."""
        self.messages.extend(messages)
        if len(self.messages) > self.MAX_MESSAGES:
            self.messages = self.messages[-self.MAX_MESSAGES:]  # Keep only the last MAX_MESSAGES
