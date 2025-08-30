"""Screen logger to write on the screen"""

import logging
from typing import List
from PIL import Image, ImageDraw, ImageFont


logger = logging.getLogger(__name__)

class ScreenLogger:
    """Class to log messages to the screen."""
    FONT_SIZE = 10 # font size for messages
    LINE_HEIGHT = 10  # pixels per line
    MAX_MESSAGES = 3  # maximum number of messages to display
    START_X = 11

    def __init__(self,width:int,height:int) -> None:
        
        self.width = width
        self.height = height
        self.image: Image.Image = Image.new("1", (self.width, self.height))
        self.draw: ImageDraw.ImageDraw = ImageDraw.Draw(self.image)
        self.font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
                                        self.FONT_SIZE)
        self.messages: List[str] = []  # Initialize messages list


    def log_message(self, front: float, left: float, right: float, current_yaw: float,
                                                            current_steering: float)->Image.Image:
        """Log the message to the screen."""
        logger.debug("Front: %.2f, Left: %.2f, Right: %.2f, Yaw: %.2f, Steering: %.2f",
                                             front, left, right, current_yaw, current_steering)
        self.draw.rectangle((0, 0, self.width, self.height),
                            outline=0, fill=0)

        if current_steering > 0:
            #Steering at right side
            self.draw.rectangle((self.width - 9, 0, self.width,
                                 self.height), outline=0, fill=1)
        else:
            #steering towards left side
            self.draw.rectangle((0, 0, 9, self.height),outline=0, fill=1)

        cursor = 0
        # Draw left sensor value
        self.draw.text((self.START_X, cursor), f"L: {left:.2f}  R: {right:.2f}",
                       font=self.font, fill=1, align="center")

        cursor += self.LINE_HEIGHT

        # Draw front sensor value
        self.draw.text((self.START_X, cursor), f"Front: {front:.2f}", font=self.font, fill=1,
                        align="center")

        cursor += self.LINE_HEIGHT

        # Draw yaw value
        self.draw.text((self.START_X, cursor), f"Y: {current_yaw:.2f}   S: {current_steering:.2f}",
                       font=self.font, fill=1, align="center")
        cursor += self.LINE_HEIGHT

        # if len(self.messages) == 0:
        #     self.messages.append("No messages to display")
        # else:
        #     logger.info("Messages to display: %s", self.messages)

        for _, msg in enumerate(self.messages):
            # Use constant for line height spacing
            self.draw.text((self.START_X, cursor), msg, font=self.font, fill=1, align="center")
            cursor += self.LINE_HEIGHT

        return self.image

    def add_message(self, messages: List[str]) -> None:
        """Add messages to the screen logger."""
        # logger.info("Screen logger messages: %s", messages)
        self.messages.extend(messages)
        if len(self.messages) > self.MAX_MESSAGES:
            self.messages = self.messages[-self.MAX_MESSAGES:]  # Keep only the last MAX_MESSAGES
