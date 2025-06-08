from abc import ABC, abstractmethod


class ExtendedCapability(ABC):

    """LEGO Spike Color Codes"""
    BLACK = 0
    MAGENTA = 1
    PURPLE = 2
    BLUE = 3
    AZURE = 4
    TURQUOISE = 5
    GREEN = 6
    YELLOW = 7
    ORANGE = 8
    RED = 9
    WHITE = 10
    UNKNOWN = -1
 
    @abstractmethod
    def getFrontDistance(self):
        """Get the distance to the front obstacle."""
        pass

    @abstractmethod
    def getBottomColor(self):
        """Get the color detected by the bottom sensor."""
        pass

    @abstractmethod
    def beep(self, frequency, duration,volume):
        """Make a beep sound."""
        pass

    @abstractmethod
    def setPowerColor(self, color):
        """Set the power color of the hub."""
        pass

    @abstractmethod
    def write_text(self, text):
        """Write text to the light matrix."""
        pass
