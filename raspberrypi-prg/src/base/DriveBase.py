from abc import ABC, abstractmethod


class DriveBase(ABC):
    @abstractmethod
    def runfront(self, speed):
        """Run the drive base forward at the specified speed."""
        pass

    @abstractmethod
    def stop(self):
        """Stop the drive base."""
        pass

    @abstractmethod
    def turnright(self, angle):
        """Turn the drive base to the right by the specified angle."""
        pass

    @abstractmethod
    def turnleft(self, angle):
        """Turn the drive base to the left by the specified angle."""
        pass
