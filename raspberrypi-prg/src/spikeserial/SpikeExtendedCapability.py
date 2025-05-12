from base import ExtendedCapability
from spikeserial import SpikeDriveBase


class SpikeExtendedCapability(ExtendedCapability):
    def __init__(self, drive_base: SpikeDriveBase):
        self.drive_base = drive_base

    def getFrontDistance(self) -> int:
        # Implement the method to get the front distance using the drive base
        self.drive_base.messagesend("FrontDistance\n")
        message = self.drive_base.messagereceive()
        return int(message)

    def getBottomColor(self):
        # Implement the method to get the bottom color using the drive base
        return self.drive_base.getBottomColor()
