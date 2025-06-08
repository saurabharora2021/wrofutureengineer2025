from rpi.LoggerSetup import LoggerSetup 
from rpi.OutputInterface import OutputInterface
from rpi.ShutdownInterfaceManager import ShutdownInterfaceManager
from time import sleep
from hat.BuildHatDriveBase import BuildHatDriveBase
import logging
import math

class Walker:
    

    CLOCKWISE_DIRECTION=1
    DEFAULT_SPEED=100
    ANTI_CLOCKWISE_DIRECTION=2
    MINFRONTDISTANCE=20
    UNKNOWN_DIRECTION=-1
    drivebase:BuildHatDriveBase
    direction=UNKNOWN_DIRECTION

    
    def equidistance_walk(self):
        pass

    #based on color.py from buildhat
    def mat_color(self, r, g, b):
        """Return the color name from RGB

        :param r: Red
        :param g: Green
        :param b: Blue
        :return: Name of the color as a string
        :rtype: str
        """
        table = [("black", (0, 0, 0)),
                 ("orange", (255, 102, 0)),
                 ("blue", (0, 51, 255)),
                 ("white", (255, 255, 255)),
                 ("line", (179, 179, 179))
                 ]
        near = ""
        euc = math.inf
        for itm in table:
            cur = math.sqrt((r - itm[1][0])**2 + (g - itm[1][1])**2 + (b - itm[1][2])**2)
            if cur < euc:
                near = itm[0]
                euc = cur
        return near
    def start_walk(self):
        print("Starting to walk...")
        # Implement the logic for starting a walk
        if self.direction == self.UNKNOWN_DIRECTION:
            while self.drivebase.getFrontDistance() > self.MINFRONTDISTANCE or self.drivebase.getFrontDistance() == -1:
                self.drivebase.runfront(self.DEFAULT_SPEED)
                sleep(0.1)
            r,g,b,_ = self.drivebase.getBottomColorRGBI()

            color = self.mat_color(r, g, b)
            while color != "blue" and color != "orange":
                self.drivebase.runfront(self.DEFAULT_SPEED)
                sleep(0.1)
                r, g, b, _ = self.drivebase.getBottomColorRGBI()
                color = self.mat_color(r, g, b)
            
            if color == "blue":
                self.direction = self.ANTI_CLOCKWISE_DIRECTION
            elif color == "orange":
                self.direction = self.CLOCKWISE_DIRECTION
            
            print(f"Direction set to: {self.direction} based on color: {color}")


            


