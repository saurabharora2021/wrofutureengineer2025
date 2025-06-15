from rpi.LoggerSetup import LoggerSetup 
from rpi.RpiInterface import RpiInterface
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
    WALLFRONTDISTANCE=15
    WALLSIDEDISTANCE=5
    UNKNOWN_DIRECTION=-1
    TURNRIGHT_ANGLE=10
    TURNLEFT_ANGLE=10
    WALLFRONTENDDISTANCE=10
    
    drivebase:BuildHatDriveBase
    outputInterface: RpiInterface
    direction=UNKNOWN_DIRECTION    
    corner=False

    
    def equidistance_walk(self):
        pass

    def wallFollow(self):
        """Follow the wall based on the current direction."""
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
                self.corner= True
            elif color == "orange":
                self.direction = self.CLOCKWISE_DIRECTION
                self.corner = True
            
            print(f"Direction set to: {self.direction} based on color: {color}")
        elif self.direction == self.CLOCKWISE_DIRECTION:
            if self.corner==True:
                #turn right
                self.drivebase.turnright(self.TURNRIGHT_ANGLE)
                self.drivebase.runfront(self.DEFAULT_SPEED)
                #either front less than 15
                #or side distance less than 5
                #only applicable if close to wall
                while self.drivebase.getFrontDistance() > self.WALLFRONTDISTANCE or self.outputInterface.getRightDistance() > self.WALLSIDEDISTANCE:
                    sleep(0.1)
                self.drivebase.turnright(-1*self.TURNRIGHT_ANGLE)
                self.corner = False
            else:
                while self.drivebase.getFrontDistance() > self.WALLFRONTENDDISTANCE :
                    self.wallFollow()
                    sleep(0.1)
                self.corner = True

        elif self.direction == self.ANTI_CLOCKWISE_DIRECTION:
            if self.corner==True:
                #turn left
                self.drivebase.turnleft(self.TURNLEFT_ANGLE)
                self.drivebase.runfront(self.DEFAULT_SPEED)
                #either front less than 15
                #or side distance less than 5
                #only applicable if close to wall
                while self.drivebase.getFrontDistance() > self.WALLFRONTDISTANCE or self.outputInterface.getLeftDistance() > self.WALLSIDEDISTANCE:
                    sleep(0.1)
                self.drivebase.turnleft(-1*self.TURNLEFT_ANGLE)
                self.corner = False
            else:
                while self.drivebase.getFrontDistance() > self.WALLFRONTENDDISTANCE :
                    self.wallFollow()
                    sleep(0.1)
                self.corner = True



            



