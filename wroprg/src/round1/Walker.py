from rpi.RpiInterface import RpiInterface
from base.ShutdownInterface import ShutdownInterface
from time import sleep
from hat.BuildHatDriveBase import BuildHatDriveBase
import logging
import math

class Walker(ShutdownInterface):

    logger = logging.getLogger(__name__)


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
    D_TARGET = 10  # Desired distance from the wall
    KP = 2.0  # Proportional gain for the controller
    MAX_ANGLE = 10
    
    drivebase:BuildHatDriveBase
    outputInterface: RpiInterface
    direction=UNKNOWN_DIRECTION    
    corner=False
    cornerCounter=0

    def __init__(self, drivebase:BuildHatDriveBase, outputInterface:RpiInterface):
        self.drivebase = drivebase
        self.outputInterface = outputInterface

    
    def equidistance_walk(self):
        pass

    def clamp(self,val, min_val, max_val):
        return max(min(val, max_val), min_val)  
    
    
    def wallFollow(self):
        """Follow the wall based on the current direction."""
        dist = 0
        if self.direction == self.CLOCKWISE_DIRECTION:   
            dist = self.outputInterface.getLeftDistance()
        elif self.direction == self.ANTI_CLOCKWISE_DIRECTION:
            dist = self.outputInterface.getRightDistance()
        error = self.D_TARGET - dist
        angle = self.clamp(self.KP * error, -1(self.MAX_ANGLE), self.MAX_ANGLE)

        #TODO: used turnleft or turnright based on direction
        # steering.set_angle(angle)
        self.drivebase.runfront(self.DEFAULT_SPEED)

        self.logger.warning(f"Distance: {dist:.2f}, angle: {angle:.2f}")
        sleep(0.1)
    
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
    def start_walk(self,noofturns=4):
        self.logger.info("Starting to walk...")
        self.logger.warning("Direction:%s", self.directiontostr(self.direction))
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
                self.cornerCounter+= 1
            elif color == "orange":
                self.direction = self.CLOCKWISE_DIRECTION
                self.corner = True
                self.cornerCounter+= 1
            
            self.logger.warning("color: %s", color)
            self.logger.warning("Direction: %s", self.directiontostr(self.direction))
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
                if self.cornerCounter/4 >= noofturns:
                    self.logger.info("Reached the end of the walk.")
                    self.drivebase.back_motor.stop()
                    #TODO: walk to middle of the section
                    return
            else:
                while self.drivebase.getFrontDistance() > self.WALLFRONTENDDISTANCE :
                    self.wallFollow()
                    sleep(0.1)
                self.corner = True
                self.cornerCounter+= 1

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
                if self.cornerCounter/4 >= noofturns:
                    self.logger.info("Reached the end of the walk.")
                    self.drivebase.back_motor.stop()
                    #TODO: walk to middle of the section
                    return
            else:
                while self.drivebase.getFrontDistance() > self.WALLFRONTENDDISTANCE :
                    self.wallFollow()
                    sleep(0.1)
                self.corner = True
                self.cornerCounter+= 1

    def shutdown(self):
        """Method to perform shutdown operations."""
        self.logger.info("Shutting down Walker...")
        self.drivebase.stop()
        self.logger.info("Walker shutdown complete.")

    def directiontostr(self,direction):
        """Convert direction to string."""
        if direction == self.CLOCKWISE_DIRECTION:
            return "Clockwise"
        elif direction == self.ANTI_CLOCKWISE_DIRECTION:
            return "Anti-clockwise"
        else:
            return "Unknown"


            



