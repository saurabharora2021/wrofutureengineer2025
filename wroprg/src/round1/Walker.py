from rpi.RpiInterface import RpiInterface
from time import sleep
from hat.BuildHatDriveBase import BuildHatDriveBase
import logging
import math

class Walker:

    logger = logging.getLogger(__name__)


    CLOCKWISE_DIRECTION=1
    DEFAULT_SPEED=100
    ANTI_CLOCKWISE_DIRECTION=2
    MINFRONTDISTANCE=100
    WALLFRONTDISTANCE=30
    WALLSIDEDISTANCE=15
    UNKNOWN_DIRECTION=-1
    TURNRIGHT_ANGLE=10
    TURNLEFT_ANGLE=-10
    WALLFRONTENDDISTANCE=10
    D_TARGET = 10  # Desired distance from the wall
    KP = 2.0  # Proportional gain for the controller
    MAX_ANGLE = 10
    
    drivebase:BuildHatDriveBase
    outputInterface: RpiInterface
    direction=UNKNOWN_DIRECTION    
    corner=False

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
    def start_walk(self,nooflaps:int=4):
        self.logger.info("Starting to walk...")
        self.logger.warning("Direction:%s", self.directiontostr(self.direction))

        cornerCounter = 0
        # Implement the logic for starting a walk
        if self.direction == self.UNKNOWN_DIRECTION:
            self.logger.info("Front Distance:%s",self.drivebase.getFrontDistance())

            #Revisit if we need to run this loop or start checking color immediately.
            while self.drivebase.getFrontDistance() > self.MINFRONTDISTANCE or self.drivebase.getFrontDistance() < 0:
                #can we think of equi wall follow or use gyro to walk straight?.
                self.drivebase.runfront(self.DEFAULT_SPEED)
                self.logger.info("Front Distance:%s",self.drivebase.getFrontDistance())
                sleep(0.1)
            
            color = self.wait_for_color(["blue", "orange"])

            
            if color == "blue":
                self.direction = self.ANTI_CLOCKWISE_DIRECTION
            elif color == "orange":
                self.direction = self.CLOCKWISE_DIRECTION
                
            self.corner = True
            cornerCounter= 1
            
            self.outputInterface.buzzer_beep()
            self.logger.warning("color: %s", color)
            self.logger.warning("Direction: %s", self.directiontostr(self.direction))
            self.drivebase.stop()
            return

        totalcorners = nooflaps * 4  # Each turn is 90 degrees, so 4 turns make a full circle

        # We will walk until we reach the end of the section or complete the required number of
        while cornerCounter < totalcorners:
            if self.corner:
                if self.direction == self.CLOCKWISE_DIRECTION:
                    self.handle_corner(self.TURNRIGHT_ANGLE, self.outputInterface.getRightDistance)
                else:
                    self.handle_corner(self.TURNLEFT_ANGLE, self.outputInterface.getLeftDistance)
                cornerCounter += 1
                if cornerCounter >= totalcorners:
                    self.logger.info("Reached the end of the walk.")
                    self.drivebase.back_motor.stop()
                    return
            else:
                self.follow_wall_until(self.WALLFRONTENDDISTANCE)
                     

    def directiontostr(self,direction):
        """Convert direction to string."""
        if direction == self.CLOCKWISE_DIRECTION:
            return "Clockwise"
        elif direction == self.ANTI_CLOCKWISE_DIRECTION:
            return "Anti-clockwise"
        else:
            return "Unknown"
        
    def handle_corner(self, turn_angle, side_distance_func):
        self.drivebase.turnsteering(turn_angle)
        self.drivebase.runfront(self.DEFAULT_SPEED)
        while self.drivebase.getFrontDistance() > self.WALLFRONTDISTANCE or side_distance_func() > self.WALLSIDEDISTANCE:
            sleep(0.1)
        self.drivebase.turnsteering(-turn_angle)
        self.corner = False
    
    def follow_wall_until(self, distance):
        while self.drivebase.getFrontDistance() > distance:
            self.wallFollow()
            sleep(0.1)
        self.corner = True
    
    def wait_for_color(self, colors):
        r, g, b, _ = self.drivebase.getBottomColorRGBI()
        color = self.mat_color(r, g, b)
        while color not in colors:
            self.drivebase.runfront(self.DEFAULT_SPEED)
            sleep(0.1)
            r, g, b, _ = self.drivebase.getBottomColorRGBI()
            color = self.mat_color(r, g, b)
        return color


            



