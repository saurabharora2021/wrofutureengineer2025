import spremote
from base.DriveBase import DriveBase
import serial
import time
import logging
from base.ShutdownInterface import ShutdownInterface


class SpikeRemoteBase(DriveBase,ShutdownInterface):

    logger = logging.getLogger(__name__)

    """
    Base class for Spike Remote.
    """

    def __init__(self, front_motor_port, back_motor_port,bottom_color_sensor_port, front_distance_sensor_port,debug=False):
        """
        Initialize the Spike Remote with two motors and two color sensors.
        """
        # Initialize the Spike Remote connection
        self.hub = spremote.Hub('/dev/ttyACM0')
        self.beep(400,1000,100)
        time.sleep(1)
        self.hub.list_devices()
        self.write_text("Remote")
        self.logger.debug("Spike Remote Hub initialized.")

        """ None intialized variables """
        self.front_motor = None
        self.back_motor = None
        self.bottom_color_sensor = None
        self.front_distance_sensor = None
        

        if (not debug):
            # Initialize the motors and sensors
            self.front_motor = spremote.Motor(self.hub, front_motor_port)
            self.back_motor = spremote.Motor(self.hub, back_motor_port)
            self.bottom_color_sensor = spremote.ColorSensor(self.hub, bottom_color_sensor_port)
            self.front_distance_sensor = spremote.DistanceSensor(self.hub, front_distance_sensor_port)
        
            # Initialize the motors
            self.front_motor.set_power(0)
            self.back_motor.set_power(0)
            self.front_motor.set_speed(0)
            self.back_motor.set_speed(0)
            self.front_motor.set_position(0)
            self.back_motor.set_position(0)
            self.front_motor.set_mode('position')
            self.back_motor.set_mode('position')
            self.front_motor.set_stop_action('brake')
            self.back_motor.set_stop_action('brake')


    def runfront(self, speedpercent):
        """ Back Motor is used to run the drive base forward """
        return self.back_motor.start(speedpercent)
    
    def stop(self):
        self.back_motor.stop()
        self.front_motor.stop()

    def turnright(self, angle):
        self.front_motor.run_degrees(angle, 50)

    def turnleft(self, angle):
        self.front_motor.run_degrees(-angle, 50)
    
    def get_front_distance(self):
        """Get the distance from the front distance sensor."""
        return self.front_distance_sensor.get_distance()
    
    def get_bottom_color(self):
        """Get the color from the bottom color sensor."""
        return self.bottom_color_sensor.get_color()
    
    def write_text(self, text):
        """Write text to the light matrix."""
        ret = self.hub.cmd(f'hub.light_matrix.write("{text}")')
        self.logger.debug(f'hub.light_matrix.write in LightMatrix.write_text returned {ret}')

    def beep(self, frequency=440, duration=500,volume=100):
        """Make a beep sound."""
        ret = self.hub.cmd(f'hub.speaker.beep({frequency}, {duration},{volume})')
        self.logger.debug(f'hub.speaker.beep in LightMatrix.beep returned {ret}')  
    
    def shutdown(self):
        """Shutdown the Spike Remote."""
        self.front_motor.stop()
        self.back_motor.stop()
        self.hub.disconnect()
        self.logger.info("Spike Remote shutdown complete.")
