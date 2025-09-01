"""Utility functions to center the bot """
import logging
from typing import Tuple
from utils import constants
from utils.mat import MATDIRECTION
from round1.matintelligence import MatIntelligence

logger = logging.getLogger(__name__)

class BotPositioner:
    """Utility class to center the bot"""

    DISTANCE_CORRECTION = 4.0
    YAW_CORRECTION = 0.75

    def __init__(self, intelligence:MatIntelligence):
        self.intelmat = intelligence



    def center_bot_correction(self,front:float, left:float,
                                right:float,prev_yaw:float) -> Tuple[bool,float,float, float]:
        """Correct the left and right distances to center the bot.
            first argument: bool is corrected value.
            second argument: float is the yaw correction.
            third argument: float is the left distance.
            fourth argument: float is the right distance.
        """
        # If Delta is high move towards the center, move by 10cm otherwise too high correction.

        direction = self.intelmat.get_direction()

        if left == constants.LEFT_DISTANCE_MAX and right < constants.RIGHT_DISTANCE_MAX:
            if direction == MATDIRECTION.CLOCKWISE_DIRECTION:
                #make sure your are away from the inside wall you can read.
                left = max(25,left)
                return (True, 0, left, right)
            else:
                #Anti clockwise,
                # make sure you are 30 away from outside wall
                right = max(30,right)
                return (True, 0, left, right)

        if right == constants.RIGHT_DISTANCE_MAX and left < constants.LEFT_DISTANCE_MAX:
            if direction == MATDIRECTION.CLOCKWISE_DIRECTION:
                # make sure you are 30 away from outside wall
                right = max(30,right)
                return (True, 0, left, right)
            else:
                # Anti clockwise,
                # make sure you are away from the inside wall you can read.
                left = max(25,left)
                return (True, 0, left, right)

        delta_yaw = 0
        # If we are very close to either wall turn.
        if right <= 20 or left <= 20:
            correction = self.DISTANCE_CORRECTION #10 if front > 130 else 5
            if left < right:
                logger.info("Adjusting left distance, moving to right")
                left += correction
                right -= correction
                delta_yaw = -self.YAW_CORRECTION
            else:
                logger.info("Adjusting right distance,moving to left")
                left -= correction
                right += correction
                delta_yaw = self.YAW_CORRECTION
            logger.info("adjusted left %.2f , right %.2f, yaw %.2f",left,right,delta_yaw)
            return True,delta_yaw,left, right
        else:
            logger.info("Delta distance is low, not changing distances.")
            return False,prev_yaw,left,right

    def side_bot_centering(self,front:float,learned_left:float,learned_right:float,
                                actual_left:float,actual_right:float,
                                prev_yaw:float,lenient:bool=False)-> Tuple[bool,float,float,float]:
        """ Side bot Centering for handle side function."""
        total_learned = learned_left + learned_right
        total_actual = actual_left + actual_right
        logger.info("Side Bot Centering: Learned L: %.2f, R: %.2f, Actual L: %.2f, R: %.2f",
                    learned_left, learned_right, actual_left, actual_right)
        #if the gap is less , or the learned is still not correct so less than 40.
        #ignore and center the bot
        if lenient:
            if total_learned > 80 and total_learned <105:
                #we can be lenient
                if actual_left >= 30 and actual_right >= 30:
                    return (False, prev_yaw, actual_left, actual_right)
            elif total_learned > 50 and total_learned < 70:
                if actual_left >= 20 and actual_right >= 20:
                    return (False, prev_yaw, actual_left, actual_right)

        if abs(total_learned - total_actual) < 2 or total_learned < 40:
            #forget the difference, center actual an return
            return self.center_bot_correction(front, actual_left, actual_right,prev_yaw)
        else:
            # we need to proportionally reduce actual
            if total_actual<total_learned:
                return self.center_bot_correction(front, actual_left, actual_right,prev_yaw)
            else:

                logger.info("Proportional reduction of actual distances")
                # total actual > total_learned
                # we need to proportionally reduce actual

                if actual_left <= constants.LEFT_DISTANCE_MAX or \
                        actual_right == constants.RIGHT_DISTANCE_MAX:
                    # If we are at the max distance, we need to be careful
                    logger.info("At max distance, adjusting...")
                    if actual_left == constants.LEFT_DISTANCE_MAX and \
                                actual_right != constants.RIGHT_DISTANCE_MAX:
                        #we can adjust the right distance
                        if actual_right > total_learned/2:
                            actual_right -= 5
                            prev_yaw = -self.YAW_CORRECTION
                    elif actual_right == constants.RIGHT_DISTANCE_MAX and \
                                        actual_left != constants.LEFT_DISTANCE_MAX:
                        #we can adjust the left distance
                        if actual_left > total_learned/2:
                            actual_left -= 5
                            prev_yaw = +self.YAW_CORRECTION
                    logger.info("Adjusted distances: L: %.2f, R: %.2f Y: %.2f",
                                actual_left, actual_right, prev_yaw)
                    return (True, prev_yaw, actual_left, actual_right)

                diff = min(total_actual - total_learned,10)
                if actual_left> actual_right:
                    actual_left -= diff
                else:
                    actual_right -= diff

                if actual_left > 10 and actual_right > 10:
                    return self.center_bot_correction(front, actual_left, actual_right,prev_yaw)
                else:
                    logger.warning("Actual distances are too low after correction,"
                                   + " returning without correction.")
                    return (False,prev_yaw,actual_left, actual_right)
