"""Test for logic round 1"""
from typing import Tuple
import logging

from utils.mat import MATDIRECTION

logger = logging.getLogger(__name__)
class LogicRound1Test:
    """Test for logic round 1"""

    _left_max = 200
    _right_max = 200
    direction = MATDIRECTION.ANTICLOCKWISE_DIRECTION

    def center_bot_correction(self,front:float, left:float,
                                right:float,prev_yaw:float) -> Tuple[bool,float,float, float]:
        """Correct the left and right distances to center the bot.
            first argument: bool is corrected value.
            second argument: float is the yaw correction.
            third argument: float is the left distance.
            fourth argument: float is the right distance.
        """
        # If Delta is high move towards the center, move by 10cm otherwise too high correction.

        if left == self._left_max and right < self._right_max:
            if self.direction == MATDIRECTION.CLOCKWISE_DIRECTION:
                #make sure your are away from the inside wall you can read.
                left = max(25,left)
                return (True, 0, left, right)
            else:
                #Anti clockwise,
                # make sure you are 30 away from outside wall
                right = max(30,right)
                return (True, 0, left, right)

        if right == self._right_max and left < self._left_max:
            if self.direction == MATDIRECTION.CLOCKWISE_DIRECTION:
                # make sure you are 30 away from outside wall
                right = max(30,right)
                return (True, 0, left, right)
            else:
                # Anti clockwise,
                # make sure you are away from the inside wall you can read.
                left = max(25,left)
                return (True, 0, left, right)

        deltadistance = right - left
        delta_yaw = 0
        # If Delta is high move towards the center, move by 10cm otherwise too high correction.
        if abs(deltadistance)> 20 or right <= 20 or left <= 20:
            correction = 5 #10 if front > 130 else 5
            yaw_correction = 1.5 if front > 130 else 2
            if left < right:
                logger.info("Adjusting left distance, moving to right")
                left += correction
                right -= correction
                delta_yaw = -yaw_correction
            else:
                logger.info("Adjusting right distance,moving to left")
                left -= correction
                right += correction
                delta_yaw = yaw_correction
            logger.info("adjusted left %.2f , right %.2f, yaw %.2f",left,right,delta_yaw)
            return True,delta_yaw,left, right
        else:
            logger.info("Delta distance is low, not changing distances.")
            return False,prev_yaw,left,right

    def side_bot_centering(self,front:float,learned_left:float,learned_right:float,
                                actual_left:float,actual_right:float,
                                prev_yaw:float,lenient:bool=False)-> Tuple[bool,float,float,float]:
        """Center the bot based on side distances."""

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
                logger.info("centering now....")
                return self.center_bot_correction(front, actual_left, actual_right,prev_yaw)
            else:

                logger.info("Proportional reduction of actual distances")
                # total actual > total_learned
                # we need to proportionally reduce actual

                if actual_left == self._left_max or actual_right == self._right_max:
                    # If we are at the max distance, we need to be careful
                    logger.info("At max distance, adjusting...")
                    if actual_left == self._left_max and actual_right != self._right_max:
                        #we can adjust the right distance
                        if actual_right > total_learned/2:
                            actual_right -= 5
                    elif actual_right == self._right_max and actual_left != self._left_max:
                        #we can adjust the left distance
                        if actual_left > total_learned/2:
                            actual_left -= 5
                    logger.info("Adjusted distances: Left: %.2f, Right: %.2f",
                                actual_left, actual_right)
                    return (True, prev_yaw, actual_left, actual_right)

                diff = total_actual - total_learned
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

    def runtest(self):
        """Run all tests for the Walker class."""

        logger = logging.getLogger()
        logger.setLevel(logging.DEBUG)

         # Console handler for WARNING and above
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.DEBUG)
        console_formatter = logging.Formatter("%(levelname)s - %(message)s")
        console_handler.setFormatter(console_formatter)
        logger.addHandler(console_handler)

        logger.info("Running LogicRound1Test")

        self.test_side_bot_centering()

    def test_side_bot_centering(self):
        """Test the side_bot_centering method of the Walker class."""

        # Call the method with test parameters
        (is_correction, yaw_delta, left_def, right_def) = self.side_bot_centering(front=150,
                                                learned_left=48.19, learned_right=48.19,
                                                actual_left=52.97, actual_right=200, prev_yaw=0)

        logger.info("Test Side Bot Centering Result: %s",
                    (is_correction, yaw_delta, left_def, right_def))
        
        (is_correction, yaw_delta, left_def, right_def) = self.side_bot_centering(front=150,
                                                learned_left=48.19, learned_right=48.19,
                                                actual_left=200, actual_right=52.97, prev_yaw=0)

        logger.info("Test Side Bot Centering Result: %s",
                    (is_correction, yaw_delta, left_def, right_def))


def main():
    """Main function to run """

    test = LogicRound1Test()
    print("Starting LogicRound1Test")
    test.runtest()


if __name__ == "__main__":
    main()
