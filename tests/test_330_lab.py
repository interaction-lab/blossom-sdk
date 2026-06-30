from blossom_sdk.robot import *
from blossom_sdk.config import *
import time

from blossom_sdk.log_conf import logger

# SET TESTING POSITIONS

# MOTOR 1: Head up (positive max 40) / down (negative min -100)
# MOTOR 2: Head tilt right (negative, 0-> -130) 
# MOTOR 3: Head tilt left (negative, 0-> -130) 
# MOTOR 4: Base swivel left (positive, 80 is full) / right (negative, 80 is full)
# MOTOR 5: Ears up (-15 kinda lower positive up to 130)
# MOTOR 6: Lungs  (from inhale at -60 to expanded at 80)

CENTER_POS = {1:0, 2:0, 3:0, 4:0, 5:0}
LEFT_POS = {1:0, 2:0, 3:0, 4:45, 5:0}
RIGHT_POS = {1:0, 2:0, 3:0, 4:-45, 5:0}

DURATION = 1000
DURATION_5_MOTORS = {1:DURATION, 2:DURATION, 3:DURATION, 4:DURATION, 5:DURATION}


my_robot = Robot(config_dict=ROBOT_330_LAB)

logger.info("Starting positions")
my_robot.check_motor_status(["all"])

# MOVE TO CENTER
my_robot.move_motors_sync(args=CENTER_POS, duration=DURATION_5_MOTORS)
logger.info("Result of move to CENTER")
my_robot.check_motor_status(["all"])
logger.info("Sleep 2")
time.sleep(2)

# MOVE TO LEFT
my_robot.move_motors_sync(args=LEFT_POS, duration=DURATION_5_MOTORS)
logger.info("Result of move to LEFT")
my_robot.check_motor_status(["all"])
logger.info("Sleep 2")
time.sleep(2)

# MOVE TO RIGHT
my_robot.move_motors_sync(args=RIGHT_POS, duration=DURATION_5_MOTORS)
logger.info("Result of move to RIGHT")
my_robot.check_motor_status(["all"])
logger.info("Sleep 2")
time.sleep(2)

# MOVE TO CENTER
my_robot.move_motors_sync(args=CENTER_POS, duration=DURATION_5_MOTORS)
logger.info("Result of move to CENTER")
my_robot.check_motor_status(["all"])
logger.info("Sleep 2")
time.sleep(2)

my_robot.check_motor_status(["all"])

my_robot.clean_shutdown()
logger.info("Ended")