from robot import *
from config import ROBOT_330_TIME

from time import sleep

my_robot = Robot(config_dict=ROBOT_330_TIME)


my_robot.check_motor_status(["all"])
my_robot.move_motors_sync(args={1:150, 2:150, 3:150, 4:150}, duration={"base":1000, "tower_1":2000, 3:3000, 4:4000}, degrees=True)
my_robot.check_motor_status(["all"])
time.sleep(2)

my_robot.move_motors_sync(args={1:0, 2:0, 3:0, 4:0}, duration={"base":5000, "tower_1":4000, 3:3000, 4:2000}, degrees=True)
my_robot.check_motor_status(["all"])
time.sleep(2)
my_robot.move_motors_sync(args={1:-150, 2:-150, 3:-150, 4:-150}, duration={"base":1000, "tower_1":5000, 3:5000, 4:1000}, degrees=True)
my_robot.check_motor_status(["all"])
time.sleep(2)

my_robot.move_motors_sync(args={1:0, 2:0, 3:0, 4:-0}, duration={"base":3000, "tower_1":3000, 3:3000, 4:3000}, degrees=True)
my_robot.check_motor_status(["all"])

my_robot.clean_shutdown()