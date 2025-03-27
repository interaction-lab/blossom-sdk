from robot import *
from config import *
import time

my_robot = Robot(config_dict=ROBOT_330_LAB)

print("Starting positions")
my_robot.check_motor_status(["all"])

my_robot.move_motors_sync(args={1:150, 2:150, 3:150, 4:150, 5:150}, duration={1:5000, 2:5000, 3:5000, 4:5000, 5:5000})
print("\nResult of move to 150")
my_robot.check_motor_status(["all"])
print("\nSleep 2")
time.sleep(2)

my_robot.move_motors_sync(args={1:0, 2:0, 3:0, 4:0, 5:0}, duration={1:5000, 2:5000, 3:5000, 4:5000, 5:5000})
print("\nResult of move to 0")
my_robot.check_motor_status(["all"])
print("\nSleep 2")
time.sleep(2)

my_robot.move_motors_sync(args={1:-150, 2:-150, 3:-150, 4:-150, 5:-150}, duration={1:5000, 2:5000, 3:5000, 4:5000, 5:5000})
print("\nResult of move to -150")
my_robot.check_motor_status(["all"])
print("\nSleep 2")
time.sleep(2)

my_robot.clean_shutdown()