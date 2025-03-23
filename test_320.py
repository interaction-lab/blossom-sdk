from robot import *
from config import ROBOT_320

from time import sleep


my_robot = Robot(ROBOT_320)

# my_robot.to_string()

my_robot.enable_torque()

# testing check_motor_status 
result = my_robot.check_motor_status([1, 2, 3, 4, 5])
assert result == 1

result = my_robot.check_motor_status(["all"])
assert result == 1

result = my_robot.check_motor_status(["base", "tower_1", "tower_2", "tower_3", "ears"])
assert result == 1

result = my_robot.check_motor_status([1, "base"])
assert result == 1

result = my_robot.check_motor_status(["all", 1])
assert result == 0

result = my_robot.check_motor_status([1, "all"])
assert result == 0

result = my_robot.check_motor_status([1, 20, 5])
assert result == 0

result = my_robot.check_motor_status(["bases", 1])
assert result == 0


# testing get_diagnostic 

result = my_robot.get_diagnostic(["all"])
assert result == 1

result = my_robot.get_diagnostic([1, 2, 3, 4, 5])
assert result == 1

result = my_robot.get_diagnostic([2, 3])
assert result == 1

result = my_robot.get_diagnostic(["base", "tower_1", "tower_2", "tower_3", "ears"])
assert result == 1

result = my_robot.get_diagnostic([1, "base"])
assert result == 1

result = my_robot.get_diagnostic(["all", 1])
assert result == 0

result = my_robot.get_diagnostic([1, "all"])
assert result == 0

result = my_robot.get_diagnostic([1, 20, 5])
assert result == 0

result = my_robot.get_diagnostic(["bases", 1])
assert result == 0


# testing move_motors

my_robot.check_motor_status(["all"])
my_robot.move_motors({1: 0, 2:0, 3:0, 4:0, 5:0}, degrees=False)
sleep(2)
my_robot.check_motor_status(["all"])
my_robot.move_motors({1: 250, 2:250, 3:250, 4:250, 5:250}, degrees=False)
sleep(2)
my_robot.check_motor_status(["all"])
my_robot.move_motors({1: 500, 2:500, 3:500, 4:500, 5:500}, degrees=False)
sleep(2)
my_robot.check_motor_status(["all"])
my_robot.move_motors({1: 750, 2:750, 3:750, 4:750, 5:750}, degrees=False)
sleep(2)
my_robot.check_motor_status(["all"])
my_robot.move_motors({1: 1000, 2:1000, 3:1000, 4:1000, 5:1000}, degrees=False)
my_robot.check_motor_status(["all"])

my_robot.move_motors({"base": 0, "tower_1":0, "tower_2":0, "tower_3":0, "ears":0}, degrees=False)
sleep(2)
my_robot.check_motor_status(["all"])
my_robot.move_motors({"base": 250, "tower_1":250, "tower_2":250, "tower_3":250, "ears":250}, degrees=False)
sleep(2)
my_robot.check_motor_status(["all"])
my_robot.move_motors({"base": 500, "tower_1":500, "tower_2":500, "tower_3":500, "ears":500}, degrees=False)
sleep(2)
my_robot.check_motor_status(["all"])
my_robot.move_motors({"base": 750, "tower_1":750, "tower_2":750, "tower_3":750, "ears":750}, degrees=False)
sleep(2)
my_robot.check_motor_status(["all"])
my_robot.move_motors({"base": 1000, "tower_1":1000, "tower_2":1000, "tower_3":1000, "ears":1000}, degrees=False)
sleep(2)
my_robot.check_motor_status(["all"])

my_robot.move_motors({1: 0, "tower_1":0, "tower_2":0, "tower_3":0, "ears":0}, degrees=False)
sleep(2)
my_robot.check_motor_status(["all"])
my_robot.move_motors({"base": 250, 2:250, "tower_2":250, "tower_3":250, "ears":250}, degrees=False)
sleep(2)
my_robot.check_motor_status(["all"])
my_robot.move_motors({"base": 500, "tower_1":500, 3:500, "tower_3":500, "ears":500}, degrees=False)
sleep(2)
my_robot.check_motor_status(["all"])
my_robot.move_motors({"base": 750, "tower_1":750, "tower_2":750, 4:750, "ears":750}, degrees=False)
sleep(2)
my_robot.check_motor_status(["all"])
my_robot.move_motors({"base": 1000, "tower_1":1000, "tower_2":1000, "tower_3":1000, 5:1000}, degrees=False)
sleep(2)
my_robot.check_motor_status(["all"])


# Testing move motors sync 
my_robot.check_motor_status(["all"])
my_robot.move_motors_sync({1: 0, 2:0, 3:0, 4:0, 5:0}, degrees=False)
sleep(2)
my_robot.check_motor_status(["all"])
my_robot.move_motors_sync({1: 250, 2:250, 3:250, 4:250, 5:250}, degrees=False)
sleep(2)
my_robot.check_motor_status(["all"])
my_robot.move_motors_sync({1: 500, 2:500, 3:500, 4:500, 5:500}, degrees=False)
sleep(2)
my_robot.check_motor_status(["all"])
my_robot.move_motors_sync({1: 750, 2:750, 3:750, 4:750, 5:750}, degrees=False)
sleep(2)
my_robot.check_motor_status(["all"])
my_robot.move_motors_sync({1: 1000, 2:1000, 3:1000, 4:1000, 5:1000}, degrees=False)
my_robot.check_motor_status(["all"])

my_robot.move_motors_sync({"base": 0, "tower_1":0, "tower_2":0, "tower_3":0, "ears":0}, degrees=False)
sleep(2)
my_robot.check_motor_status(["all"])
my_robot.move_motors_sync({"base": 250, "tower_1":250, "tower_2":250, "tower_3":250, "ears":250}, degrees=False)
sleep(2)
my_robot.check_motor_status(["all"])
my_robot.move_motors_sync({"base": 500, "tower_1":500, "tower_2":500, "tower_3":500, "ears":500}, degrees=False)
sleep(2)
my_robot.check_motor_status(["all"])
my_robot.move_motors_sync({"base": 750, "tower_1":750, "tower_2":750, "tower_3":750, "ears":750}, degrees=False)
sleep(2)
my_robot.check_motor_status(["all"])
my_robot.move_motors_sync({"base": 1000, "tower_1":1000, "tower_2":1000, "tower_3":1000, "ears":1000}, degrees=False)
sleep(2)
my_robot.check_motor_status(["all"])

my_robot.move_motors_sync({1: 0, "tower_1":0, "tower_2":0, "tower_3":0, "ears":0}, degrees=False)
sleep(2)
my_robot.check_motor_status(["all"])
my_robot.move_motors_sync({"base": 250, 2:250, "tower_2":250, "tower_3":250, "ears":250}, degrees=False)
sleep(2)
my_robot.check_motor_status(["all"])
my_robot.move_motors_sync({"base": 500, "tower_1":500, 3:500, "tower_3":500, "ears":500}, degrees=False)
sleep(2)
my_robot.check_motor_status(["all"])
my_robot.move_motors_sync({"base": 750, "tower_1":750, "tower_2":750, 4:750, "ears":750}, degrees=False)
sleep(2)
my_robot.check_motor_status(["all"])
my_robot.move_motors_sync({"base": 1000, "tower_1":1000, "tower_2":1000, "tower_3":1000, 5:1000}, degrees=False)
sleep(2)
my_robot.check_motor_status(["all"])

my_robot.clean_shutdown()