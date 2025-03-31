import log_conf

from robot import *
from config import ROBOT_330

from time import sleep


my_robot = Robot(ROBOT_330)

# my_robot.to_string()

my_robot.enable_torque()

# testing check_motor_status 
'''
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
'''

# testing get_diagnostic 
'''
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
'''

# testing move_motors

# my_robot.check_motor_status(["all"])
# my_robot.move_motors({1: 1000, 2:1000, 3:1000, 4:1000, 5:1000}, degrees=False)
# sleep(2)
# my_robot.check_motor_status(["all"])
# my_robot.move_motors({1: 1500, 2:1500, 3:1500, 4:1500, 5:1500}, degrees=False)
# sleep(2)
# my_robot.check_motor_status(["all"])
# my_robot.move_motors({1: 2000, 2:2000, 3:2000, 4:2000, 5:2000}, degrees=False)
# sleep(2)
# my_robot.check_motor_status(["all"])
# my_robot.move_motors({1: 2500, 2:2500, 3:2500, 4:2500, 5:2500}, degrees=False)
# sleep(2)
# my_robot.check_motor_status(["all"])
# my_robot.move_motors({1: 3000, 2:3000, 3:3000, 4:3000, 5:3000}, degrees=False)
# my_robot.check_motor_status(["all"])

# my_robot.move_motors({"base": 1000, "tower_1":1000, "tower_2":1000, "tower_3":1000, "ears":1000}, degrees=False)
# sleep(2)
# my_robot.check_motor_status(["all"])
# my_robot.move_motors({"base": 1500, "tower_1":1500, "tower_2":1500, "tower_3":1500, "ears":1500}, degrees=False)
# sleep(2)
# my_robot.check_motor_status(["all"])
# my_robot.move_motors({"base": 2000, "tower_1":2000, "tower_2":2000, "tower_3":2000, "ears":2000}, degrees=False)
# sleep(2)
# my_robot.check_motor_status(["all"])
# my_robot.move_motors({"base": 2500, "tower_1":2500, "tower_2":2500, "tower_3":2500, "ears":2500}, degrees=False)
# sleep(2)
# my_robot.check_motor_status(["all"])
# my_robot.move_motors({"base": 3000, "tower_1":3000, "tower_2":3000, "tower_3":3000, "ears":3000}, degrees=False)
# sleep(2)
# my_robot.check_motor_status(["all"])

# my_robot.move_motors({1: 1000, "tower_1":1000, "tower_2":1000, "tower_3":1000, "ears":1000}, degrees=False)
# sleep(2)
# my_robot.check_motor_status(["all"])
# my_robot.move_motors({"base": 1500, 2:1500, "tower_2":1500, "tower_3":1500, "ears":1500}, degrees=False)
# sleep(2)
# my_robot.check_motor_status(["all"])
# my_robot.move_motors({"base": 2000, "tower_1":2000, 3:2000, "tower_3":2000, "ears":2000}, degrees=False)
# sleep(2)
# my_robot.check_motor_status(["all"])
# my_robot.move_motors({"base": 2500, "tower_1":2500, "tower_2":2500, 4:2500, "ears":2500}, degrees=False)
# sleep(2)
# my_robot.check_motor_status(["all"])
# my_robot.move_motors({"base": 3000, "tower_1":3000, "tower_2":3000, "tower_3":3000, 5:3000}, degrees=False)
# sleep(2)
# my_robot.check_motor_status(["all"])

# Testing move motors sync 
# my_robot.check_motor_status(["all"])
# my_robot.move_motors_sync({1: 1000, 2:1000, 3:1000, 4:1000, 5:1000}, degrees=False)
# sleep(2)
# my_robot.check_motor_status(["all"])
# my_robot.move_motors_sync({1: 1500, 2:1500, 3:1500, 4:1500, 5:1500}, degrees=False)
# sleep(2)
# my_robot.check_motor_status(["all"])
# my_robot.move_motors_sync({1: 2000, 2:2000, 3:2000, 4:2000, 5:2000}, degrees=False)
# sleep(2)
# my_robot.check_motor_status(["all"])
# my_robot.move_motors_sync({1: 2500, 2:2500, 3:2500, 4:2500, 5:2500}, degrees=False)
# sleep(2)
# my_robot.check_motor_status(["all"])
# my_robot.move_motors_sync({1: 3000, 2:3000, 3:3000, 4:3000, 5:3000}, degrees=False)
# my_robot.check_motor_status(["all"])

# my_robot.move_motors_sync({"base": 1000, "tower_1":1000, "tower_2":1000, "tower_3":1000, "ears":1000}, degrees=False)
# sleep(2)
# my_robot.check_motor_status(["all"])
# my_robot.move_motors_sync({"base": 1500, "tower_1":1500, "tower_2":1500, "tower_3":1500, "ears":1500}, degrees=False)
# sleep(2)
# my_robot.check_motor_status(["all"])
# my_robot.move_motors_sync({"base": 2000, "tower_1":2000, "tower_2":2000, "tower_3":2000, "ears":2000}, degrees=False)
# sleep(2)
# my_robot.check_motor_status(["all"])
# my_robot.move_motors_sync({"base": 2500, "tower_1":2500, "tower_2":2500, "tower_3":2500, "ears":2500}, degrees=False)
# sleep(2)
# my_robot.check_motor_status(["all"])
# my_robot.move_motors_sync({"base": 3000, "tower_1":3000, "tower_2":3000, "tower_3":3000, "ears":3000}, degrees=False)
# sleep(2)
# my_robot.check_motor_status(["all"])

# my_robot.move_motors_sync({1: 1000, "tower_1":1000, "tower_2":1000, "tower_3":1000, "ears":1000}, degrees=False)
# sleep(2)
# my_robot.check_motor_status(["all"])
# my_robot.move_motors_sync({"base": 1500, 2:1500, "tower_2":1500, "tower_3":1500, "ears":1500}, degrees=False)
# sleep(2)
# my_robot.check_motor_status(["all"])
# my_robot.move_motors_sync({"base": 2000, "tower_1":2000, 3:2000, "tower_3":2000, "ears":2000}, degrees=False)
# sleep(2)
# my_robot.check_motor_status(["all"])
# my_robot.move_motors_sync({"base": 2500, "tower_1":2500, "tower_2":2500, 4:2500, "ears":2500}, degrees=False)
# sleep(2)
# my_robot.check_motor_status(["all"])
# my_robot.move_motors_sync({"base": 3000, "tower_1":3000, "tower_2":3000, "tower_3":3000, 5:3000}, degrees=False)
# sleep(2)
# my_robot.check_motor_status(["all"])

my_robot.move_motors({1:3000}, degrees=False)

my_robot.clean_shutdown()