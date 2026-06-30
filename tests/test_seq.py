from blossom_sdk.sequence import *
from blossom_sdk.config import *
from blossom_sdk.robot import *
import blossom_sdk.log_conf as log_conf


my_robot = Robot(config_dict=ROBOT_320)
my_robot.enable_torque()

my_sequence = Sequence(file_name="Sequences/tiny_test.json", robot_config=ROBOT_320)

my_sequence.to_string()
my_sequence.play_sequence(robot=my_robot)
my_robot.clean_shutdown()