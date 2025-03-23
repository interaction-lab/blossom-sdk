# Based on blossom-public version of sequences 
import json
import time 
import logging 

import jsonschema
from jsonschema import validate

from robot import *

SCHEMA_PATH = "Sequences/sequence_schema.json"

class Sequence():
    def __init__(self, file_name, robot_config):
        # validate file 
        self.validate_file(file_path=file_name)

        # get the raw sequence dictionary 
        self.seq_dict = self.get_dict(file_name=file_name)

        # get the name of the animation 
        self.name = self.seq_dict['animation']

        # get a list of motors, frame times, and frame positions 
        # frame times and positions should be of the same length 
        self.motors_used, self.frame_times, self.frame_positions, self.frame_durations = self.interpret_sequence(robot_config)

        self.num_frames = len(self.frame_positions)

    def validate_file(self, file_path):
        try:
            # Load JSON Schema from file
            with open(SCHEMA_PATH, "r") as schema_file:
                schema = json.load(schema_file)

            # Load JSON data from file
            with open(file_path, "r") as json_file:
                json_data = json.load(json_file)

            # Validate JSON against the schema
            validate(instance=json_data, schema=schema)

            print("Sequence is valid against the schema!")

        except jsonschema.exceptions.ValidationError as e:
            print("JSON validation error:", e.message)
            quit()

        except jsonschema.exceptions.SchemaError as e:
            print("Schema error:", e.message)
            quit()

        except FileNotFoundError as e:
            print("File not found:", e)
            quit()

        except json.JSONDecodeError as e:
            print("Invalid JSON format:", e)
            quit()

        except Exception as e:
            print("Unexpected error:", e)
            quit()

    def get_dict(self, file_name):
        """Load the raw sequence dictionary from a json file."""
        with open(file_name) as fn:
            return json.load(fn)

    def interpret_sequence(self, robot_config):
        """Gets a list of motors, frame times, frame positions, and frame 
        durations from the raw sequence dictionary for each frame in the 
        sequence. Also checks that motors in sequence match available motors 
        from robot config file. If more motors are available in the sequence 
        than in the config file, drops the additional motors from the 
        sequence data structure."""

        # get available motors from config file 
        available_motors = [key for key in robot_config["motors"]]

        # get motors used from first frame 
        # order matters here! Is the same as order of the positions! 
        motors_used = [position["dof"] for position in self.seq_dict["frame_list"][0]["positions"]]

        # get indices of motors in motors_used that aren't in config 
        unavailable_idx = []
        unavailable = [] 
        for i in range(len(motors_used)):
            if motors_used[i] not in available_motors:
                unavailable_idx.append(i)
                unavailable.append(motors_used[i])

        for motor in unavailable:
            motors_used.remove(motor)

        # get the frame times in milliseconds, and a list of lists of the positions in each frame 
        frame_times = []
        frame_positions = []
        frame_durations = []
        for frame in self.seq_dict["frame_list"]:
            frame_times.append(frame['millis'])
            
            # exclude position if at the index of an unused motor 
            positions = []
            for i in range(len(frame["positions"])):
                if i not in unavailable_idx:
                    positions.append(frame["positions"][i]["pos"])

            frame_positions.append(positions)

            # exclude duration if at the index of an unused motor
            durations = []
            for i in range(len(frame["positions"])):
                if i not in unavailable_idx:
                    durations.append(frame["positions"][i]["duration"])

            frame_durations.append(durations)

        # check that frame times and positions are the same size 
        assert len(frame_times) == len(frame_positions)
        assert len(frame_times) == len(frame_durations)

        return (motors_used, frame_times, frame_positions, frame_durations)

    def play_sequence(self, robot=None):
        # start time
        start_time = time.clock_gettime_ns(time.CLOCK_MONOTONIC)

        final_sleep = 0
        # iterate through the list of frames in the sequence
        for i in range(self.num_frames):
            # get args for motor movement
            args = {}
            for j in range(len(self.motors_used)):
                args[self.motors_used[j]] = self.frame_positions[i][j]

            # get durations for motor movement 
            durations = {}
            for j in range(len(self.motors_used)):
                durations[self.motors_used[j]] = self.frame_durations[i][j]

            delta_time_ms = (time.clock_gettime_ns(time.CLOCK_MONOTONIC)-start_time) / 1000000.0
            t_delay_ms = self.frame_times[i] - delta_time_ms

            # if the calculated delay is negative, there is no delay
            if t_delay_ms > 0:
                t_delay_s = t_delay_ms/1000.0
                # sleep for time delay
                logging.info(f"Sleeping for {t_delay_s}")
                time.sleep(t_delay_s)

            if i == self.num_frames - 1:
                final_sleep = max(self.frame_durations[i])

            # move robot
            robot.move_motors_sync(args, duration=durations,degrees=True)
            
        time.sleep((final_sleep/1000.0) + 0.1)
        return 1

    def to_string(self):
        print(self.name)
        print(self.seq_dict)
        print(self.motors_used)
        print(self.frame_times)
        print(self.frame_positions)
        print(self.frame_durations)