#!/usr/bin/env python
# -*- coding: utf-8 -*-

# TODO: velocity limit 

import time
from log_conf import logger

from dynamixel_sdk import *

from control_table_defs import *
from conversion import *

class Robot:
    def __init__(self, config_dict):
        config_controllers = config_dict["controllers"]
        config_motors = config_dict["motors"]

        # interface constants 
        self.device_name = config_controllers["port"]
        self.protocol = config_controllers["protocol"]
        self.baud_rate = config_controllers["baudrate"]
        self.blocking = config_controllers["blocking"]

        # motor configuration from config_dict only 
        self.dxl_ids = []
        self.names = []

        self.name_to_id = {}
        self.id_to_name = {}

        self.id_to_limit = {}
        self.name_to_limit = {}

        self.model_type = -1
        model_types = []

        for alias in config_motors:
            # find all ids and names 
            self.dxl_ids.append(config_motors[alias]["id"])
            self.names.append(alias)

            # get all model types 
            model_types.append(config_motors[alias]["type"])

            # build conversion dictionaries 
            self.id_to_name[config_motors[alias]["id"]] = alias
            self.name_to_id[alias] = config_motors[alias]["id"]

        logger.info(f"Motors with ids {self.dxl_ids} found in config file.")

        # confirm all motors in config are of same type 
        model_set = set(model_types)
        if len(model_set) > 1:
            logger.critical(f"Cannot combine motors of different types: {model_set}")
            quit()
        elif model_types[0] != 350 and model_types[0] != 1200:
            logger.critical(f"Non XL-320 and XL-330 motors specified in robot configuration.")
            quit()
        else:
            self.model_type = model_types[0]
        
        # initialize port and packet handlers 
        self.port_handler = PortHandler(self.device_name)
        self.packet_handler = PacketHandler(self.protocol)

        if not self.port_handler.openPort():
            logger.critical(f"Failed to open port: {self.device_name}")
            quit()
        else:
            logger.info(f"Successfully opened port: {self.device_name}")

        if not self.port_handler.setBaudRate(self.baud_rate):
            logger.critical(f"Failed to set baud rate: {self.baud_rate}")
            quit()
        else:
            logger.info(f"Successfully set baud rate: {self.baud_rate}")

        # ping motors to confirm that model type is correct and all ids are accessible in the setup 
        model_nums = []
        for id in self.dxl_ids:
            dxl_model_number, dxl_comm_result, dxl_error = self.packet_handler.ping(self.port_handler, id)

            model_nums.append(dxl_model_number)

            if dxl_comm_result != COMM_SUCCESS:
                logger.critical(f"{self.packet_handler.getTxRxResult(dxl_comm_result)}")
                quit()
            elif dxl_error != 0:
                logger.critical(f"{self.packet_handler.getRxPacketError(dxl_error)}")
                quit()

        ping_set = set(model_nums)
        if len(ping_set) > 1:
            logger.critical(f"Cannot combine motors of different types: {ping_set}")
            quit()

        ping_type = model_nums[0]
        if ping_type != self.model_type:
            logger.critical(f"Motor type in config file {self.model_type} incompatible with detected type {ping_type}")
            quit()
        else:
            logger.info(f"Successfully confirmed model type {self.model_type}")

        # get all angle limits and build appropriate libraries 
        for alias in config_motors:
            limits = config_motors[alias]["angle_limit"]
            dxl_limits = [degree_to_dxl(angle, self.model_type) for angle in limits]

            self.id_to_limit[config_motors[alias]["id"]] = dxl_limits
            self.name_to_limit[alias] = dxl_limits

        # finish motor configuration with control table constants 
        if self.model_type == 350:   # XL320
            self.ADDR_TORQUE_ENABLE = XL320_CONFIG["ADDR_TORQUE_ENABLE"]
            self.ADDR_GOAL_POSITION = XL320_CONFIG["ADDR_GOAL_POSITION"]
            self.ADDR_PRESENT_POSITION = XL320_CONFIG["ADDR_PRESENT_POSITION"]
            self.ADDR_HARDWARE_ERROR_STATUS = XL320_CONFIG["ADDR_HARDWARE_ERROR_STATUS"]
            self.TORQUE_ENABLE = XL320_CONFIG["TORQUE_ENABLE"]
            self.TORQUE_DISABLE = XL320_CONFIG["TORQUE_DISABLE"]
            self.ADDR_MOVING_SPEED = XL320_CONFIG["ADDR_MOVING_SPEED"]
            self.ADDR_TORQUE_LIMIT = XL320_CONFIG["ADDR_TORQUE_LIMIT"]
            self.ADDR_P_GAIN = XL320_CONFIG["ADDR_P_GAIN"]
            self.ADDR_MOVING = XL320_CONFIG["ADDR_MOVING"]

            self.ADDR_CW_ANGLE_LIMIT = XL320_CONFIG["ADDR_CW_ANGLE_LIMIT"]
            self.ADDR_CCW_ANGLE_LIMIT = XL320_CONFIG["ADDR_CCW_ANGLE_LIMIT"]

            self.VALID_DXL = (0, 1023)       # range where 320 can move 

        elif self.model_type == 1200:    # XL330
            self.ADDR_TORQUE_ENABLE = XL330_CONFIG["ADDR_TORQUE_ENABLE"]
            self.ADDR_GOAL_POSITION = XL330_CONFIG["ADDR_GOAL_POSITION"]
            self.ADDR_PRESENT_POSITION = XL330_CONFIG["ADDR_PRESENT_POSITION"]
            self.ADDR_HARDWARE_ERROR_STATUS = XL330_CONFIG["ADDR_HARDWARE_ERROR_STATUS"]
            self.TORQUE_ENABLE = XL330_CONFIG["TORQUE_ENABLE"]
            self.TORQUE_DISABLE = XL330_CONFIG["TORQUE_DISABLE"]
            self.ADDR_PROFILE_ACCELERATION = XL330_CONFIG["ADDR_PROFILE_ACCELERATION"]
            self.ADDR_PROFILE_VELOCITY = XL330_CONFIG["ADDR_PROFILE_VELOCITY"]
            self.ADDR_DRIVE_MODE = XL330_CONFIG["ADDR_DRIVE_MODE"]
            self.ADDR_MOVING_THRESHOLD = XL330_CONFIG["ADDR_MOVING_THRESHOLD"]
            self.ADDR_MOVING = XL330_CONFIG["ADDR_MOVING"]

            self.ADDR_MAX_POSITION_LIMIT = XL330_CONFIG["ADDR_MAX_POSITION_LIMIT"]
            self.ADDR_MIN_POSITION_LIMIT = XL330_CONFIG["ADDR_MIN_POSITION_LIMIT"]

            self.VALID_DXL = (341, 3755)     # range where 330 can move 

        # check that angle limits are within valid range for each dxl motor, reset if necessary 
        for id in self.id_to_limit:
            if self.id_to_limit[id][0] < self.VALID_DXL[0]:
                self.id_to_limit[id][0] = dxl_to_degree(self.VALID_DXL[0], self.model_type)
                self.name_to_limit[self.id_to_name[id]][0] = dxl_to_degree(self.VALID_DXL[0], self.model_type)
            
            if self.id_to_limit[id][1] > self.VALID_DXL[1]:
                self.id_to_limit[id][1] = dxl_to_degree(self.VALID_DXL[1], self.model_type)
                self.name_to_limit[self.id_to_name[id]][1] = dxl_to_degree(self.VALID_DXL[1], self.model_type)

        # initialize instance for group sync read/write
        if self.model_type == 350:
            self.group_goal_write = GroupSyncWrite(self.port_handler, self.packet_handler, 
                                                   self.ADDR_GOAL_POSITION, CT_XL320_ADDR[self.ADDR_GOAL_POSITION][1])
            self.group_position_read = GroupSyncRead(self.port_handler, self.packet_handler, 
                                                 self.ADDR_PRESENT_POSITION, CT_XL320_ADDR[self.ADDR_PRESENT_POSITION][1])
            
            self.group_move_read = GroupSyncRead(self.port_handler, self.packet_handler, 
                                                 self.ADDR_MOVING, CT_XL320_ADDR[self.ADDR_MOVING][1])
        elif self.model_type == 1200:
            self.group_goal_write = GroupSyncWrite(self.port_handler, self.packet_handler, 
                                                   self.ADDR_GOAL_POSITION, CT_XL330_ADDR[self.ADDR_GOAL_POSITION][1])
            self.group_position_read = GroupSyncRead(self.port_handler, self.packet_handler, 
                                                 self.ADDR_PRESENT_POSITION, CT_XL330_ADDR[self.ADDR_PRESENT_POSITION][1])
            
            self.group_move_read = GroupSyncRead(self.port_handler, self.packet_handler, 
                                                 self.ADDR_MOVING, CT_XL330_ADDR[self.ADDR_MOVING][1])

            self.group_duration_write = GroupSyncWrite(self.port_handler, self.packet_handler, 
                                                  self.ADDR_PROFILE_VELOCITY, CT_XL330_ADDR[self.ADDR_PROFILE_VELOCITY][1])
            self.group_duration_read = GroupSyncRead(self.port_handler, self.packet_handler, 
                                                     self.ADDR_PROFILE_VELOCITY, CT_XL330_ADDR[self.ADDR_PROFILE_VELOCITY][1])
            
        # add parameter storage for each motor's present position value 
        for dxl_id in self.dxl_ids:
            dxl_addparam_result = self.group_position_read.addParam(dxl_id)
            if dxl_addparam_result != True:
                logger.critical("[ID:%d] group_position_read addParam failed", dxl_id)
                quit()

            dxl_addparam_result = self.group_move_read.addParam(dxl_id)
            if dxl_addparam_result != True:
                logger.critical("[ID:%d] group_move_read addParam failed", dxl_id)
                quit()

        # hard code acceleration, velocity, and moving_threshold, and write to address 
        if self.model_type == 1200:
            self.acceleration = 10      # used to be 5, too slow 
            self.velocity = 200
            self.moving_threshold = 1

            self.drive_mode = config_controllers["drivemode"]

            for id in self.dxl_ids:
                self.packet_handler.write1ByteTxRx(self.port_handler, id, self.ADDR_DRIVE_MODE, self.drive_mode)
                self.packet_handler.write4ByteTxRx(self.port_handler, id, self.ADDR_PROFILE_ACCELERATION, self.acceleration)
                self.packet_handler.write4ByteTxRx(self.port_handler, id, self.ADDR_PROFILE_VELOCITY, self.velocity)
                self.packet_handler.write4ByteTxRx(self.port_handler, id, self.ADDR_MOVING_THRESHOLD, self.moving_threshold)
                logger.info(f"Set profile acceleration ({self.acceleration}) and velocity ({self.velocity}) set for motor {id}")

        # hard code moving speed, torque limit, and P gain
        elif self.model_type == 350:
            self.moving_speed = 100
            self.torque_limit = 512
            self.p_gain = 32
            self.drive_mode = 0

            for id in self.dxl_ids:
                self.packet_handler.write2ByteTxRx(self.port_handler, id, self.ADDR_MOVING_SPEED, self.moving_speed)
                self.packet_handler.write2ByteTxRx(self.port_handler, id, self.ADDR_TORQUE_LIMIT, self.torque_limit)
                self.packet_handler.write2ByteTxRx(self.port_handler, id, self.ADDR_P_GAIN, self.p_gain)
                logger.info(f"Set moving speed ({self.moving_speed}), torque limit ({self.torque_limit}), and P gain ({self.p_gain}) set for motor {id}")

        # enforce angle limits 
        if self.model_type == 1200:
            for id in self.dxl_ids:
                self.packet_handler.write4ByteTxRx(self.port_handler, id, self.ADDR_MIN_POSITION_LIMIT, self.id_to_limit[id][0])
                self.packet_handler.write4ByteTxRx(self.port_handler, id, self.ADDR_MAX_POSITION_LIMIT, self.id_to_limit[id][1])
                logger.info(f"Set min position limit ({self.id_to_limit[id][0]}), max position limit ({self.id_to_limit[id][1]})")
        elif self.model_type == 350:
            for id in self.dxl_ids:
                self.packet_handler.write2ByteTxRx(self.port_handler, id, self.ADDR_CW_ANGLE_LIMIT, self.id_to_limit[id][0])
                self.packet_handler.write2ByteTxRx(self.port_handler, id, self.ADDR_CCW_ANGLE_LIMIT, self.id_to_limit[id][1])
                logger.info(f"Set CW angle limit ({self.id_to_limit[id][0]}), CCW angle limit ({self.id_to_limit[id][1]})")

    def reset(self):
        '''
        Move all motors to position 0, or as close to 0 as their limit allows.
        
        Returns 0 if a move fails, 1 if all successful. 
        '''
        args = {id:0 for id in self.dxl_ids}
        result = self.move_motors_sync(args, degrees=True)
        return result 
    
    def to_string(self):
        ''' Prints all attributes. No return. '''
        logger.info("Device Name = ", self.device_name)
        logger.info("Protocol = ", self.protocol)
        logger.info("Baud Rate = ", self.baud_rate)
        logger.info("Model Type = ", self.model_type)
        logger.info("DXL IDS = ", self.dxl_ids)
        logger.info("Names = ", self.names)
        logger.info("Id, name = ", self.id_to_name)
        logger.info("Name, id = ", self.name_to_id)
        logger.info("Id, angle limit = ", self.id_to_limit)
        logger.info("Name, angle limit = ", self.name_to_limit)
        logger.info("Blocking Moves = ", self.blocking)

        if self.model_type == 350:
            logger.info("Moving speed = ", self.moving_speed)
            logger.info("Torque limit = ", self.torque_limit)
            logger.info("P gain = ", self.p_gain)
        elif self.model_type == 1200:
            logger.info("Drive mode = ", self.drive_mode)
            logger.info("Acceleration = ", self.acceleration)
            logger.info("Velocity = ", self.velocity)
            logger.info("Moving Threshold = ", self.moving_threshold)

    def check_motor_status(self, args):
        '''
        Checks the current position of the specified motors. 

        Inputs: args -- a list of str motor names or int ids, or a list containing only the string "all"
        Returns 1 if successful, 0 if input was invalid. 
        '''
        # if checking all, do not specify additional motors 
        if "all" in args and len(args) != 1:
            return 0
        
        # if specifying motors, motors must be valid 
        if args[0] != "all":
            for elem in args:
                if elem not in self.dxl_ids and elem not in self.names:
                    logger.error("%s not a valid motor name/id.", elem)
                    return 0
        
        # get status of all motors 
        if args[0] == "all":
            for dxl_id in self.dxl_ids:
                if self.model_type == 350:
                    pos, _, _ = self.packet_handler.read2ByteTxRx(self.port_handler, dxl_id, self.ADDR_PRESENT_POSITION)
                elif self.model_type == 1200:
                    pos, _, _ = self.packet_handler.read4ByteTxRx(self.port_handler, dxl_id, self.ADDR_PRESENT_POSITION)

                logger.info("Status Check: Motor %d Model Type: %d Position: %d", dxl_id, self.model_type, pos)

        # get status of specified motors 
        else:
            for motor in args:
                if type(motor) == str:
                    dxl_id = self.name_to_id[motor]

                    if self.model_type == 350:
                        pos, _, _ = self.packet_handler.read2ByteTxRx(self.port_handler, dxl_id, self.ADDR_PRESENT_POSITION)
                    elif self.model_type == 1200:
                        pos, _, _ = self.packet_handler.read4ByteTxRx(self.port_handler, dxl_id, self.ADDR_PRESENT_POSITION)

                    logger.info("Status Check: Motor %d Model Type: %d Position: %d", dxl_id, self.model_type, pos)

                elif type(motor) == int:
                    if self.model_type == 350:
                        pos, _, _ = self.packet_handler.read2ByteTxRx(self.port_handler, motor, self.ADDR_PRESENT_POSITION)
                    elif self.model_type == 1200:
                        pos, _, _ = self.packet_handler.read4ByteTxRx(self.port_handler, motor, self.ADDR_PRESENT_POSITION)

                    logger.info("Status Check: Motor %d Model Type: %d Position: %d", dxl_id, self.model_type, pos)
        
        return 1

    def get_diagnostic(self, args):
        '''
        Checks the error status of the specified motors 

        Inputs: args -- a list of str motor names or int ids, or a list containing only the string "all"
        Returns 1 if successful, 0 if input was invalid. 
        '''
        # if checking all, do not specify additional motors 
        if "all" in args and len(args) != 1:
            return 0
        
        # if specifying motors, motors must be valid 
        if args[0] != "all":
            for elem in args:
                if elem not in self.dxl_ids and elem not in self.names:
                    logger.error("%s not a valid motor name/id.", elem)
                    return 0
        
        # get error status of all motors 
        if args[0] == "all":
            for dxl_id in self.dxl_ids:
                error_status, _, _ = self.packet_handler.read1ByteTxRx(self.port_handler, dxl_id, self.ADDR_HARDWARE_ERROR_STATUS)                
                logger.info("Diagnostic: Motor %d Error Status: %s", dxl_id, bin(error_status))
                
        # get status of specified motors 
        else:
            for motor in args:
                if type(motor) == str:
                    dxl_id = self.name_to_id[motor]

                    error_status, _, _ = self.packet_handler.read1ByteTxRx(self.port_handler, dxl_id, self.ADDR_HARDWARE_ERROR_STATUS)
                    logger.info("Diagnostic: Motor %d Error Status: %s", dxl_id, bin(error_status))
                
                elif type(motor) == int:
                    error_status, _, _ = self.packet_handler.read1ByteTxRx(self.port_handler, motor, self.ADDR_HARDWARE_ERROR_STATUS)
                    logger.info("Diagnostic: Motor %d Error Status: %s", motor, bin(error_status))
        
        return 1

    def enable_torque(self):
        """Enables torque. Torque must be enabled before motors will move."""
        for id in self.dxl_ids:
            self.packet_handler.write1ByteTxRx(self.port_handler, id, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
            logger.info("Torque enabled for Motor %d", id)

    def disable_torque(self):
        """Disables torque. Torque must be disabled for a clean shutdown, and before setting certain values in the control table."""
        for id in self.dxl_ids:
            self.packet_handler.write1ByteTxRx(self.port_handler, id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
            logger.info("Torque disabled for Motor %d", id)

    def move_motors(self, args, duration=None, degrees=True):
        """Move motors sequentially. If blocking is set in the config, 
        waits for all movements in args to complete before continuing."""

        # translate into dynamixel compatible values if degrees=True
        if degrees == False:
            temp = args
        elif degrees == True:
            temp = {key: degree_to_dxl(args[key], self.model_type) for key in args}

        # translate any string names into ids and check that they exist 
        targets = {}
        for key in temp:
            if key not in self.names and key not in self.dxl_ids:
                logger.error("%s not a valid motor name/id.", key)
                return 0

            if type(key) == int:
                targets[key] = temp[key]
            elif type(key) == str:
                targets[self.name_to_id[key]] = temp[key]

        # verify that movement is within valid range 
        for target in targets:
            if targets[target] < self.id_to_limit[target][0]:
                logger.warning("Invalid movement target %d. Valid range = %d. Changed to %d", targets[target], self.id_to_limit[target], self.id_to_limit[target][0])
                targets[target] = self.id_to_limit[target][0]
            elif targets[target] > self.id_to_limit[target][1]:
                logger.warning(f"Invalid movement target %d. Valid range = %d. Changed to %d", targets[target], self.id_to_limit[target], self.id_to_limit[target][1])
                targets[target] = self.id_to_limit[target][1]

        # set duration 
        if (duration != None) and (self.drive_mode & DRIVE_MODE_TIME != 0):
            # check for valid times and motors 
            motor_list = [key for key in duration]
            times = {}

            for m in motor_list:
                if type(m) == int:
                    if m not in targets.keys():
                        logger.error("Invalid motor provided %d.", m)
                        return 0
                    else:
                        times[m] = duration[m]
                elif type(m) == str:
                    if m not in self.name_to_id.keys() or self.name_to_id[m] not in targets.keys():
                        logger.error("Invalid motor provided %d.", m)
                        return 0
                    else:
                        times[self.name_to_id[m]] = duration[m]
                    
            # change the times 
            for id in times:
                self.packet_handler.write4ByteTxRx(self.port_handler, id, self.ADDR_PROFILE_VELOCITY, times[id])

        # make the moves 
        if self.model_type == 350:
            for dxl_id, goal_position in targets.items():
                self.packet_handler.write2ByteTxRx(self.port_handler, dxl_id, self.ADDR_GOAL_POSITION, goal_position)
                logger.info("Motor %d Model Type: %d moved to position %d", dxl_id, self.model_type, goal_position)
        elif self.model_type == 1200:
            for dxl_id, goal_position in targets.items():
                self.packet_handler.write4ByteTxRx(self.port_handler, dxl_id, self.ADDR_GOAL_POSITION, goal_position)
                logger.info("Motor %d Model Type: %d moved to position %d", dxl_id, self.model_type, goal_position)

        # spin until completion only if blocking is set to True in the config 
        if self.blocking:
            self.check_move_complete()

        self.check_motor_status("all")

        return 1

    def move_motors_sync(self, args, duration=None, degrees=True):
        """Move motors simultaneously using group sync write and read. 
        If blocking is set in config, waits for all movements in args
        to complete before continuing. """
        # translate into dynamixel compatible values if degrees=True
        if degrees == False:
            temp = args
        elif degrees == True:
            temp = {key: degree_to_dxl(args[key], self.model_type) for key in args}

        # translate any string names into ids and check that they exist 
        targets = {}
        for key in temp:
            if key not in self.names and key not in self.dxl_ids:
                logger.error("%s not a valid motor name/id.", key)
                return 0

            if type(key) == int:
                targets[key] = temp[key]
            elif type(key) == str:
                targets[self.name_to_id[key]] = temp[key]

        # verify that movement is within valid range 
        for target in targets:
            if targets[target] < self.id_to_limit[target][0]:
                logger.warning("Invalid movement target %d. Valid range = %d. Changed to %d", targets[target], self.id_to_limit[target], self.id_to_limit[target][0])
                targets[target] = self.id_to_limit[target][0]
            elif targets[target] > self.id_to_limit[target][1]:
                logger.warning("Invalid movement target %d. Valid range = %d. Changed to %d", targets[target], self.id_to_limit[target], self.id_to_limit[target][1])
                targets[target] = self.id_to_limit[target][1]

        # set durations
        if (duration != None) and (self.drive_mode & DRIVE_MODE_TIME != 0):
            # check for valid times and motors 
            motor_list = [key for key in duration]
            times = {}

            for m in motor_list:
                if type(m) == int:
                    if m not in targets.keys():
                        logger.error("Invalid motor provided %d.", m)
                        return 0
                    else:
                        times[m] = [DXL_LOBYTE(DXL_LOWORD(duration[m])), DXL_HIBYTE(DXL_LOWORD(duration[m])), DXL_LOBYTE(DXL_HIWORD(duration[m])), DXL_HIBYTE(DXL_HIWORD(duration[m]))]
                elif type(m) == str:
                    if m not in self.name_to_id.keys() or self.name_to_id[m] not in targets.keys():
                        logger.error("Invalid motor provided %d.", m)
                        return 0
                    else:
                        times[self.name_to_id[m]] = [DXL_LOBYTE(DXL_LOWORD(duration[m])), DXL_HIBYTE(DXL_LOWORD(duration[m])), DXL_LOBYTE(DXL_HIWORD(duration[m])), DXL_HIBYTE(DXL_HIWORD(duration[m]))]
                    
            # add each profile velocity to the Syncwrite storage 
            for id in times:
                dxl_addparam_result = self.group_duration_write.addParam(id, times[id])
                if dxl_addparam_result != True:
                    logger.error("[ID:%d] group_duration_write addparam failed for duration", id)

            # Syncwrite durations 
            dxl_comm_result = self.group_duration_write.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                logger.error("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))

            # clear storage 
            self.group_duration_write.clearParam()                

        # Allocate goal position value into byte array
        param_goal_positions = {}
        if self.model_type == 350:
            for target in targets:
                param_goal_position = [DXL_LOBYTE(DXL_LOWORD(targets[target])), DXL_HIBYTE(DXL_LOWORD(targets[target]))]
                param_goal_positions[target] = param_goal_position

        elif self.model_type == 1200:
            for target in targets:
                param_goal_position = [DXL_LOBYTE(DXL_LOWORD(targets[target])), DXL_HIBYTE(DXL_LOWORD(targets[target])), DXL_LOBYTE(DXL_HIWORD(targets[target])), DXL_HIBYTE(DXL_HIWORD(targets[target]))]
                param_goal_positions[target] = param_goal_position

        # add each dynamixel goal position value to the Syncwrite storage
        for dxl_id in targets:
            dxl_addparam_result = self.group_goal_write.addParam(dxl_id, param_goal_positions[dxl_id])

            if dxl_addparam_result != True:
                logger.error("[ID:%d] group_goal_write addparam failed for position", dxl_id)
                quit()

        # log movement for sanity check 
        for dxl_id in targets:
            logger.debug("Moving ID %d to position %d", dxl_id, targets[dxl_id])

        # Syncwrite goal positions
        dxl_comm_result = self.group_goal_write.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            logger.error("%s", self.packet_handler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        self.group_goal_write.clearParam()

        # # spin until completion only if blocking is set to True in the config 
        if self.blocking:
            self.check_move_complete()

        self.check_motor_status(["all"])

        return 1
    
    def get_positions(self):
        ''' Get all positions with group_position_read. May replace check_motor_status.'''
        # Syncread present position
        dxl_comm_result = self.group_position_read.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            logger.info("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))
                
        positions = {}
        for dxl_id in self.dxl_ids:
            if self.model_type == 350:
                positions[dxl_id] = self.group_position_read.getData(dxl_id, self.ADDR_PRESENT_POSITION, CT_XL320_ADDR[self.ADDR_PRESENT_POSITION][1])
            elif self.model_type == 1200:
                positions[dxl_id] = self.group_position_read.getData(dxl_id, self.ADDR_PRESENT_POSITION, CT_XL330_ADDR[self.ADDR_PRESENT_POSITION][1])

        return positions 

    def check_move_complete(self):
        ''' Given a list of motors, gets the goal and present positions. '''
         # spin until completion
        while 1:
            goal_reached = 0
            time.sleep(0.1)

            # Syncread moving status
            dxl_comm_result = self.group_move_read.txRxPacket()
            if dxl_comm_result != COMM_SUCCESS:
                logger.error("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))
                continue

            for dxl_id in self.dxl_ids:
                # Check if groupsyncread data of motor is available
                if self.model_type == 350:
                    dxl_getdata_result = self.group_move_read.isAvailable(dxl_id, self.ADDR_MOVING, CT_XL320_ADDR[self.ADDR_MOVING][1])
                    if dxl_getdata_result != True:
                        logger.error("[ID:%03d] group_move_read getdata failed" % dxl_id)

                    # Get motor moving status
                    moving = self.group_move_read.getData(dxl_id, self.ADDR_MOVING, CT_XL320_ADDR[self.ADDR_MOVING][1])
                elif self.model_type == 1200:
                    dxl_getdata_result = self.group_move_read.isAvailable(dxl_id, self.ADDR_MOVING, CT_XL330_ADDR[self.ADDR_MOVING][1])
                    if dxl_getdata_result != True:
                        logger.error("[ID:%03d] group_move_read getdata failed" % dxl_id)

                    # Get motor moving status
                    moving = self.group_move_read.getData(dxl_id, self.ADDR_MOVING, CT_XL330_ADDR[self.ADDR_MOVING][1])

                if not moving:
                    goal_reached += 1
                
            if goal_reached == len(self.dxl_ids):
                break

        return 
    
    def get_motor_ids(self):
        """Returns the list of motor ids."""
        return self.dxl_ids


    def clean_shutdown(self):
        """Makes a clean shutdown of the motors."""
        logger.info("Initiating shutdown...")

        self.disable_torque()

        self.port_handler.closePort()

        logger.info("Shutdown complete.")