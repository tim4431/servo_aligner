import os
import numpy as np
import matplotlib.pyplot as plt
import time
from smbus2 import SMBus, i2c_msg
from scservo_sdk import *  # Uses SCServo SDK library
import MCP342x
import logging
from copy import deepcopy

import json
from pathlib import Path
import atexit
from config import (
    DEVICENAME_LIST,
    BAUDRATE,
    SERVO_SPEED,
    SERVO_ACC,
    STATE_FOLDER,
    sts3032_dict,
    SERVO_CHANNEL_LIST,
    SERVER,
    DE_HYSTERESIS,
    DEHYS_OVERSHOOT,
    DEHYS_THRESHOLD,
)
from datetime import datetime
from servo_util import ENCODER_CENTER, COUNTS_PER_TURN, DEGREES_PER_TURN

# Control table address
ADDR_STS_TORQUE_ENABLE = 40
ADDR_STS_GOAL_ACC = 41
ADDR_STS_GOAL_POSITION = 42
ADDR_STS_GOAL_SPEED = 46
ADDR_STS_PRESENT_POSITION = 56
ADDR_STS_MOVING_STATUS = 66

# Default setting
# dmesg | grep tty
protocol_end = 0  # SCServo bit end(STS/SMS=0, SCS=1)


class sts3032:

    def __init__(self, channel, portHandler, packetHandler):
        self.portHandler = portHandler
        self.packetHandler = packetHandler
        self.SCS_ID = sts3032_dict[channel][0]
        self.SCS_MOVING_SPEED = 1500  # SCServo moving speed
        self.SCS_MOVING_ACC = 50  # SCServo moving acc
        self.message = "Servo " + sts3032_dict[channel][1] + ": "
        # atexit.register(self.home)
        self.set_acc(self.SCS_MOVING_ACC)
        self.set_speed(self.SCS_MOVING_SPEED)

    def set_zero(self):
        # Set Zeros
        scs_comm_result, scs_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, self.SCS_ID, ADDR_STS_TORQUE_ENABLE, 128
        )
        if scs_comm_result != COMM_SUCCESS:
            logging.info("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            logging.error("%s" % self.packetHandler.getRxPacketError(scs_error))
        try:
            scs_present_position_speed, scs_comm_result, scs_error = (
                self.packetHandler.read4ByteTxRx(
                    self.portHandler, self.SCS_ID, ADDR_STS_PRESENT_POSITION
                )
            )
            if scs_comm_result != COMM_SUCCESS:
                logging.info(
                    self.message
                    + "result "
                    + str(self.packetHandler.getTxRxResult(scs_comm_result))
                )
            elif scs_error != 0:
                logging.error(
                    self.message
                    + "error "
                    + str(self.packetHandler.getRxPacketError(scs_error))
                )
            logging.info(self.message + "SCServo zero set!")
            return 0
        except:
            logging.error(self.message + "Read not sucessful!")
            return 1

    def set_acc(self, set_acc):
        # Write SCServo acc
        self.SCS_MOVING_ACC = set_acc
        scs_comm_result, scs_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, self.SCS_ID, ADDR_STS_GOAL_ACC, self.SCS_MOVING_ACC
        )
        if scs_comm_result != COMM_SUCCESS:
            logging.info("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            logging.error("%s" % self.packetHandler.getRxPacketError(scs_error))

        logging.info(self.message + "SCServo acc set!")

    def set_speed(self, set_speed):
        # Write SCServo speed
        self.SCS_MOVING_SPEED = set_speed
        scs_comm_result, scs_error = self.packetHandler.write2ByteTxRx(
            self.portHandler, self.SCS_ID, ADDR_STS_GOAL_SPEED, self.SCS_MOVING_SPEED
        )
        if scs_comm_result != COMM_SUCCESS:
            logging.info(
                self.message + "%s" % self.packetHandler.getTxRxResult(scs_comm_result)
            )
        elif scs_error != 0:
            logging.error(
                self.message + "%s" % self.packetHandler.getRxPacketError(scs_error)
            )
        logging.info(self.message + "SCServo speed set!")

    def set_position(self, goal_position):

        if goal_position >= 0:
            val = 0b0000000000000000 | abs(goal_position)
            scs_goal_position = val
        elif goal_position < 0:
            val = 0b1000000000000000 | abs(goal_position)
            scs_goal_position = val

        # pre-read
        scs_present_position_speed, scs_comm_result, scs_error = (
            self.packetHandler.read4ByteTxRx(
                self.portHandler, self.SCS_ID, ADDR_STS_PRESENT_POSITION
            )
        )
        if scs_comm_result != COMM_SUCCESS:
            logging.info("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            logging.error("%s" % self.packetHandler.getRxPacketError(scs_error))
        self.raw_angle_current = SCS_LOWORD(scs_present_position_speed)

        # Write SCServo goal position
        scs_comm_result, scs_error = self.packetHandler.write2ByteTxRx(
            self.portHandler, self.SCS_ID, ADDR_STS_GOAL_POSITION, scs_goal_position
        )
        if scs_comm_result != COMM_SUCCESS:
            logging.info("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            logging.error("%s" % self.packetHandler.getRxPacketError(scs_error))

        i = 0
        while i < 5000:
            # Read SCServo present position
            scs_present_position_speed, scs_comm_result, scs_error = (
                self.packetHandler.read4ByteTxRx(
                    self.portHandler, self.SCS_ID, ADDR_STS_PRESENT_POSITION
                )
            )
            if scs_comm_result != COMM_SUCCESS:
                logging.info("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
            elif scs_error != 0:
                logging.error("%s" % self.packetHandler.getRxPacketError(scs_error))
            # Read SCServo present status
            scs_present_status, scs_comm_result, scs_error = (
                self.packetHandler.read4ByteTxRx(
                    self.portHandler, self.SCS_ID, ADDR_STS_MOVING_STATUS
                )
            )
            if scs_comm_result != COMM_SUCCESS:
                logging.info("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
            elif scs_error != 0:
                logging.error("%s" % self.packetHandler.getRxPacketError(scs_error))
            # Present position is read straight from the servo; hardware
            # multi-turn (register 18 -> 124) is authoritative, no software
            # turn counting.
            scs_present_position = SCS_LOWORD(scs_present_position_speed)

            scs_present_status = scs_present_status & 0x0001
            if i % 10 == 0:
                print(
                    i,
                    scs_present_status,
                    scs_present_position,
                    goal_position,
                )
            i = i + 1

            if (abs(goal_position - scs_present_position) == 0) and (
                scs_present_status == 0
            ):
                print(
                    i,
                    scs_present_status,
                    scs_present_position,
                    goal_position,
                )
                break

    def home(self):
        self.set_position(ENCODER_CENTER)

    def torque_disable(self):
        scs_comm_result, scs_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, self.SCS_ID, ADDR_STS_TORQUE_ENABLE, 0
        )
        if scs_comm_result != COMM_SUCCESS:
            logging.info("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            logging.error("%s" % self.packetHandler.getRxPacketError(scs_error))

    def torque_enable(self):
        scs_comm_result, scs_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, self.SCS_ID, ADDR_STS_TORQUE_ENABLE, 1
        )
        if scs_comm_result != COMM_SUCCESS:
            logging.info("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            logging.error("%s" % self.packetHandler.getRxPacketError(scs_error))


# A bigger Class that contains all the motors, should try to initialize the port connection as well in init of this class
class Servoset:
    def __init__(self, board_id=0, servo_channel_list=[]):
        self.board_id = board_id
        self.servo_channel_list = servo_channel_list
        self.timeout = 10
        self.de_hysterisis = DE_HYSTERESIS

        self.refresh()

    def refresh(self):
        self.connect()
        self.servo_list = []

        for channel in self.servo_channel_list:
            servo = sts3032(channel, self.portHandler, self.packetHandler)
            servo.set_acc(SERVO_ACC)
            servo.set_speed(SERVO_SPEED)
            servo.torque_enable()
            self.servo_list.append(servo)

        self.SCS_ID_list = []
        for servo in self.servo_list:
            self.SCS_ID_list.append(servo.SCS_ID)
        #
        self.file = Path(STATE_FOLDER) / "servos_{:s}.json".format(str(self.board_id))
        self.file.parent.mkdir(parents=True, exist_ok=True)
        self.load()
        #
        # Initialize GroupSyncRead instace for Present Position
        self.groupSyncRead_position = GroupSyncRead(
            self.portHandler, self.packetHandler, ADDR_STS_PRESENT_POSITION, 2
        )
        self.set_group_sync_read(self.groupSyncRead_position)
        self.groupSyncRead_status = GroupSyncRead(
            self.portHandler, self.packetHandler, ADDR_STS_MOVING_STATUS, 1
        )
        self.set_group_sync_read(self.groupSyncRead_status)
        self.groupSyncWrite_position = GroupSyncWrite(
            self.portHandler, self.packetHandler, ADDR_STS_GOAL_POSITION, 2
        )

        # make sure the current position gets saved to disk when the programm exits
        atexit.register(self.save)

    def set_timeout(self, timeout):
        self.timeout = timeout

    def connect(self):
        for DEVICENAME in DEVICENAME_LIST:
            try:
                self.portHandler = PortHandler(DEVICENAME)
                self.portHandler.setPacketTimeoutMillis(100)
                # Initialize self.packetHandler instance
                # Get methods and members of Protocol
                self.packetHandler = PacketHandler(protocol_end)

                # Open port
                try:
                    if self.portHandler.openPort():
                        logging.info("Succeeded to open the port")
                    else:
                        logging.error("Failed to open the port")
                        logging.error("Press any key to terminate...")
                        getch()  # type: ignore
                        quit()
                except Exception as e:
                    print(e)
                # Set port baudrate
                if self.portHandler.setBaudRate(BAUDRATE):
                    logging.info("Succeeded to change the baudrate")
                else:
                    logging.error("Failed to change the baudrate")
                    logging.error("Press any key to terminate...")
                    getch()  # type: ignore
                    quit()

                # if success
                break
            except Exception:
                logging.info(f"The device {DEVICENAME} is not available, try next ...")
        else:
            raise Exception("None of the device is working!")

    def save(self):
        # Persist last-known encoder positions across restarts.
        # `position` is authoritative; the rest is metadata for humans and for a
        # stale-state sanity check on load.
        positions = [int(p) for p in self.multi_position_list]
        dct = {
            "board_id": self.board_id,
            "saved_at": datetime.now().isoformat(timespec="seconds"),
            "servo_ids": list(self.SCS_ID_list),
            "position": positions,
            "angles_deg": [
                round(float(a), 4) for a in self.position_to_angle(positions)
            ],
        }
        self.file.write_text(json.dumps(dct, indent=2))

    def load(self):
        if self.file.exists():  # load existing data
            logging.info("loading position from disk")
            try:
                # load position from file
                dct = json.loads(self.file.read_text())
                positions = dct["position"]
                # Stale-state guard: the channel map changed since this was saved.
                if len(positions) != len(self.SCS_ID_list):
                    logging.warning(
                        "Saved state has %d positions but %d servos are connected; "
                        "ignoring stale file and centering.",
                        len(positions),
                        len(self.SCS_ID_list),
                    )
                    self.multi_position_list = list(
                        np.ones(len(self.SCS_ID_list)) * ENCODER_CENTER
                    )
                    self.save()
                    return
                saved_ids = dct.get("servo_ids")
                if saved_ids is not None and list(saved_ids) != list(self.SCS_ID_list):
                    logging.warning(
                        "Saved servo IDs %s differ from connected %s; positions may be stale.",
                        saved_ids,
                        self.SCS_ID_list,
                    )
                self.multi_position_list = positions
                message = "Loaded, the positions are: \n"
                for i in range(len(self.multi_position_list)):
                    message += self.servo_list[i].message
                    message += (
                        str(
                            (self.multi_position_list[i] - ENCODER_CENTER)
                            * DEGREES_PER_TURN
                            / COUNTS_PER_TURN
                        )
                        + " deg\t"
                    )
                logging.info(message)
            except Exception as e:
                logging.error(
                    f"Error loading position from disk: {e}; centering instead."
                )
                self.multi_position_list = list(
                    np.ones(len(self.SCS_ID_list)) * ENCODER_CENTER
                )
                self.save()
        else:
            logging.info("No position data found on disk")
            self.multi_position_list = list(
                np.ones(len(self.SCS_ID_list)) * ENCODER_CENTER
            )
            # create new file
            self.save()

    # def __del__(self):
    #     # also save when object is destroyed
    #     self.save()

    def set_zero(self):
        logging.info("Setting Zero")
        for servo in self.servo_list:
            iteration = 1
            while 1:
                logging.debug("Set Zero Trail " + str(iteration))
                result = servo.set_zero()
                if result == 0:
                    break
                iteration += 1
        self.multi_position_list = [ENCODER_CENTER] * len(self.SCS_ID_list)
        self.save()

    def torques_enable(self, mask=None):
        for i, servo in enumerate(self.servo_list):
            if mask is None or mask[i]:
                servo.torque_enable()

    def torques_disable(self, mask=None):
        for i, servo in enumerate(self.servo_list):
            if mask is None or mask[i]:
                servo.torque_disable()

    def home(self):
        logging.info("going home")
        goal_position_list = np.ones(len(self.SCS_ID_list)) * ENCODER_CENTER
        self.set_position(goal_position_list)

    def set_group_sync_read(self, groupSyncRead):
        # Add parameter
        for SCS_ID in self.SCS_ID_list:
            scs_addparam_result = groupSyncRead.addParam(SCS_ID)
            if scs_addparam_result != True:
                logging.error("[ID:%03d] groupSyncRead addparam failed" % SCS_ID)
                quit()
        return groupSyncRead

    def group_sync_read(self, groupSyncRead, start_address, data_length):
        # Pre-read
        scs_comm_result = groupSyncRead.txRxPacket()
        if scs_comm_result != COMM_SUCCESS:
            logging.info("%s" % self.packetHandler.getTxRxResult(scs_comm_result))

        datas = []

        for SCS_ID in self.SCS_ID_list:
            scs_getdata_result = groupSyncRead.isAvailable(
                SCS_ID, start_address, data_length
            )
            if scs_getdata_result == True:
                datas.append(groupSyncRead.getData(SCS_ID, start_address, data_length))
            else:
                logging.error("[ID:%03d] groupSyncRead getdata failed" % SCS_ID)
                datas.append(0)

        return datas

    def group_sync_write_2byte(self, groupSyncWrite, datas):
        # Allocate goal position value into byte array
        # Add SCServo goal position values to the Syncwrite parameter storage
        index = 0
        for SCS_ID in self.SCS_ID_list:
            param_goal_position = [SCS_LOBYTE(datas[index]), SCS_HIBYTE(datas[index])]
            scs_addparam_result = groupSyncWrite.addParam(SCS_ID, param_goal_position)
            index += 1
            if scs_addparam_result != True:
                logging.error("[ID:%03d] groupSyncWrite addparam failed" % SCS_ID)
                quit()

        # Syncwrite goal position
        scs_comm_result = groupSyncWrite.txPacket()
        if scs_comm_result != COMM_SUCCESS:
            logging.info("%s" % self.packetHandler.getTxRxResult(scs_comm_result))

    def get_position(self):
        scs_present_position_list = self.group_sync_read(
            self.groupSyncRead_position, ADDR_STS_PRESENT_POSITION, 2
        )
        self.multi_position_list = list(scs_present_position_list)
        self.save()
        return scs_present_position_list

    def get_angle(self):
        position_list = self.get_position()
        return self.position_to_angle(position_list)

    def moving_status(self):
        sts_moving_status = self.group_sync_read(
            self.groupSyncRead_status, ADDR_STS_MOVING_STATUS, 1
        )
        return sts_moving_status

    def set_position(self, goal_position_list, pos_mask=None):
        self.get_position()
        #
        if len(goal_position_list) != len(self.SCS_ID_list):
            logging.error(
                "Goal position list length {} does not match the number of motors {}".format(
                    len(goal_position_list), len(self.SCS_ID_list)
                )
            )
            return
        if pos_mask is not None:
            goal_position_list = [
                goal_position_list[i] if pos_mask[i] else self.multi_position_list[i]
                for i in range(len(goal_position_list))
            ]
        goal_position_list = [
            int(goal_position) for goal_position in list(goal_position_list)
        ]
        #
        POS_THRESHOLD = DEHYS_THRESHOLD
        # always go to plus direction, if goes to negative, then first go to more negative, then go to positive
        de_hysterisis_mask = [0] * len(self.SCS_ID_list)
        for i in range(len(self.SCS_ID_list)):
            d_pos = goal_position_list[i] - self.multi_position_list[i]
            if (d_pos < 0) and (abs(d_pos) > POS_THRESHOLD):
                de_hysterisis_mask[i] = 1
        #
        if (not self.de_hysterisis) or (np.sum(de_hysterisis_mask) == 0):
            self._set_position(goal_position_list)
        else:
            # print(de_hysterisis_mask,goal_position_list)
            goal_position_list_deh = [
                x - DEHYS_OVERSHOOT if de_hysterisis_mask[i] else x
                for i, x in enumerate(goal_position_list)
            ]
            logging.debug("De-hysterisis On")
            logging.debug(
                "First go to {} and then go to {}".format(
                    goal_position_list_deh, goal_position_list
                )
            )
            self._set_position(goal_position_list_deh)
            self._set_position(goal_position_list)

    def _set_position(self, goal_position_list):
        scs_goal_position = []
        for goal_position in goal_position_list:
            if goal_position >= 0:
                val = 0b0000000000000000 | abs(goal_position)
                scs_goal_position.append(val)
            elif goal_position < 0:
                val = 0b1000000000000000 | abs(goal_position)
                scs_goal_position.append(val)

        # Initialize GroupSyncWrite instance
        self.group_sync_write_2byte(self.groupSyncWrite_position, scs_goal_position)
        self.groupSyncWrite_position.clearParam()

        # starting position
        status_string = "Start Position: " + "\t"
        self.multi_position_list = self.get_position()
        for index in range(len(self.SCS_ID_list)):
            status_string += (
                "[ID:"
                + f"{self.SCS_ID_list[index]:03d}"
                + "] Goal:"
                + f"{goal_position_list[index]:03d}"
                + " Pres:"
                + f"{self.multi_position_list[index]:03d}"
                + "\t"
            )

        logging.debug(status_string)

        t0 = time.time()
        all_stop_moving = False
        iteration = 0
        while (time.time() - t0) < self.timeout and (not all_stop_moving):
            status_string = "Iteration: " + str(iteration) + "\t"

            self.multi_position_list = self.get_position()
            sts_moving_status = self.moving_status()

            for index in range(len(self.SCS_ID_list)):
                status_string += (
                    "[ID:"
                    + f"{self.SCS_ID_list[index]:03d}"
                    + "] Goal:"
                    + f"{goal_position_list[index]:03d}"
                    + " Pres:"
                    + f"{self.multi_position_list[index]:03d}"
                    + "\t"
                )

            if iteration % 100 == 0:
                logging.debug(status_string)
                # logging.debug(sts_moving_status)

            # count how many motors have finished moving
            if sum(sts_moving_status) == 0:
                all_stop_moving = True
                logging.debug(status_string)
                break
            iteration += 1

        self.get_position()
        # Clear syncread parameter storage
        # self.groupSyncRead_position.clearParam()

    def angle_to_position(self, angle):
        angle = np.array(angle)
        return (angle * (COUNTS_PER_TURN / DEGREES_PER_TURN) + ENCODER_CENTER).astype(
            int
        )

    def position_to_angle(self, position):
        position = np.array(position)
        return (
            (position - ENCODER_CENTER) * DEGREES_PER_TURN / COUNTS_PER_TURN
        ).astype(float)

    def set_angle(self, goal_angle_list, pos_mask=None):
        goal_position_list = self.angle_to_position(goal_angle_list)
        self.set_position(goal_position_list, pos_mask=pos_mask)

    def set_single(self, index, angle):
        pos_mask = [0 for i in range(len(self.servo_list))]
        pos_mask[index] = 1
        angle_list = [0 for i in range(len(self.servo_list))]
        angle_list[index] = angle
        self.set_angle(angle_list, pos_mask)

    def random_play(self):
        # self.set_precision(10)
        TIME_TO_WAIT = 8
        print("Random play starting in {} seconds".format(TIME_TO_WAIT))
        time.sleep(TIME_TO_WAIT)
        print("Random play starting")
        for i in range(len(self.servo_list)):
            print("Servo ", i)
            self.set_single(i, 30)
            self.set_single(i, 0)
            time.sleep(1)

    def close(self):
        # Close port
        self.portHandler.closePort()


if __name__ == "__main__":
    servos = Servoset(SERVER["board_id"], SERVO_CHANNEL_LIST)
    servos.random_play()
    # servos.set_angle([50,-50])
    # servos.set_angle([30])
    # servos.set_angle([0])
    # servos.home()
    # print(servos.get_angle())
