import numpy as np
import time
import threading
from scservo_sdk import *  # Uses FEETECH SCServo SDK library (sms_sts model)
import logging
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
ADDR_STS_PRESENT_LOAD = 60
ADDR_STS_MOVING_STATUS = 66
# Group-sync move-completion thresholds (see Servoset._set_position): a servo
# counts as "arrived" once it is within MOVING_POSITION_THRESHOLD encoder counts
# of its goal AND its present speed magnitude has dropped to <=
# MOVING_SPEED_THRESHOLD. Mirrors sync_read_write.py, which decides completion
# from a single position+speed sync-read rather than the moving-status register.
MOVING_POSITION_THRESHOLD = 20  # encoder counts
MOVING_SPEED_THRESHOLD = 50  # present-speed units (~0 = stopped); tune on hardware


class sts3032:

    def __init__(self, channel, packetHandler, lock=None):
        self.packetHandler = packetHandler
        # Serialises bus access so the ZMQ-server thread and the console can share
        # one serial port without their transactions interleaving (see Servoset).
        self.lock = lock or threading.RLock()
        self.SCS_ID = sts3032_dict[channel][0]
        # Last speed/acc pushed to the servo. The real values come from config
        # (SERVO_SPEED / SERVO_ACC) and are applied by Servoset.refresh() via
        # set_speed()/set_acc() -- the low-level sts3032 deliberately writes
        # nothing to hardware on construction. None == not pushed yet.
        self.SCS_MOVING_SPEED = None
        self.SCS_MOVING_ACC = None
        self.message = "Servo " + sts3032_dict[channel][1] + ": "

    def set_register(self, address, value, length=1):
        """Write `value` (1/2/4 bytes) to control-table `address` on this servo.

        Centralizes the write + comm/servo-error logging shared by every
        register-level setter. Returns (comm_result, error).
        """
        writer = {
            1: self.packetHandler.write1ByteTxRx,
            2: self.packetHandler.write2ByteTxRx,
            4: self.packetHandler.write4ByteTxRx,
        }[length]
        with self.lock:
            scs_comm_result, scs_error = writer(self.SCS_ID, address, value)
        if scs_comm_result != COMM_SUCCESS:
            logging.info(self.message + self.packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            logging.error(self.message + self.packetHandler.getRxPacketError(scs_error))
        return scs_comm_result, scs_error

    def read_register(self, address, length=1):
        """Read `length` bytes (1/2/4) from control-table `address` on this servo.

        Symmetric counterpart to set_register: runs the single read transaction
        under the bus lock and logs any comm/servo error. Returns
        (value, comm_result, error); value is 0 on a failed read.
        """
        reader = {
            1: self.packetHandler.read1ByteTxRx,
            2: self.packetHandler.read2ByteTxRx,
            4: self.packetHandler.read4ByteTxRx,
        }[length]
        with self.lock:
            value, scs_comm_result, scs_error = reader(self.SCS_ID, address)
        if scs_comm_result != COMM_SUCCESS:
            logging.info(self.message + self.packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            logging.error(self.message + self.packetHandler.getRxPacketError(scs_error))
        return value, scs_comm_result, scs_error

    def set_zero(self):
        # Writing 128 to the torque-enable register tells the servo to treat its
        # current shaft position as the new zero (FEETECH "set middle" command).
        # Hold the bus lock across the write + read-back so it stays one transaction.
        with self.lock:
            self.set_register(ADDR_STS_TORQUE_ENABLE, 128)
            try:
                scs_present_position_speed, scs_comm_result, scs_error = (
                    self.packetHandler.read4ByteTxRx(
                        self.SCS_ID, ADDR_STS_PRESENT_POSITION
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
        self.SCS_MOVING_ACC = set_acc
        self.set_register(ADDR_STS_GOAL_ACC, set_acc, length=1)
        logging.info(self.message + "SCServo acc set!")

    def set_speed(self, set_speed):
        self.SCS_MOVING_SPEED = set_speed
        self.set_register(ADDR_STS_GOAL_SPEED, set_speed, length=2)
        logging.info(self.message + "SCServo speed set!")

    def torque_disable(self):
        self.set_register(ADDR_STS_TORQUE_ENABLE, 0)

    def torque_enable(self):
        self.set_register(ADDR_STS_TORQUE_ENABLE, 1)


# A bigger Class that contains all the motors, should try to initialize the port connection as well in init of this class
class Servoset:
    def __init__(self, board_id=0, servo_channel_list=[]):
        self.board_id = board_id
        self.servo_channel_list = servo_channel_list
        self.timeout = 10
        self.de_hysterisis = DE_HYSTERESIS
        # How _set_position decides a move is finished -- see _move_finished():
        #   "position_speed"  : within MOVING_POSITION_THRESHOLD of goal AND
        #                       |speed| <= MOVING_SPEED_THRESHOLD (single sync-read)
        #   "moving_register" : servo Moving flag (register 66) == 0 for all servos
        #                       (authoritative, but an extra sync-read per poll)
        self.completion_mode = "position_speed"
        # One re-entrant lock guards every bus transaction (the per-servo register
        # writes and the group sync read/writes), so the ZMQ-server thread and the
        # interactive console can use this shared Servoset at the same time without
        # their packets interleaving on the wire. Each transaction is atomic; the
        # lock is NOT held across a whole multi-second move, so a live monitor read
        # still gets a turn between the move loop's reads.
        self.lock = threading.RLock()

        self.refresh()

    def refresh(self):
        self.connect()
        self.servo_list = []

        for channel in self.servo_channel_list:
            servo = sts3032(channel, self.packetHandler, self.lock)
            servo.set_acc(SERVO_ACC)
            servo.set_speed(SERVO_SPEED)
            # NB: torque is deliberately NOT enabled here -- refresh() must not
            # energise the motors as a side effect of connecting. Callers that
            # need holding torque (clip_scan / calibrate_jacobian / the ZMQ
            # server) enable it explicitly via torques_enable(); use get_torque()
            # to read the current per-servo state.
            self.servo_list.append(servo)

        self.SCS_ID_list = []
        for servo in self.servo_list:
            self.SCS_ID_list.append(servo.SCS_ID)
        # present speed and load per servo, refreshed alongside position by
        # get_position() (load = output-torque proxy, ~0.1% of max, signed)
        self.multi_speed_list = [0] * len(self.SCS_ID_list)
        self.multi_load_list = [0] * len(self.SCS_ID_list)
        # per-servo torque-enable state (read on demand by get_torque())
        self.multi_torque_list = [False] * len(self.SCS_ID_list)
        #
        self.file = Path(STATE_FOLDER) / "servos_{:s}.json".format(str(self.board_id))
        self.file.parent.mkdir(parents=True, exist_ok=True)
        self.load()
        #
        # Initialize GroupSyncRead instance for Present Position + Speed + Load
        # (6 contiguous bytes: position 56-57, speed 58-59, load 60-61 -- read
        # together in one transaction so the move loop needs a single round-trip,
        # not three). getData() only decodes 1/2/4-byte fields, so get_position()
        # extracts the position+speed word and the load word separately from the
        # same fetched buffer.
        self.groupSyncRead_position = GroupSyncRead(
            self.packetHandler, ADDR_STS_PRESENT_POSITION, 6
        )
        self.set_group_sync_read(self.groupSyncRead_position)
        self.groupSyncRead_status = GroupSyncRead(
            self.packetHandler, ADDR_STS_MOVING_STATUS, 1
        )
        self.set_group_sync_read(self.groupSyncRead_status)
        # 1-byte read of the torque-enable register (40) -- lets get_torque()
        # query every servo's on/off state in a single bus transaction.
        self.groupSyncRead_torque = GroupSyncRead(
            self.packetHandler, ADDR_STS_TORQUE_ENABLE, 1
        )
        self.set_group_sync_read(self.groupSyncRead_torque)
        self.groupSyncWrite_position = GroupSyncWrite(
            self.packetHandler, ADDR_STS_GOAL_POSITION, 2
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
                # sms_sts is the STS/SMS model handler; it subclasses
                # protocol_packet_handler and owns the port, so it doubles as
                self.packetHandler = sms_sts(self.portHandler)

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

    def set_torque(self, i, on):
        # Per-servo torque toggle (by channel index), keeping the cached
        # multi_torque_list in sync. The single-servo counterpart of the
        # mask-based torques_enable/torques_disable -- the monitor TUI uses this
        # instead of reaching into servo_list[i] directly, so every torque write
        # goes through Servoset and the cache stays authoritative.
        (self.servo_list[i].torque_enable if on else self.servo_list[i].torque_disable)()
        self.multi_torque_list[i] = bool(on)

    def torques_enable(self, mask=None):
        for i, servo in enumerate(self.servo_list):
            if mask is None or mask[i]:
                servo.torque_enable()
                self.multi_torque_list[i] = True

    def torques_disable(self, mask=None):
        for i, servo in enumerate(self.servo_list):
            if mask is None or mask[i]:
                servo.torque_disable()
                self.multi_torque_list[i] = False

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
        # Pre-read (one bus transaction), then extract a single field per servo.
        with self.lock:
            scs_comm_result = groupSyncRead.txRxPacket()
            if scs_comm_result != COMM_SUCCESS:
                logging.info("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
            return self._sync_extract(groupSyncRead, start_address, data_length)

    def _sync_extract(self, groupSyncRead, start_address, data_length):
        # Extract one (start_address, data_length) field per servo from data
        # already fetched by groupSyncRead.txRxPacket() -- no new bus
        # transaction. Lets one wide read feed several fields (e.g. position and
        # load out of the same 6-byte fetch). Missing data -> 0.
        datas = []
        for SCS_ID in self.SCS_ID_list:
            # New SDK: isAvailable returns (available, error) instead of a bool.
            scs_getdata_result, _ = groupSyncRead.isAvailable(
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
        with self.lock:
            index = 0
            for SCS_ID in self.SCS_ID_list:
                param_goal_position = [
                    self.packetHandler.scs_lobyte(datas[index]),
                    self.packetHandler.scs_hibyte(datas[index]),
                ]
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
        # One 6-byte sync-read yields present position (56-57), present speed
        # (58-59) and present load (60-61) together. txRxPacket() runs the single
        # bus transaction; the position+speed word and the load word are then
        # extracted from the same fetched buffer (getData only decodes 1/2/4-byte
        # fields, so they cannot come out as one 6-byte value).
        with self.lock:
            scs_comm_result = self.groupSyncRead_position.txRxPacket()
            if scs_comm_result != COMM_SUCCESS:
                logging.info("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
            ph = self.packetHandler
            # Position (low word, 56-57) + speed (high word, 58-59): both FEETECH
            # bit-15 sign-magnitude, decoded with scs_tohost(..., 15) into signed
            # values -- the inverse of the goal-position encoding in _set_position.
            # (Without the decode, a negative multi-turn position with bit 15 set
            # would read as ~32768+ and corrupt the angle. cf. sync_read_write.py.)
            pos_speed_list = self._sync_extract(
                self.groupSyncRead_position, ADDR_STS_PRESENT_POSITION, 4
            )
            scs_present_position_list = [
                ph.scs_tohost(ph.scs_loword(d), 15) for d in pos_speed_list
            ]
            self.multi_speed_list = [
                ph.scs_tohost(ph.scs_hiword(d), 15) for d in pos_speed_list
            ]
            # Present load (60-61): output-torque proxy, ~0.1% of max torque. Unlike
            # position/speed it is bit-10 sign-magnitude (bit 10 = direction), so it
            # decodes with scs_tohost(..., 10).
            load_list = self._sync_extract(
                self.groupSyncRead_position, ADDR_STS_PRESENT_LOAD, 2
            )
            self.multi_load_list = [ph.scs_tohost(d, 10) for d in load_list]
            self.multi_position_list = list(scs_present_position_list)
            self.save()
            return scs_present_position_list

    def get_angle(self):
        position_list = self.get_position()
        return self.position_to_angle(position_list)

    def get_load(self):
        # Present load per servo (signed; magnitude ~0.1% of max torque, sign =
        # drive direction). Refreshed together with position/speed in the single
        # get_position() sync-read, so reading torque costs no extra round-trip.
        self.get_position()
        return list(self.multi_load_list)

    def get_torque(self):
        # Query the torque-enable register (40) of every servo in one sync-read
        # and return a list of bools (True = torque on). Cached in
        # self.multi_torque_list. Unlike torques_enable(), this only reads --
        # it never energises a motor.
        torque_raw = self.group_sync_read(
            self.groupSyncRead_torque, ADDR_STS_TORQUE_ENABLE, 1
        )
        self.multi_torque_list = [bool(t) for t in torque_raw]
        return list(self.multi_torque_list)

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

    def _move_finished(self, goal_position_list):
        # Decide whether every servo has finished moving, per self.completion_mode.
        if self.completion_mode == "moving_register":
            # Servo's own Moving flag (register 66): 0 == stopped. Authoritative,
            # but costs one extra sync-read per poll.
            return sum(self.moving_status()) == 0
        # "position_speed" (default): within MOVING_POSITION_THRESHOLD of goal AND
        # slowed to <= MOVING_SPEED_THRESHOLD. Requiring both avoids a false "done"
        # at move start (far from goal, speed still 0) and during overshoot (near
        # goal but still moving fast). Uses the position+speed already read by
        # get_position(), so no extra round-trip.
        return all(
            abs(goal_position_list[index] - self.multi_position_list[index])
            <= MOVING_POSITION_THRESHOLD
            and abs(self.multi_speed_list[index]) <= MOVING_SPEED_THRESHOLD
            for index in range(len(self.SCS_ID_list))
        )

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

            # Refresh position (and speed) and persist; one sync-read. In
            # "moving_register" mode _move_finished() does one more read.
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

            if iteration % 100 == 0:
                logging.debug(status_string)

            all_stop_moving = self._move_finished(goal_position_list)
            if all_stop_moving:
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
