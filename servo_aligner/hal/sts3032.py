"""FEETECH STS3032 backend: serial bus, single servo, and the actuator.

Split of the legacy ``servodriver.py`` into three layers:

- :class:`ScsBus` — owns the serial port and encapsulates every
  ``scservo_sdk`` call (so tests can substitute a fake bus).
- :class:`Sts3032Servo` — one servo: registers, torque, speed/acc, and the
  software multi-turn tracking of the single-servo move path.
- :class:`Sts3032Actuator` — the channel set: group sync moves, the
  de-hysteresis sequence, angle conversion, and position persistence.

The motion numerics (de-hysteresis overshoot, multi-turn wraparound
threshold, sign-magnitude goal encoding, polling loops) are lab-validated
legacy behavior — do not change them.

``scservo_sdk`` (and therefore pyserial) is imported lazily inside
:class:`ScsBus`, so this module is importable on machines without serial
support.
"""

from __future__ import annotations

import atexit
import logging
import time
from pathlib import Path
from typing import List, Optional, Sequence

import numpy as np

from ..config import ActuatorConfig
from ..vectors import COUNTS_PER_TURN, ZERO_COUNT
from .persistence import PositionStore

logger = logging.getLogger(__name__)

# Control table addresses (STS3032)
ADDR_STS_TORQUE_ENABLE = 40
ADDR_STS_GOAL_ACC = 41
ADDR_STS_GOAL_POSITION = 42
ADDR_STS_GOAL_SPEED = 46
ADDR_STS_PRESENT_POSITION = 56
ADDR_STS_MOVING_STATUS = 66

#: SCServo bit end (STS/SMS = 0, SCS = 1)
PROTOCOL_END = 0

COMM_SUCCESS = 0

#: A position jump larger than this is interpreted as an encoder wraparound.
WRAPAROUND_THRESHOLD = 3500


# byte macros for PROTOCOL_END = 0 (see scservo_sdk/scservo_def.py)
def _loword(value: int) -> int:
    return value & 0xFFFF


def _lobyte(word: int) -> int:
    return word & 0xFF


def _hibyte(word: int) -> int:
    return (word >> 8) & 0xFF


def _encode_goal(goal_position: int) -> int:
    """Sign-magnitude encoding of a goal position (bit 15 = direction)."""
    if goal_position >= 0:
        return abs(goal_position)
    return 0b1000000000000000 | abs(goal_position)


class ScsBus:
    """Serial connection to the daisy-chained servos.

    Tries each device in ``ports`` until one opens at ``baudrate``.
    All ``scservo_sdk`` usage lives here.
    """

    def __init__(
        self,
        ports: Sequence[str],
        baudrate: int,
        packet_timeout_ms: int = 100,
        auto_connect: bool = True,
    ):
        self.ports = tuple(ports)
        self.baudrate = baudrate
        self.packet_timeout_ms = packet_timeout_ms
        self.port = None
        self.packet = None
        self._sync_readers = {}
        self._sync_writers = {}
        if auto_connect:
            self.connect()

    def connect(self) -> None:
        from ..scservo_sdk import PacketHandler, PortHandler

        for device in self.ports:
            try:
                port = PortHandler(device)
                port.setPacketTimeoutMillis(self.packet_timeout_ms)
                packet = PacketHandler(PROTOCOL_END)
                if not port.openPort():
                    raise ConnectionError(f"failed to open the port {device}")
                if not port.setBaudRate(self.baudrate):
                    port.closePort()
                    raise ConnectionError(
                        f"failed to set baudrate {self.baudrate} on {device}"
                    )
                self.port = port
                self.packet = packet
                logger.info("Connected to %s at %d baud", device, self.baudrate)
                return
            except Exception:
                logger.info("The device %s is not available, try next ...", device)
        raise ConnectionError(
            f"none of the serial devices {list(self.ports)} is working"
        )

    def close(self) -> None:
        if self.port is not None:
            self.port.closePort()

    # ------------------------------------------------------- single-servo IO

    def _check(self, comm_result: int, error: int) -> None:
        if comm_result != COMM_SUCCESS:
            logger.info("%s", self.packet.getTxRxResult(comm_result))
        elif error != 0:
            logger.error("%s", self.packet.getRxPacketError(error))

    def write1(self, scs_id: int, addr: int, value: int) -> None:
        comm, err = self.packet.write1ByteTxRx(self.port, scs_id, addr, value)
        self._check(comm, err)

    def write2(self, scs_id: int, addr: int, value: int) -> None:
        comm, err = self.packet.write2ByteTxRx(self.port, scs_id, addr, value)
        self._check(comm, err)

    def read4(self, scs_id: int, addr: int) -> int:
        value, comm, err = self.packet.read4ByteTxRx(self.port, scs_id, addr)
        self._check(comm, err)
        return value

    # -------------------------------------------------------------- group IO

    def sync_read(self, ids: Sequence[int], addr: int, length: int) -> List[int]:
        from ..scservo_sdk import GroupSyncRead

        key = (addr, length, tuple(ids))
        reader = self._sync_readers.get(key)
        if reader is None:
            reader = GroupSyncRead(self.port, self.packet, addr, length)
            for scs_id in ids:
                if reader.addParam(scs_id) is not True:
                    raise RuntimeError(
                        f"[ID:{scs_id:03d}] groupSyncRead addparam failed"
                    )
            self._sync_readers[key] = reader

        comm = reader.txRxPacket()
        if comm != COMM_SUCCESS:
            logger.info("%s", self.packet.getTxRxResult(comm))

        datas = []
        for scs_id in ids:
            if reader.isAvailable(scs_id, addr, length) is True:
                datas.append(reader.getData(scs_id, addr, length))
            else:
                logger.error("[ID:%03d] groupSyncRead getdata failed", scs_id)
                datas.append(0)
        return datas

    def sync_write_2byte(
        self, ids: Sequence[int], addr: int, values: Sequence[int]
    ) -> None:
        from ..scservo_sdk import GroupSyncWrite

        writer = self._sync_writers.get(addr)
        if writer is None:
            writer = GroupSyncWrite(self.port, self.packet, addr, 2)
            self._sync_writers[addr] = writer

        for scs_id, value in zip(ids, values):
            if writer.addParam(scs_id, [_lobyte(value), _hibyte(value)]) is not True:
                raise RuntimeError(f"[ID:{scs_id:03d}] groupSyncWrite addparam failed")
        comm = writer.txPacket()
        if comm != COMM_SUCCESS:
            logger.info("%s", self.packet.getTxRxResult(comm))
        writer.clearParam()


class Sts3032Servo:
    """A single STS3032 servo on the bus."""

    def __init__(self, bus: ScsBus, servo_id: int, label: str = ""):
        self.bus = bus
        self.id = servo_id
        self.label = label
        self.turn_num = 0
        self.raw_angle_current = 0
        self.angle_current = ZERO_COUNT
        self.message = f"Servo {label}: "

    def set_zero(self) -> int:
        """Define the current pose as encoder 2048 (torque register 128)."""
        self.bus.write1(self.id, ADDR_STS_TORQUE_ENABLE, 128)
        self.turn_num = 0
        self.angle_current = ZERO_COUNT
        try:
            self.bus.read4(self.id, ADDR_STS_PRESENT_POSITION)
            logger.info("%sSCServo zero set!", self.message)
            return 0
        except Exception:
            logger.error("%sRead not successful!", self.message)
            return 1

    def set_acc(self, acc: int) -> None:
        self.acc = acc
        self.bus.write1(self.id, ADDR_STS_GOAL_ACC, acc)
        logger.info("%sSCServo acc set!", self.message)

    def set_speed(self, speed: int) -> None:
        self.speed = speed
        self.bus.write2(self.id, ADDR_STS_GOAL_SPEED, speed)
        logger.info("%sSCServo speed set!", self.message)

    def torque_enable(self) -> None:
        self.bus.write1(self.id, ADDR_STS_TORQUE_ENABLE, 1)

    def torque_disable(self) -> None:
        self.bus.write1(self.id, ADDR_STS_TORQUE_ENABLE, 0)

    def set_position(self, goal_position: int) -> None:
        """Move this servo, tracking multi-turn position in software.

        The encoder reports 0–4095 per turn; a jump larger than
        ``WRAPAROUND_THRESHOLD`` counts is interpreted as a wraparound and
        accumulated into ``turn_num`` so ``angle_current`` is continuous.
        """
        scs_goal_position = _encode_goal(goal_position)

        # pre-read
        present = self.bus.read4(self.id, ADDR_STS_PRESENT_POSITION)
        self.raw_angle_current = _loword(present)

        self.bus.write2(self.id, ADDR_STS_GOAL_POSITION, scs_goal_position)

        i = 0
        while i < 5000:
            present = self.bus.read4(self.id, ADDR_STS_PRESENT_POSITION)
            status = self.bus.read4(self.id, ADDR_STS_MOVING_STATUS)

            scs_present_position = _loword(present)
            if abs(scs_present_position - self.raw_angle_current) > WRAPAROUND_THRESHOLD:
                self.turn_num = self.turn_num + int(
                    np.sign(self.raw_angle_current - scs_present_position)
                )
            self.raw_angle_current = scs_present_position
            self.angle_current = scs_present_position + COUNTS_PER_TURN * self.turn_num

            status = status & 0x0001
            if i % 10 == 0:
                logger.debug(
                    "%d %d %d %d %d",
                    i, status, self.raw_angle_current, self.angle_current, goal_position,
                )
            i = i + 1

            if (abs(goal_position - self.angle_current) == 0) and (status == 0):
                logger.debug(
                    "%d %d %d %d %d",
                    i, status, self.raw_angle_current, self.angle_current, goal_position,
                )
                break

    def home(self) -> None:
        self.set_position(ZERO_COUNT)


class Sts3032Actuator:
    """The full servo set behind the :class:`~..interfaces.Actuator` protocol."""

    def __init__(
        self,
        cfg: ActuatorConfig,
        state_dir: Path,
        bus: Optional[ScsBus] = None,
    ):
        self.cfg = cfg
        self.de_hysteresis = cfg.de_hysteresis.enabled
        self.timeout = cfg.move_timeout_s
        self.bus = bus if bus is not None else ScsBus(cfg.ports, cfg.baudrate)

        self.servos = [
            Sts3032Servo(self.bus, ch.servo_id, ch.label or ch.name)
            for ch in cfg.channels
        ]
        for servo in self.servos:
            servo.set_acc(cfg.acc)
            servo.set_speed(cfg.speed)
            servo.torque_enable()
        self.ids = [servo.id for servo in self.servos]

        self.store = PositionStore(
            Path(state_dir) / f"servos_{cfg.board_id}.json"
        )
        self._load_positions()
        # make sure the current position gets saved when the program exits
        atexit.register(self.save)

    # ------------------------------------------------------------ protocol

    @property
    def n_channels(self) -> int:
        return len(self.servos)

    @property
    def channel_names(self):
        return tuple(ch.name for ch in self.cfg.channels)

    def __enter__(self) -> "Sts3032Actuator":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def close(self) -> None:
        self.save()
        atexit.unregister(self.save)
        self.bus.close()

    # -------------------------------------------------------- persistence

    def save(self) -> None:
        self.store.save(self.positions, self.position_to_angle(self.positions))

    def _load_positions(self) -> None:
        loaded = self.store.load()
        if loaded is not None:
            logger.info("loading position from disk")
            self.positions = loaded
            message = "Loaded, the positions are: \n"
            for servo, pos in zip(self.servos, self.positions):
                message += servo.message
                message += f"{(pos - ZERO_COUNT) * 360 / COUNTS_PER_TURN} deg\t"
            logger.info(message)
        else:
            self.positions = [ZERO_COUNT] * len(self.servos)
            self.save()

    # ----------------------------------------------------------- commands

    def set_zero(self) -> None:
        logger.info("Setting Zero")
        for servo in self.servos:
            iteration = 1
            while True:
                logger.debug("Set Zero Trial %d", iteration)
                if servo.set_zero() == 0:
                    break
                iteration += 1
        self.positions = [ZERO_COUNT] * len(self.servos)
        self.save()

    def torque_enable(self) -> None:
        for servo in self.servos:
            servo.torque_enable()

    def torque_disable(self) -> None:
        for servo in self.servos:
            servo.torque_disable()

    def home(self) -> None:
        logger.info("going home")
        self.set_positions([ZERO_COUNT] * len(self.servos))

    # ------------------------------------------------------------ readout

    def get_positions(self) -> List[int]:
        positions = self.bus.sync_read(self.ids, ADDR_STS_PRESENT_POSITION, 2)
        self.positions = list(positions)
        return self.positions

    def get_angles(self) -> np.ndarray:
        return self.position_to_angle(self.get_positions())

    def moving_status(self) -> List[int]:
        return self.bus.sync_read(self.ids, ADDR_STS_MOVING_STATUS, 1)

    # ------------------------------------------------------------- motion

    def set_positions(
        self,
        goal_position_list: Sequence[int],
        mask: Optional[Sequence[int]] = None,
    ) -> None:
        """Group move with the de-hysteresis sequence.

        The 3D-printed mount frame flexes: moves in the negative direction
        first overshoot by ``overshoot_counts`` and then return, so backlash
        is always taken up from the same side (legacy behavior, calibration
        accuracy depends on it).
        """
        self.get_positions()
        if len(goal_position_list) != len(self.ids):
            raise ValueError(
                f"Goal position list length {len(goal_position_list)} does not "
                f"match the number of motors {len(self.ids)}"
            )
        if mask is not None:
            goal_position_list = [
                goal_position_list[i] if mask[i] else self.positions[i]
                for i in range(len(goal_position_list))
            ]
        goal_position_list = [int(g) for g in list(goal_position_list)]

        threshold = self.cfg.de_hysteresis.threshold_counts
        overshoot = self.cfg.de_hysteresis.overshoot_counts
        # always approach from the positive direction: a negative move first
        # goes further negative, then comes back up
        de_hysteresis_mask = [0] * len(self.ids)
        for i in range(len(self.ids)):
            d_pos = goal_position_list[i] - self.positions[i]
            if (d_pos < 0) and (abs(d_pos) > threshold):
                de_hysteresis_mask[i] = 1

        if (not self.de_hysteresis) or (np.sum(de_hysteresis_mask) == 0):
            self._set_positions(goal_position_list)
        else:
            goals_overshoot = [
                g - overshoot if de_hysteresis_mask[i] else g
                for i, g in enumerate(goal_position_list)
            ]
            logger.debug("De-hysteresis On")
            logger.debug(
                "First go to %s and then go to %s", goals_overshoot, goal_position_list
            )
            self._set_positions(goals_overshoot)
            self._set_positions(goal_position_list)
        # persist once per completed move (the legacy code re-wrote the state
        # file on every poll read, wearing the SD card)
        self.save()

    def _set_positions(self, goal_position_list: Sequence[int]) -> None:
        scs_goals = [_encode_goal(g) for g in goal_position_list]
        self.bus.sync_write_2byte(self.ids, ADDR_STS_GOAL_POSITION, scs_goals)

        def _status_string(prefix: str) -> str:
            parts = [prefix]
            for index in range(len(self.ids)):
                parts.append(
                    f"[ID:{self.ids[index]:03d}] "
                    f"Goal:{goal_position_list[index]:03d} "
                    f"Pres:{self.positions[index]:03d}"
                )
            return "\t".join(parts)

        self.get_positions()
        logger.debug(_status_string("Start Position: "))

        t0 = time.time()
        iteration = 0
        while (time.time() - t0) < self.timeout:
            self.get_positions()
            moving = self.moving_status()

            if iteration % 100 == 0:
                logger.debug(_status_string(f"Iteration: {iteration}"))

            if sum(moving) == 0:
                logger.debug(_status_string(f"Iteration: {iteration}"))
                break
            iteration += 1

        self.get_positions()

    # -------------------------------------------------------- conversions

    def angle_to_position(self, angle) -> np.ndarray:
        angle = np.array(angle)
        return (angle * (COUNTS_PER_TURN / 360) + ZERO_COUNT).astype(int)

    def position_to_angle(self, position) -> np.ndarray:
        position = np.array(position)
        return ((position - ZERO_COUNT) * 360 / COUNTS_PER_TURN).astype(float)

    def set_angles(
        self, angles_deg: Sequence[float], mask: Optional[Sequence[int]] = None
    ) -> None:
        self.set_positions(self.angle_to_position(angles_deg), mask=mask)

    def set_single(self, index: int, angle_deg: float) -> None:
        mask = [0] * len(self.servos)
        mask[index] = 1
        angles = [0] * len(self.servos)
        angles[index] = angle_deg
        self.set_angles(angles, mask)
