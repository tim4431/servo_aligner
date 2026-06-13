"""Test double for ScsBus: records every write, simulates instant moves."""

from __future__ import annotations

from servo_aligner.hal.sts3032 import (
    ADDR_STS_GOAL_POSITION,
    ADDR_STS_MOVING_STATUS,
    ADDR_STS_PRESENT_POSITION,
    ZERO_COUNT,
)


def decode_goal(value: int) -> int:
    """Invert the sign-magnitude goal encoding."""
    if value & 0x8000:
        return -(value & 0x7FFF)
    return value


class FakeScsBus:
    """Stands in for ScsBus; servos teleport to the last written goal.

    ``sync_writes`` records ``(addr, [decoded goals...])`` per group write —
    the de-hysteresis tests assert on this command log. ``scripted_reads``
    (per servo id) overrides PRESENT_POSITION reads for the multi-turn tests.
    """

    def __init__(self, ids):
        self.ids = list(ids)
        self.positions = {scs_id: ZERO_COUNT for scs_id in self.ids}
        self.register_writes = []  # (id, addr, value) from write1/write2
        self.sync_writes = []  # (addr, [decoded values]) from sync_write_2byte
        self.scripted_reads = {}  # id -> list of raw PRESENT_POSITION values
        self.closed = False

    # ------------------------------------------------------- single-servo IO

    def write1(self, scs_id, addr, value):
        self.register_writes.append((scs_id, addr, value))

    def write2(self, scs_id, addr, value):
        self.register_writes.append((scs_id, addr, value))
        if addr == ADDR_STS_GOAL_POSITION:
            self.positions[scs_id] = decode_goal(value)

    def read4(self, scs_id, addr):
        if addr == ADDR_STS_PRESENT_POSITION:
            script = self.scripted_reads.get(scs_id)
            if script:
                return script.pop(0)
            return self.positions[scs_id]
        if addr == ADDR_STS_MOVING_STATUS:
            return 0
        return 0

    # -------------------------------------------------------------- group IO

    def sync_read(self, ids, addr, length):
        if addr == ADDR_STS_PRESENT_POSITION:
            return [self.positions[scs_id] for scs_id in ids]
        if addr == ADDR_STS_MOVING_STATUS:
            return [0 for _ in ids]
        return [0 for _ in ids]

    def sync_write_2byte(self, ids, addr, values):
        decoded = [decode_goal(v) for v in values]
        self.sync_writes.append((addr, decoded))
        if addr == ADDR_STS_GOAL_POSITION:
            for scs_id, goal in zip(ids, decoded):
                self.positions[scs_id] = goal

    def close(self):
        self.closed = True
