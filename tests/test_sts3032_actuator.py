"""De-hysteresis sequencing, masking, persistence, and multi-turn tracking,
tested against a fake bus (no hardware)."""

import json

import numpy as np
import pytest

from servo_aligner.config import (
    ActuatorConfig,
    ChannelConfig,
    DeHysteresisConfig,
)
from servo_aligner.hal.interfaces import Actuator
from servo_aligner.hal.sts3032 import (
    ADDR_STS_GOAL_POSITION,
    Sts3032Actuator,
    Sts3032Servo,
)

from .fake_bus import FakeScsBus


def make_actuator(tmp_path, n=2, **overrides):
    channels = tuple(
        ChannelConfig(name=f"ch{i}", servo_id=i + 1, label=f"l{i}") for i in range(n)
    )
    cfg = ActuatorConfig(
        channels=channels,
        de_hysteresis=DeHysteresisConfig(
            enabled=overrides.pop("de_hysteresis_enabled", True)
        ),
        **overrides,
    )
    bus = FakeScsBus([ch.servo_id for ch in channels])
    actuator = Sts3032Actuator(cfg, state_dir=tmp_path, bus=bus)
    return actuator, bus


def goal_writes(bus):
    return [vals for addr, vals in bus.sync_writes if addr == ADDR_STS_GOAL_POSITION]


# --------------------------------------------------------------- de-hysteresis


def test_positive_move_is_single_write(tmp_path):
    actuator, bus = make_actuator(tmp_path)
    actuator.set_positions([2148, 2148])
    assert goal_writes(bus) == [[2148, 2148]]


def test_negative_move_overshoots_then_returns(tmp_path):
    actuator, bus = make_actuator(tmp_path)
    actuator.set_positions([2148, 2148])
    bus.sync_writes.clear()
    actuator.set_positions([2048, 2148])
    # channel 0 moves -100: overshoot to 1948 first, then settle at 2048;
    # channel 1 stays and must NOT overshoot
    assert goal_writes(bus) == [[1948, 2148], [2048, 2148]]


def test_small_negative_move_within_threshold_skips_dehysteresis(tmp_path):
    actuator, bus = make_actuator(tmp_path)
    actuator.set_positions([2050, 2048])
    bus.sync_writes.clear()
    actuator.set_positions([2048, 2048])  # delta -2, not > threshold 2
    assert goal_writes(bus) == [[2048, 2048]]


def test_dehysteresis_disabled_flag(tmp_path):
    actuator, bus = make_actuator(tmp_path)
    actuator.set_positions([2148, 2148])
    bus.sync_writes.clear()
    actuator.de_hysteresis = False
    actuator.set_positions([2048, 2048])
    assert goal_writes(bus) == [[2048, 2048]]


def test_config_can_disable_dehysteresis(tmp_path):
    actuator, _ = make_actuator(tmp_path, de_hysteresis_enabled=False)
    assert actuator.de_hysteresis is False


def test_mask_holds_unmasked_channels(tmp_path):
    actuator, bus = make_actuator(tmp_path)
    actuator.set_positions([2200, 2300])
    bus.sync_writes.clear()
    actuator.set_positions([2400, 9999], mask=[1, 0])
    assert goal_writes(bus) == [[2400, 2300]]


def test_wrong_length_raises(tmp_path):
    actuator, _ = make_actuator(tmp_path)
    with pytest.raises(ValueError, match="does not match"):
        actuator.set_positions([2048])


# ----------------------------------------------------------- angles & helpers


def test_angle_roundtrip(tmp_path):
    actuator, bus = make_actuator(tmp_path)
    actuator.set_angles([90.0, -45.0])
    # 90 deg -> 3072, -45 deg -> 1536
    assert goal_writes(bus)[-1] == [3072, 1536]
    np.testing.assert_allclose(actuator.get_angles(), [90.0, -45.0])


def test_set_single(tmp_path):
    actuator, bus = make_actuator(tmp_path)
    actuator.set_angles([10.0, 20.0])
    bus.sync_writes.clear()
    actuator.set_single(0, 45.0)
    goals = goal_writes(bus)[-1]
    assert goals[0] == 2560  # 45 deg
    assert goals[1] == actuator.angle_to_position([10.0, 20.0])[1]


def test_home(tmp_path):
    actuator, bus = make_actuator(tmp_path)
    actuator.set_positions([2500, 2500])
    actuator.home()
    assert goal_writes(bus)[-1] == [2048, 2048]


def test_satisfies_actuator_protocol(tmp_path):
    actuator, _ = make_actuator(tmp_path)
    assert isinstance(actuator, Actuator)


# ----------------------------------------------------------------- persistence


def test_state_file_created_and_updated_per_move(tmp_path):
    actuator, _ = make_actuator(tmp_path)
    state = tmp_path / "servos_0.json"
    assert state.exists()
    assert json.loads(state.read_text())["position"] == [2048, 2048]

    actuator.set_positions([2148, 2248])
    saved = json.loads(state.read_text())
    assert saved["position"] == [2148, 2248]
    assert saved["angles_deg"] == pytest.approx(
        [(2148 - 2048) * 360 / 4096, (2248 - 2048) * 360 / 4096]
    )


def test_existing_state_file_loads(tmp_path):
    state = tmp_path / "servos_0.json"
    state.write_text(json.dumps({"position": [2100, 2200], "angles_deg": [0, 0]}))
    actuator, _ = make_actuator(tmp_path)
    assert actuator.positions == [2100, 2200]


def test_set_zero_resets_state(tmp_path):
    actuator, bus = make_actuator(tmp_path)
    actuator.set_positions([2500, 2500])
    actuator.set_zero()
    assert actuator.positions == [2048, 2048]
    # torque register written with 128 for each servo
    zero_writes = [w for w in bus.register_writes if w[1] == 40 and w[2] == 128]
    assert len(zero_writes) == 2


def test_close_saves_and_closes_bus(tmp_path):
    actuator, bus = make_actuator(tmp_path)
    actuator.positions = [2222, 2333]
    actuator.close()
    assert bus.closed
    assert json.loads((tmp_path / "servos_0.json").read_text())["position"] == [
        2222,
        2333,
    ]


# ------------------------------------------------------------------ multi-turn


def test_single_servo_multiturn_wraparound_negative(tmp_path):
    bus = FakeScsBus([1])
    servo = Sts3032Servo(bus, servo_id=1, label="t")
    # start at raw 100; command -96: the encoder wraps to 4000 and the
    # software turn counter must make angle_current = 4000 - 4096 = -96
    bus.scripted_reads[1] = [100, 4000]
    servo.set_position(-96)
    assert servo.turn_num == -1
    assert servo.angle_current == -96


def test_single_servo_multiturn_wraparound_positive(tmp_path):
    bus = FakeScsBus([1])
    servo = Sts3032Servo(bus, servo_id=1, label="t")
    # start near the top of the range; wrap upward: raw 4090 -> 30
    # angle_current = 30 + 4096 = 4126
    bus.scripted_reads[1] = [4090, 30]
    servo.set_position(4126)
    assert servo.turn_num == 1
    assert servo.angle_current == 4126


def test_single_servo_no_wraparound(tmp_path):
    bus = FakeScsBus([1])
    servo = Sts3032Servo(bus, servo_id=1, label="t")
    bus.scripted_reads[1] = [2048, 2148]
    servo.set_position(2148)
    assert servo.turn_num == 0
    assert servo.angle_current == 2148
