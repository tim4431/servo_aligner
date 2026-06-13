"""Build the hardware (or simulation) stack from configuration.

Hardware drivers are imported lazily inside the builders, so importing this
module — and selecting ``backend: simulation`` — works on machines without
smbus2/pyserial installed.
"""

from __future__ import annotations

from dataclasses import dataclass

from .channels import ChannelLayout
from .config import Config
from .hal.interfaces import Actuator, IntensitySensor
from .measurement import Measurement


@dataclass
class AlignerStack:
    """Everything a routine needs, built from one config."""

    config: Config
    layout: ChannelLayout
    actuator: Actuator
    sensor: IntensitySensor
    measurement: Measurement

    def close(self) -> None:
        self.actuator.close()
        self.sensor.close()

    def __enter__(self) -> "AlignerStack":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()


def build_actuator(cfg: Config) -> Actuator:
    backend = cfg.actuator.backend
    if backend == "sts3032":
        from .hal.sts3032 import Sts3032Actuator

        cfg.state_dir.mkdir(parents=True, exist_ok=True)
        return Sts3032Actuator(cfg.actuator, state_dir=cfg.state_dir)
    if backend == "simulation":
        from .hal.simulation import SimulatedActuator

        return SimulatedActuator(cfg.actuator)
    raise ValueError(f"unknown actuator backend: {backend!r}")


def build_sensor(cfg: Config, actuator: Actuator) -> IntensitySensor:
    backend = cfg.sensor.backend
    if backend == "mcp3424":
        from .hal.mcp3424 import Mcp3424Sensor

        return Mcp3424Sensor(cfg.sensor)
    if backend == "simulation":
        from .hal.simulation import SimulatedActuator, SimulatedIntensitySensor

        if not isinstance(actuator, SimulatedActuator):
            raise ValueError(
                "sensor backend 'simulation' requires actuator backend "
                "'simulation' (the simulated sensor reads the simulated pose)"
            )
        return SimulatedIntensitySensor(
            actuator=actuator,
            layout=cfg.layout(),
            sim=cfg.simulation,
            samples_per_read=cfg.sensor.samples_per_read,
        )
    raise ValueError(f"unknown sensor backend: {backend!r}")


def build_stack(cfg: Config) -> AlignerStack:
    actuator = build_actuator(cfg)
    sensor = build_sensor(cfg, actuator)
    return AlignerStack(
        config=cfg,
        layout=cfg.layout(),
        actuator=actuator,
        sensor=sensor,
        measurement=Measurement(actuator, sensor),
    )
