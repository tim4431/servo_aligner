"""YAML configuration loaded into typed dataclasses.

One YAML file per machine replaces the legacy ``customize.py`` plus all the
constants that used to be hardcoded across the scripts (paths, optimizer
tuning, accept lines, scan stages...). See ``config/example_config.yaml``
for a fully commented reference.

Resolution order for the config path: explicit argument →
``$SERVO_ALIGNER_CONFIG`` → ``./servo_aligner.yaml``.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field, fields
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple, Union

import yaml

from .channels import ChannelLayout

ENV_VAR = "SERVO_ALIGNER_CONFIG"
DEFAULT_FILENAME = "servo_aligner.yaml"


# --------------------------------------------------------------------- HAL


@dataclass(frozen=True)
class ChannelConfig:
    name: str
    servo_id: int
    label: str = ""


@dataclass(frozen=True)
class DeHysteresisConfig:
    enabled: bool = True
    overshoot_counts: int = 100
    threshold_counts: int = 2


@dataclass(frozen=True)
class ActuatorConfig:
    channels: Tuple[ChannelConfig, ...]
    backend: str = "sts3032"  # sts3032 | simulation
    board_id: int = 0
    ports: Tuple[str, ...] = ("/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2")
    baudrate: int = 1000000
    speed: int = 2000
    acc: int = 90
    move_timeout_s: float = 10.0
    de_hysteresis: DeHysteresisConfig = field(default_factory=DeHysteresisConfig)


@dataclass(frozen=True)
class SensorConfig:
    backend: str = "mcp3424"  # mcp3424 | simulation
    i2c_bus: int = 1
    address: int = 0x68
    channel: int = 0
    gain: int = 1
    resolution: int = 16
    samples_per_read: int = 2


# --------------------------------------------------------------- optimizer


@dataclass(frozen=True)
class SpiralParams:
    """Field names match SpiralPath attributes (applied via load_options)."""

    I_meaningful: float = 0.005
    D: float = 2.4
    SPIRAL_RESOLUTION: int = 14
    SPIRAL_SPAN: float = 6
    SINGLE_SPIRAL_SPAN: float = 3.5
    N_LOOPS_BEFORE_RESET_ORIGIN: float = 0.5
    MAX_X0Y0_DISPLACEMENT: float = 10
    COEF_I_RESET_ORIGIN: float = 1.4
    alpha: float = 0.03
    COEF_I_DECAY: float = 0.995

    def as_options(self) -> dict:
        return {f.name: getattr(self, f.name) for f in fields(self)}


@dataclass(frozen=True)
class OptimizeConfig:
    accept_ratio: float = 0.7
    default_bounds: Tuple[float, float] = (-100.0, 100.0)
    spiral: SpiralParams = field(default_factory=SpiralParams)
    lbfgsb: Dict[str, object] = field(
        default_factory=lambda: {"disp": True, "maxiter": 10, "eps": 5}
    )


# ----------------------------------------------------------------- routines


@dataclass(frozen=True)
class AcceptLine:
    """Linear acceptance band |y - slope*x - intercept| < tol for 2D scans."""

    slope: float
    intercept: float
    tol: float


@dataclass(frozen=True)
class ClipScanStage:
    group: str
    n_pts: int
    range: float
    accept: bool = True
    plot_type: int = 0


@dataclass(frozen=True)
class ClipScanConfig:
    output_dir: str = "{data_dir}/clip_scan"
    I_meaningful: float = 0.1
    accept_lines: Dict[str, AcceptLine] = field(default_factory=dict)
    stages: Tuple[ClipScanStage, ...] = ()
    iterations: Tuple[int, ...] = (0,)


@dataclass(frozen=True)
class JacobianStage:
    group_suffix: str
    method: str = "spiral"
    bounds: Optional[Tuple[float, float]] = None


@dataclass(frozen=True)
class JacobianConfig:
    output_dir: str = "{data_dir}/jacobian"
    paths: Tuple[str, ...] = ("A", "B")
    master: str = "B"
    offset_type: str = "zero"  # pm | rand | lin | zero | spec
    norm: float = 20.0
    n_iterations: int = 1
    assumed_jacobian: Optional[str] = None
    spec_offset: Tuple[float, ...] = (3.0, 0.0, 0.0, 0.0)
    lin_comb_vectors: Dict[str, Tuple[Tuple[float, ...], ...]] = field(
        default_factory=dict
    )
    stages: Tuple[JacobianStage, ...] = (
        JacobianStage("X_Y", method="spiral", bounds=(-100.0, 100.0)),
        JacobianStage("X_XDOT", method="spiral"),
        JacobianStage("Y_YDOT", method="spiral"),
        JacobianStage("POS_ALL", method="L-BFGS-B"),
    )


# --------------------------------------------------------------- simulation


@dataclass(frozen=True)
class SimBeamPath:
    channels: Tuple[str, str, str, str]  # x, y, xdot, ydot channel names
    weight: float = 1.0


@dataclass(frozen=True)
class SimulationConfig:
    noise_rms: float = 1e-4
    seed: Optional[int] = None
    smooth_transition: Optional[float] = None  # test-only; None = lab physics
    model: Dict[str, object] = field(default_factory=dict)
    paths: Tuple[SimBeamPath, ...] = ()
    true_zero: Tuple[float, ...] = ()


# ------------------------------------------------------------------- server


@dataclass(frozen=True)
class ServerConfig:
    name: str = "STS1"
    port: int = 60627
    de_hysteresis_on_start: bool = True
    # module names that pickled Sequence payloads may reference
    sequence_module_aliases: Tuple[str, ...] = ("sequence",)


# --------------------------------------------------------------------- root


@dataclass(frozen=True)
class Config:
    actuator: ActuatorConfig
    state_dir: Path = Path("~/servo_aligner/state")
    data_dir: Path = Path("~/servo_aligner/data")
    sensor: SensorConfig = field(default_factory=SensorConfig)
    groups: Dict[str, Union[List[str], str]] = field(default_factory=dict)
    optimize: OptimizeConfig = field(default_factory=OptimizeConfig)
    clip_scan: ClipScanConfig = field(default_factory=ClipScanConfig)
    jacobian: JacobianConfig = field(default_factory=JacobianConfig)
    simulation: SimulationConfig = field(default_factory=SimulationConfig)
    server: ServerConfig = field(default_factory=ServerConfig)
    source_path: Optional[Path] = None

    @property
    def channel_names(self) -> Tuple[str, ...]:
        return tuple(ch.name for ch in self.actuator.channels)

    def layout(self) -> ChannelLayout:
        return ChannelLayout(self.channel_names, self.groups)

    def resolve_dir(self, template: str) -> Path:
        """Expand ``{data_dir}`` / ``{state_dir}`` / ``~`` in a dir template."""
        expanded = str(template).format(
            data_dir=self.data_dir, state_dir=self.state_dir
        )
        return Path(expanded).expanduser()


# ----------------------------------------------------------------- loading


class ConfigError(ValueError):
    pass


def _build(cls, data: dict, context: str):
    """Construct a dataclass from a dict, rejecting unknown keys."""
    known = {f.name for f in fields(cls)}
    unknown = set(data) - known
    if unknown:
        raise ConfigError(f"{context}: unknown keys {sorted(unknown)}")
    return cls(**data)


def _parse(raw: dict, source_path: Optional[Path]) -> Config:
    raw = dict(raw or {})

    act_raw = dict(raw.pop("actuator", None) or {})
    channels = tuple(
        _build(ChannelConfig, dict(ch), "actuator.channels")
        for ch in act_raw.pop("channels", [])
    )
    if not channels:
        raise ConfigError("actuator.channels must define at least one channel")
    if "ports" in act_raw:
        act_raw["ports"] = tuple(act_raw["ports"])
    if "de_hysteresis" in act_raw:
        act_raw["de_hysteresis"] = _build(
            DeHysteresisConfig, dict(act_raw["de_hysteresis"]), "actuator.de_hysteresis"
        )
    actuator = _build(ActuatorConfig, {**act_raw, "channels": channels}, "actuator")

    sensor = _build(SensorConfig, dict(raw.pop("sensor", None) or {}), "sensor")

    opt_raw = dict(raw.pop("optimize", None) or {})
    if "spiral" in opt_raw:
        opt_raw["spiral"] = _build(SpiralParams, dict(opt_raw["spiral"]), "optimize.spiral")
    if "default_bounds" in opt_raw:
        opt_raw["default_bounds"] = tuple(opt_raw["default_bounds"])
    optimize = _build(OptimizeConfig, opt_raw, "optimize")

    cs_raw = dict(raw.pop("clip_scan", None) or {})
    if "accept_lines" in cs_raw:
        cs_raw["accept_lines"] = {
            name: _build(AcceptLine, dict(line), f"clip_scan.accept_lines.{name}")
            for name, line in cs_raw["accept_lines"].items()
        }
    if "stages" in cs_raw:
        cs_raw["stages"] = tuple(
            _build(ClipScanStage, dict(st), "clip_scan.stages") for st in cs_raw["stages"]
        )
    if "iterations" in cs_raw:
        cs_raw["iterations"] = tuple(cs_raw["iterations"])
    clip_scan = _build(ClipScanConfig, cs_raw, "clip_scan")

    jac_raw = dict(raw.pop("jacobian", None) or {})
    if "stages" in jac_raw:
        jac_raw["stages"] = tuple(
            _build(
                JacobianStage,
                {**dict(st), "bounds": tuple(st["bounds"]) if st.get("bounds") else None},
                "jacobian.stages",
            )
            for st in jac_raw["stages"]
        )
    if "paths" in jac_raw:
        jac_raw["paths"] = tuple(jac_raw["paths"])
    if "spec_offset" in jac_raw:
        jac_raw["spec_offset"] = tuple(jac_raw["spec_offset"])
    if "lin_comb_vectors" in jac_raw:
        jac_raw["lin_comb_vectors"] = {
            path: tuple(tuple(v) for v in vecs)
            for path, vecs in jac_raw["lin_comb_vectors"].items()
        }
    jacobian = _build(JacobianConfig, jac_raw, "jacobian")

    sim_raw = dict(raw.pop("simulation", None) or {})
    if "paths" in sim_raw:
        sim_raw["paths"] = tuple(
            _build(
                SimBeamPath,
                {**dict(p), "channels": tuple(p["channels"])},
                "simulation.paths",
            )
            for p in sim_raw["paths"]
        )
    if "true_zero" in sim_raw:
        sim_raw["true_zero"] = tuple(sim_raw["true_zero"])
    simulation = _build(SimulationConfig, sim_raw, "simulation")

    srv_raw = dict(raw.pop("server", None) or {})
    if "sequence_module_aliases" in srv_raw:
        srv_raw["sequence_module_aliases"] = tuple(srv_raw["sequence_module_aliases"])
    server = _build(ServerConfig, srv_raw, "server")

    groups = dict(raw.pop("groups", None) or {})

    state_dir = Path(raw.pop("state_dir", "~/servo_aligner/state")).expanduser()
    data_dir = Path(raw.pop("data_dir", "~/servo_aligner/data")).expanduser()

    if raw:
        raise ConfigError(f"unknown top-level config keys: {sorted(raw)}")

    cfg = Config(
        actuator=actuator,
        state_dir=state_dir,
        data_dir=data_dir,
        sensor=sensor,
        groups=groups,
        optimize=optimize,
        clip_scan=clip_scan,
        jacobian=jacobian,
        simulation=simulation,
        server=server,
        source_path=source_path,
    )
    _validate(cfg)
    return cfg


def _validate(cfg: Config) -> None:
    names = cfg.channel_names
    if len(set(names)) != len(names):
        raise ConfigError(f"duplicate channel names: {names}")
    ids = [ch.servo_id for ch in cfg.actuator.channels]
    if len(set(ids)) != len(ids):
        raise ConfigError(f"duplicate servo ids: {ids}")

    # groups must reference existing channels (also raises via ChannelLayout)
    layout = cfg.layout()

    for group_name in cfg.clip_scan.accept_lines:
        if group_name not in layout:
            raise ConfigError(
                f"clip_scan.accept_lines references unknown group {group_name!r}"
            )
    for stage in cfg.clip_scan.stages:
        if stage.group not in layout:
            raise ConfigError(f"clip_scan.stages references unknown group {stage.group!r}")
        if layout.group(stage.group).n != 2:
            raise ConfigError(
                f"clip_scan stage group {stage.group!r} must select exactly 2 channels"
            )

    if cfg.jacobian.master not in cfg.jacobian.paths:
        raise ConfigError(
            f"jacobian.master {cfg.jacobian.master!r} not in paths {cfg.jacobian.paths}"
        )

    for path in cfg.simulation.paths:
        for ch in path.channels:
            if ch not in names:
                raise ConfigError(f"simulation.paths references unknown channel {ch!r}")
    if cfg.simulation.true_zero and len(cfg.simulation.true_zero) != len(names):
        raise ConfigError(
            "simulation.true_zero must have one entry per channel "
            f"({len(names)}), got {len(cfg.simulation.true_zero)}"
        )


def find_config_path(path: Union[str, Path, None] = None) -> Path:
    """Resolve the config file: explicit arg → $SERVO_ALIGNER_CONFIG → cwd."""
    if path is not None:
        return Path(path).expanduser()
    env = os.environ.get(ENV_VAR)
    if env:
        return Path(env).expanduser()
    return Path(DEFAULT_FILENAME)


def load_config(path: Union[str, Path, None] = None) -> Config:
    """Load and validate the YAML configuration."""
    resolved = find_config_path(path)
    if not resolved.exists():
        raise ConfigError(
            f"config file not found: {resolved} "
            f"(pass --config, set ${ENV_VAR}, or create ./{DEFAULT_FILENAME}; "
            "see config/example_config.yaml)"
        )
    raw = yaml.safe_load(resolved.read_text())
    return _parse(raw, source_path=resolved)
