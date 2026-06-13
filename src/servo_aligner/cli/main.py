"""The ``servo-aligner`` command-line interface.

Replaces the legacy practice of executing the alignment scripts directly
(which moved motors at import time). Every command loads the YAML config
(``-c`` / ``$SERVO_ALIGNER_CONFIG`` / ``./servo_aligner.yaml``) and builds
the hardware — or simulation — stack explicitly.
"""

from __future__ import annotations

import argparse
import logging
import sys
from typing import Optional, Sequence

from ..config import Config, load_config
from ..factory import build_actuator


def _setup_logging(verbose: bool) -> None:
    logging.basicConfig(
        level=logging.DEBUG if verbose else logging.INFO,
        format="%(asctime)s - %(levelname)s - %(message)s",
    )


def _load(args) -> Config:
    return load_config(args.config)


# ----------------------------------------------------------------- commands


def cmd_status(args) -> int:
    cfg = _load(args)
    actuator = build_actuator(cfg)
    try:
        angles = actuator.get_angles()
        print(f"backend: {cfg.actuator.backend}")
        for name, angle in zip(actuator.channel_names, angles):
            print(f"  {name:10s} {angle:10.3f} deg")
    finally:
        actuator.close()
    return 0


def cmd_home(args) -> int:
    cfg = _load(args)
    actuator = build_actuator(cfg)
    try:
        actuator.torque_enable()
        actuator.home()
    finally:
        actuator.close()
    return 0


def cmd_set_zero(args) -> int:
    cfg = _load(args)
    actuator = build_actuator(cfg)
    try:
        actuator.set_zero()
    finally:
        actuator.close()
    return 0


def cmd_set_angle(args) -> int:
    cfg = _load(args)
    actuator = build_actuator(cfg)
    try:
        actuator.torque_enable()
        actuator.set_angles(args.angle)
    finally:
        actuator.close()
    return 0


def cmd_set_single(args) -> int:
    cfg = _load(args)
    actuator = build_actuator(cfg)
    try:
        actuator.torque_enable()
        actuator.set_single(args.index, args.angle)
    finally:
        actuator.close()
    return 0


def cmd_clip_scan(args) -> int:
    from ..routines.clip_scan import run_clip_scan

    cfg = _load(args)
    run_clip_scan(cfg, plot=not args.no_plot, iterations=args.iterations)
    return 0


def cmd_calibrate_jacobian(args) -> int:
    from ..routines.calibrate_jacobian import run_jacobian_calibration

    cfg = _load(args)
    run_jacobian_calibration(
        cfg,
        master=args.master,
        offset_type=args.offset_type,
        norm=args.norm,
        n_iterations=args.n_iterations,
        plot=args.plot,
    )
    return 0


def cmd_server(args) -> int:
    cfg = _load(args)
    try:
        from ..server.sts_server import serve
    except ImportError as e:
        raise SystemExit(
            f"the server command requires the [server] extra (pip install "
            f"'servo-aligner[server]'): {e}"
        )
    de_hysteresis = cfg.server.de_hysteresis_on_start
    if args.dehys is not None:
        de_hysteresis = bool(args.dehys)
    serve(cfg, de_hysteresis=de_hysteresis)
    return 0


# -------------------------------------------------------------------- parser


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="servo-aligner",
        description="Automated laser-beam alignment with serial-bus servos.",
    )
    parser.add_argument(
        "-c",
        "--config",
        default=None,
        help="YAML config path (default: $SERVO_ALIGNER_CONFIG or ./servo_aligner.yaml)",
    )
    parser.add_argument("-v", "--verbose", action="store_true", help="debug logging")
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("status", help="read and print all channel angles")
    p.set_defaults(func=cmd_status)

    p = sub.add_parser("home", help="move all channels to 0 degrees")
    p.set_defaults(func=cmd_home)

    p = sub.add_parser("set-zero", help="define the current pose as zero")
    p.set_defaults(func=cmd_set_zero)

    p = sub.add_parser("set-angle", help="move all channels to the given angles")
    p.add_argument("angle", nargs="+", type=float, help="one angle per channel (deg)")
    p.set_defaults(func=cmd_set_angle)

    p = sub.add_parser("set-single", help="move one channel")
    p.add_argument("index", type=int, help="channel index")
    p.add_argument("angle", type=float, help="angle to move to (deg)")
    p.set_defaults(func=cmd_set_single)

    p = sub.add_parser(
        "clip-scan", help="raster knob pairs, fit the beam-clip center, re-zero"
    )
    p.add_argument("--no-plot", action="store_true", help="skip figures (headless)")
    p.add_argument(
        "--iterations",
        nargs="+",
        type=int,
        default=None,
        help="iteration indices (default: from config)",
    )
    p.set_defaults(func=cmd_clip_scan)

    p = sub.add_parser(
        "calibrate-jacobian",
        help="offset one beam path and re-optimize the other (Jacobian dataset)",
    )
    p.add_argument("--master", default=None, help="master path (e.g. A or B)")
    p.add_argument(
        "--offset-type",
        default=None,
        choices=["pm", "rand", "lin", "zero", "spec"],
        help="offset generation strategy",
    )
    p.add_argument("--norm", type=float, default=None, help="offset norm (deg)")
    p.add_argument(
        "-n", "--n-iterations", type=int, default=None, help="number of iterations"
    )
    p.add_argument("--plot", action="store_true", help="save optimizer trace figures")
    p.set_defaults(func=cmd_calibrate_jacobian)

    p = sub.add_parser("server", help="run the expctl ZMQ server (server extra)")
    p.add_argument(
        "--dehys",
        type=int,
        choices=[0, 1],
        default=None,
        help="override de-hysteresis on start",
    )
    p.set_defaults(func=cmd_server)

    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    _setup_logging(args.verbose)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
