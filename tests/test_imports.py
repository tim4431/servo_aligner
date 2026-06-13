"""Import hygiene: the core package must not require hardware libraries.

This test environment intentionally has no pyserial/smbus2/MCP342x/pyzmq
installed, so these imports passing proves the lazy-import design.
"""

import pytest


def test_core_imports_without_hardware_libs():
    import servo_aligner  # noqa: F401
    import servo_aligner.factory  # noqa: F401
    import servo_aligner.hal.mcp3424  # noqa: F401
    import servo_aligner.hal.sts3032  # noqa: F401
    import servo_aligner.optimize.spiral  # noqa: F401
    import servo_aligner.vectors  # noqa: F401


def test_unknown_backend_rejected(tmp_path):
    from servo_aligner.config import load_config
    from servo_aligner.factory import build_actuator

    p = tmp_path / "cfg.yaml"
    p.write_text(
        """
actuator:
  backend: warp_drive
  channels:
    - {name: x, servo_id: 1}
"""
    )
    with pytest.raises(ValueError, match="unknown actuator backend"):
        build_actuator(load_config(p))
