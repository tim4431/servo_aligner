"""Server adapter isolation: importable core, clear errors without extras."""

import pytest

from .conftest import write_sim_config


def test_server_package_import_without_zmq():
    # the server subpackage itself (and compat) must import without pyzmq
    import servo_aligner.server  # noqa: F401
    import servo_aligner.server.compat  # noqa: F401


def test_sequence_alias_requires_expctl(monkeypatch):
    # without expctl's `utilities` package the alias install must fail loudly
    from servo_aligner.server.compat import install_sequence_aliases

    with pytest.raises(RuntimeError, match="expctl"):
        install_sequence_aliases(("sequence_test_alias",))


def test_cli_server_requires_extra(tmp_path):
    # pyzmq is not installed in the test env: the CLI must explain the extra
    from servo_aligner.cli.main import main

    cfg = write_sim_config(tmp_path)
    with pytest.raises(SystemExit, match="server"):
        main(["-c", str(cfg), "server"])
