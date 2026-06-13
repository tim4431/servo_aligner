import json
from pathlib import Path

import pytest

GOLDEN_PATH = Path(__file__).parent / "golden" / "golden_values.json"


@pytest.fixture(scope="session")
def golden():
    """Golden values captured from the legacy flat modules (pre-refactor).

    See tests/golden/_capture_from_legacy.py for provenance.
    """
    return json.loads(GOLDEN_PATH.read_text())
