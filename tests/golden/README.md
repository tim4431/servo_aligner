# golden

| File | Role |
|------|------|
| `golden_values.json` | Reference outputs captured from the **pre-refactor** flat modules (tag `pre-refactor`): `compose_para`, the spiral trajectory, both optimizer paths, the fits, the beam-clip scan, and the legacy mask constants. The parity tests assert the refactored package reproduces these exactly. |
| `_capture_from_legacy.py` | The one-shot script that produced `golden_values.json`, kept for provenance. It imported the old `src/*.py` modules, which are now deleted, so **it can no longer run**. |

⚠️ Never regenerate `golden_values.json` from the *current* code — that would
make the parity tests tautological. The file is a frozen record of the
original behavior; it only changes if the original behavior is intentionally
re-captured from the `pre-refactor` tag.
