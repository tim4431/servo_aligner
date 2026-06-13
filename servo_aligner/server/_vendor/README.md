# _vendor — vendored expctl code

| Module | Role |
|--------|------|
| [sequence.py](sequence.py) | The expctl `Sequence` / `Channel` classes, copied verbatim so the ZMQ server can unpickle `Sequence` payloads sent by the lab client. |

**Do not edit** — keep in sync with upstream expctl. The module imports
expctl's `utilities` package, so it is only importable inside an expctl
environment; [../compat.py](../compat.py) handles wiring it under the module
name the pickled payloads expect.
