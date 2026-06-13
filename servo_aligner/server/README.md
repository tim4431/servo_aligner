# server — expctl ZMQ adapter (optional)

The bridge to the lab's `expctl` experiment-control framework. Requires the
`server` extra (`pip install -e ".[server]"`, pulls pyzmq + coloredlogs). The
**core package never imports this subpackage** — it is loaded only by
`servo-aligner server`.

| Module | Role |
|--------|------|
| [base.py](base.py) | `Server`: the ZMQ REP message loop (RUN / SEQ / QUEUE / GETPLOTDATA / PING, multipart pickle). Wire protocol unchanged from the pre-refactor server, so expctl clients need no changes. |
| [sts_server.py](sts_server.py) | `STSServer` (actuator injected, not constructed here) + `serve(cfg)`: during the QUEUE phase it applies the requested `Sequence` channel values as servo angles. |
| [compat.py](compat.py) | `install_sequence_aliases`: maps the module names pickled `Sequence` payloads reference (default `sequence`) onto the vendored copy before unpickling, with a clear error when expctl's `utilities` is missing. |
| [_vendor/](_vendor/) | Vendored expctl `Sequence`/`Channel` classes (needs expctl's `utilities` at runtime). |

Server port and the unpickle aliases come from the YAML `server` section.
Unpickling `Sequence` objects requires expctl's `utilities` package importable
on the server side — this is the one path that can only be verified on the Pi
against the real client (see [../../doc/migration.md](../../doc/migration.md) §6).
