# scservo_sdk — vendored FEETECH SDK

FEETECH's official SCServo Python SDK, vendored **unmodified**. It provides
the serial protocol layer (`PortHandler`, `PacketHandler`, `GroupSyncRead`,
`GroupSyncWrite`, byte macros) used by the STS3032 backend in
[../hal/sts3032.py](../hal/sts3032.py). Requires `pyserial` (the `hardware`
extra).

**Do not edit** — it is upstream code. The original archive is in
[../../example/scsservo_sdk_source/](../../example/scsservo_sdk_source/) and
usage examples in
[../../example/scsservo_sdk_example/](../../example/scsservo_sdk_example/).

Only the HAL touches it; the rest of the package goes through the `Actuator`
protocol and never imports this package directly.
