<!-- SPDX-FileCopyrightText: 2026 Matthew Powelson
     SPDX-License-Identifier: Apache-2.0
-->
<!-- markdownlint-disable MD013 -->
# fanuc_driver Errata

This file captures compatibility issues found while testing `fanuc_driver` against a physical FANUC controller in March 2026.

Test context used for the findings below:

- Controller family: R-50 series
- RMI version shown on controller: `9`
- Driver network:
  - RMI TCP on `16001`
  - Stream Motion UDP on `60015`

## Confirmed Bugs

### 1. Stream Motion controller capability response is not always 28 bytes

Observed behavior:

- The driver originally expected `sizeof(ControllerCapabilityResultPacket) == 28` bytes.
- On the tested R-50 controller, the capability response packet was `25` bytes.
- Packet capture showed:
  - request payload length `28`
  - response payload length `25`

Impact:

- `StreamMotionConnection::getControllerCapability()` incorrectly logged a timeout / failure even though the controller had replied.
- The driver could then continue with uninitialized or byte-swapped capability data.

Notes:

- The observed `25`-byte payload decoded to:
  - `sampling_rate=2`
  - `start_move=1`
  - `available_version=2`
  - `rob_status_use_tcp=0`

### 2. RMI `FRC_GetStatus` response is incompatible with current packet definition on RMI v9

Observed behavior:

- The controller returned valid `FRC_GetStatus` JSON, but the driver rejected it and timed out waiting for a response.
- The response included additional keys not modeled in `StatusRequestPacket::Response`:
  - `UI[2]`
  - `UI[8]`

Example payload observed from the controller:

```json
{"Command":"FRC_GetStatus","ErrorID":0,"ServoReady":1,"TPMode":0,"RMIMotionStatus":0,"ProgramStatus":2,"SingleStepMode":0,"NumberUTool":10,"NumberUFrame":9,"Override":100,"UI[2]":1,"UI[8]":1}
```

Impact:

- `RMIConnection::getStatus()` threw:
  - `Timeout waiting for response`
  - `You may have incompatible packet definitions for the current RMI version.`

Root cause:

- JSON deserialization is strict enough that extra controller fields can invalidate packet parsing when the response struct does not declare them.

### 3. `rmi` target did not export `reflectcpp` even though its public headers required it

Observed behavior:

- After updating `rmi/include/rmi/packets.hpp` to use `rfl::Rename`, downstream compilation failed with:
  - `fatal error: rfl.hpp: No such file or directory`

Impact:

- `fanuc_client` could not compile against `rmi` even though `rmi`'s public header referenced reflect-cpp types.

Root cause:

- `fanuc_libs/rmi/CMakeLists.txt` linked `reflectcpp` as `PRIVATE` instead of `PUBLIC`.

## Open Compatibility Issues

### 4. Stream Motion startup still fails on the tested R-50 controller

Observed behavior:

- The controller reports and/or visibly shows that `STREAM_MOTN` is running.
- RMI connectivity is healthy.
- Stream Motion capability query succeeds.
- Stream Motion robot limit queries succeed.
- The driver never receives the first realtime status packet after startup.

Observed RMI status during the failed startup wait:

- `ErrorID=0`
- `ServoReady=1`
- `TPMode=0`
- `RMIMotionStatus=1` in some runs
- `ProgramStatus=0` in some runs
- `ProgramStatus=2` in some runs

Observed UDP behavior during startup:

- The controller responds to capability and robot limit queries.
- The driver sends:
  - start packets (`8` bytes payload)
  - hold-position command packets (`344` bytes payload)
- No realtime status packet is received after startup begins.

Current conclusion:

- This is no longer a basic network failure.
- It appears to be a protocol or startup-sequencing compatibility issue specific to this controller / firmware combination.
- At the time of writing, it is still under investigation whether the remaining issue is caused by:
  - a different Stream Motion v2 startup handshake,
  - a status-packet format mismatch not yet modeled in the driver,
  - or a controller-side precondition not surfaced cleanly through current RMI / UDP logging.

## Local Debugging Changes Used During Investigation

These were useful for investigation and may or may not be appropriate for upstreaming as-is:

- Extra UDP logging in `stream_motion/src/stream.cpp`
- Compatibility parsing for `25`-byte controller capability responses
- Extra RMI status logging during stream startup in `fanuc_client/src/fanuc_client.cpp`
- Retry logic for resending startup packets during stream activation
- Temporary seed command logic to send a hold-position command before the first realtime status is received

## Recommended Follow-Up

- Confirm whether R-50 / Stream Motion v2 uses a different realtime status packet shape than the currently modeled `V3RobotStatusPacket`.
- Confirm whether RMI v9 introduces additional `FRC_GetStatus` fields beyond `UI[2]` and `UI[8]` that should be modeled generically.
- Compare the tested controller behavior against the exact FANUC manual revision for:
  - Stream Motion version `2`
  - RMI version `9`
