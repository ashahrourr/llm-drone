# Flying real PX4 firmware

The simulator in this repo is my own. This directory runs the *real* PX4 flight
stack instead — EKF2, commander, the position controller, MAVLink — and flies it
with the same command layer.

It works on an Apple Silicon Mac. There is no Ubuntu box and no Gazebo.

## Why no Gazebo

PX4 ships **SIH** (Simulation-In-Hardware): the firmware integrates its own
rigid-body model in-process at 250 Hz. Gazebo is only needed when you want
cameras, lidar or contact physics.

That matters here because PX4's published Docker images are `amd64`-only, and
Gazebo's are too, so both would run under emulation. PX4 itself builds cleanly
for `arm64` from source, and SIH removes the only reason we needed Gazebo.

The airframe id selects it:

| `PX4_SYS_AUTOSTART` | simulator | works on arm64 |
|---|---|---|
| `10016` | Gazebo x500 | ✗ blocks on TCP 4560 waiting for an external sim |
| **`10040`** | **SIH quadx, built in** | **✓** |

Getting this wrong is quiet: PX4 starts, prints
`Waiting for simulator to accept connection on TCP port 4560`, and never brings
MAVLink up, so the symptom is a missing heartbeat rather than an error.

## Run it

```bash
./px4/build.sh        # clone + compile PX4 for arm64 (~25 min, once)
./px4/fly.sh          # arm, climb to 10 m, translate, land
```

Output from a real run:

```
PX4 ready, home set
OFFBOARD: ACCEPTED
ARMED after 4 attempts
  alt   3.13 m
  alt   9.93 m
PEAK ALTITUDE: 10.05 m
  N=   8.6 E=   5.9 alt= 10.0
  N=  20.3 E=  15.2 alt= 10.3
FINAL: N=20.3 E=15.4 alt=9.9
land: ACCEPTED
```

## Two things PX4 will not forgive

**Setpoints have to stream before *and* during OFFBOARD.** PX4 requires a
setpoint at better than 2 Hz to enter OFFBOARD and to stay in it. Send the mode
switch first and it is rejected; stop streaming mid-flight and the vehicle drops
out of the mode. `fly.sh` streams from a background thread for exactly this
reason.

**Arming is `TEMPORARILY_REJECTED` until the EKF is happy.** It is not a
failure — the estimator is still converging and home is not set. Retrying every
two seconds gets there in about four attempts. Treating the first rejection as
fatal is the usual mistake.

## What this shares with the rest of the repo

`drone/commands.py` — the schema and the guard — is transport-agnostic. The same
validated command that becomes a setpoint for the built-in simulator becomes a
`SET_POSITION_TARGET_LOCAL_NED` here. The envelope, the clamping and the
rejections are identical; only the thing on the other end changed.
