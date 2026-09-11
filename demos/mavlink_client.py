#!/usr/bin/env python3
"""Fly the MAVLink vehicle the way a ground station would.

Start `python mavlink_sim.py` first, then run this. It does what QGroundControl
does: wait for a heartbeat, arm, take off, send position targets, return.
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone.mavlink_io import MavlinkLink


def wait_until(link, north, east, altitude, tol=1.2, timeout=25.0) -> bool:
    """Re-send the target and watch telemetry until it is reached."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        link.goto(north, east, altitude)
        msg = link.telemetry(timeout=1.0)
        if msg is None:
            continue
        gap = math.dist((msg.x, msg.y, msg.z), (north, east, -abs(altitude)))
        print(f"    N={msg.x:6.1f} E={msg.y:6.1f} alt={-msg.z:5.1f}  gap={gap:5.2f}m", end="\r")
        if gap < tol:
            print()
            return True
    print()
    return False


def main() -> int:
    link = MavlinkLink("udpout:127.0.0.1:14550")
    print("waiting for heartbeat...")
    if not link.wait_heartbeat(timeout=10.0):
        print("no vehicle found — is mavlink_sim.py running?")
        return 1
    print(f"connected to system {link.target_system}")

    print("arm + takeoff to 8m")
    link.arm()
    time.sleep(0.5)
    link.takeoff(8.0)
    wait_until(link, 0, 0, 8.0)

    for north, east in [(15.0, 10.0), (-8.0, 12.0), (0.0, 0.0)]:
        print(f"goto N={north} E={east}")
        wait_until(link, north, east, 8.0)

    print("return to launch")
    link.rtl()
    # Descending 8m takes longer than a fixed sleep: watch telemetry until the
    # vehicle is actually down rather than guessing at a duration.
    deadline = time.time() + 30.0
    msg = None
    while time.time() < deadline:
        msg = link.telemetry(timeout=1.0)
        if msg is None:
            continue
        print(f"    N={msg.x:6.1f} E={msg.y:6.1f} alt={-msg.z:5.1f}", end="\r")
        if -msg.z < 0.5:
            break
    print()
    if msg:
        print(f"    landed N={msg.x:.1f} E={msg.y:.1f} alt={-msg.z:.1f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
