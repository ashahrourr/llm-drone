#!/usr/bin/env python3
"""Serve the simulated quadrotor over MAVLink.

    python mavlink_sim.py                       # listen on udp 14550
    python mavlink_sim.py --endpoint udpin:0.0.0.0:14540

Anything that speaks MAVLink can then fly it — QGroundControl, MAVSDK,
mavproxy, or `demos/mavlink_client.py` in this repo. The vehicle accepts
SET_POSITION_TARGET_LOCAL_NED and the usual arm/takeoff/land/RTL commands, and
streams LOCAL_POSITION_NED and ATTITUDE back.
"""

from __future__ import annotations

import argparse
import time

import numpy as np

from drone.control import Controller
from drone.dynamics import Params, Quadrotor
from drone.mavlink_io import MavlinkVehicle

RATE_HZ = 200.0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--endpoint", default="udpin:0.0.0.0:14550")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)

    params = Params()
    drone = Quadrotor(params)
    control = Controller(params)
    vehicle = MavlinkVehicle(args.endpoint)

    print(f"vehicle listening on {args.endpoint} at {RATE_HZ:.0f} Hz")
    print("waiting for a ground station...")

    dt = 1.0 / RATE_HZ
    next_tick = time.time()
    try:
        while True:
            for note in vehicle.poll():
                if not args.quiet:
                    print(f"  {note}")

            sp = vehicle.setpoint
            if vehicle.armed:
                target = np.array([sp.north, sp.east, sp.down])
                rotors = control(drone.state, target, sp.yaw, dt)
            else:
                rotors = np.zeros(4)
            drone.step(rotors, dt)
            vehicle.pump(drone.position, drone.velocity, drone.euler)

            # Keep wall-clock pace so telemetry rates mean something.
            next_tick += dt
            slack = next_tick - time.time()
            if slack > 0:
                time.sleep(slack)
            else:
                next_tick = time.time()
    except KeyboardInterrupt:
        print("\nvehicle stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
