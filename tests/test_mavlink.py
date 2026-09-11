"""MAVLink round-trip: a ground station commanding the simulated vehicle.

Runs the vehicle and the link in one process over loopback UDP, which keeps the
test hermetic while still exercising real MAVLink encode/decode rather than a
mock.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone.control import Controller                       # noqa: E402
from drone.dynamics import Params, Quadrotor               # noqa: E402
from drone.mavlink_io import MavlinkLink, MavlinkVehicle   # noqa: E402

PORT = 14577          # off the default so a running sim does not interfere
DT = 1.0 / 200.0


@pytest.fixture
def rig():
    vehicle = MavlinkVehicle(f"udpin:127.0.0.1:{PORT}")
    link = MavlinkLink(f"udpout:127.0.0.1:{PORT}")
    params = Params()
    drone = Quadrotor(params)
    control = Controller(params)
    yield vehicle, link, drone, control
    vehicle.conn.close()
    link.conn.close()


def spin(vehicle, drone, control, seconds: float) -> None:
    for _ in range(int(seconds / DT)):
        vehicle.poll()
        if vehicle.armed:
            sp = vehicle.setpoint
            rotors = control(drone.state,
                             np.array([sp.north, sp.east, sp.down]), sp.yaw, DT)
        else:
            rotors = np.zeros(4)
        drone.step(rotors, DT)
        vehicle.pump(drone.position, drone.velocity, drone.euler)


def test_heartbeat_is_discoverable(rig):
    """The GCS must speak first.

    The vehicle binds with udpin, so it has no return address until a packet
    arrives from the ground station. A GCS that only listens will wait forever
    — which is why MavlinkLink.wait_heartbeat sends its own heartbeat before
    blocking for the vehicle's.
    """
    vehicle, link, drone, control = rig
    link.conn.mav.heartbeat_send(6, 8, 0, 0, 0)      # MAV_TYPE_GCS
    for _ in range(40):
        vehicle.poll()                                # learns the peer address
        vehicle.send_heartbeat()
        if link.conn.recv_match(type="HEARTBEAT", blocking=False):
            return
        time.sleep(0.02)
    pytest.fail("no heartbeat reached the ground station")


def test_arm_and_takeoff_climbs(rig):
    vehicle, link, drone, control = rig
    link.arm()
    link.takeoff(8.0)
    spin(vehicle, drone, control, 12.0)
    assert vehicle.armed
    assert drone.altitude > 7.0


def test_position_target_is_tracked(rig):
    vehicle, link, drone, control = rig
    link.arm()
    link.takeoff(6.0)
    spin(vehicle, drone, control, 8.0)
    link.goto(12.0, 8.0, 6.0)
    spin(vehicle, drone, control, 14.0)
    assert np.linalg.norm(drone.position[:2] - np.array([12.0, 8.0])) < 1.5


def test_disarmed_vehicle_does_not_fly(rig):
    vehicle, link, drone, control = rig
    link.goto(20.0, 20.0, 10.0)      # never armed
    spin(vehicle, drone, control, 4.0)
    assert drone.altitude < 0.1


def test_rtl_returns_and_descends(rig):
    vehicle, link, drone, control = rig
    link.arm()
    link.takeoff(8.0)
    spin(vehicle, drone, control, 10.0)
    link.goto(14.0, 10.0, 8.0)
    spin(vehicle, drone, control, 14.0)
    link.rtl()
    spin(vehicle, drone, control, 22.0)
    assert np.linalg.norm(drone.position[:2]) < 1.5
    assert drone.altitude < 1.0
