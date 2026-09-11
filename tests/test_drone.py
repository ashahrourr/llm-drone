"""Tests for the physics, the controller and the command guard.

The physics tests check against closed-form answers rather than against
recorded output, so they catch a wrong sign or a broken mixer instead of just
noticing that behaviour changed.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone.commands import Envelope, Rejected, parse          # noqa: E402
from drone.control import Controller                          # noqa: E402
from drone.dynamics import GRAVITY, Params, Quadrotor, rotation_matrix  # noqa: E402
from drone.mission import Mission                             # noqa: E402
from drone.planner import ScriptedPlanner, State              # noqa: E402

DT = 0.005


def fly(quad, ctrl, target, yaw=0.0, seconds=12.0):
    for _ in range(int(seconds / DT)):
        quad.step(ctrl(quad.state, np.asarray(target, float), yaw, DT), DT)
    return quad


# ---------------------------------------------------------------- physics
def test_hover_holds_altitude():
    q = Quadrotor(position=(0, 0, -10))
    w = q.hover_rotor_speed()
    for _ in range(1000):
        q.step(np.full(4, w), DT)
    assert q.altitude == pytest.approx(10.0, abs=1e-3)
    assert abs(q.velocity[2]) < 1e-3


def test_free_fall_matches_kinematics():
    """With drag off, one second of fall must be g/2 metres."""
    q = Quadrotor(Params(drag=0.0), position=(0, 0, -50))
    for _ in range(int(1.0 / DT)):
        q.step(np.zeros(4), DT)
    assert 50.0 - q.altitude == pytest.approx(0.5 * GRAVITY, rel=1e-3)


def test_drag_slows_the_fall():
    plain = Quadrotor(Params(drag=0.0), position=(0, 0, -50))
    dragged = Quadrotor(Params(drag=0.5), position=(0, 0, -50))
    for _ in range(int(2.0 / DT)):
        plain.step(np.zeros(4), DT)
        dragged.step(np.zeros(4), DT)
    assert dragged.altitude > plain.altitude


@pytest.mark.parametrize("axis,moment", [
    (0, np.array([0.02, 0.0, 0.0])),
    (1, np.array([0.0, 0.02, 0.0])),
    (2, np.array([0.0, 0.0, 0.01])),
])
def test_mixer_axes_are_independent(axis, moment):
    """Inverting the mixer must reproduce the demand on one axis only.

    This is the test that would have caught the original layout, where pitch
    and yaw shared a rotor combination and pitch moment was always zero.
    """
    p = Params()
    q, c = Quadrotor(p), Controller(p)
    speeds = c.to_rotor_speeds(9.0, moment)
    thrust, produced = q.mix(speeds)
    assert thrust == pytest.approx(9.0, rel=1e-6)
    assert produced[axis] == pytest.approx(moment[axis], rel=1e-6)
    for other in {0, 1, 2} - {axis}:
        assert abs(produced[other]) < 1e-9


def test_tilt_maps_to_the_expected_direction():
    """+roll accelerates east, +pitch accelerates south (NED)."""
    thrust = np.array([0.0, 0.0, -10.0])
    east = rotation_matrix(math.radians(10), 0.0, 0.0) @ thrust
    south = rotation_matrix(0.0, math.radians(10), 0.0) @ thrust
    assert east[1] > 0.5 and abs(east[0]) < 1e-9
    assert south[0] < -0.5 and abs(south[1]) < 1e-9


def test_ground_is_solid():
    q = Quadrotor(position=(0, 0, -0.2))
    for _ in range(400):
        q.step(np.zeros(4), DT)
    assert q.altitude >= 0.0
    assert q.velocity[2] <= 0.0


# ---------------------------------------------------------------- control
def test_reaches_setpoint_in_all_axes():
    p = Params()
    q = fly(Quadrotor(p), Controller(p), [8.0, 6.0, -5.0])
    assert np.linalg.norm(q.position - np.array([8.0, 6.0, -5.0])) < 0.6


def test_holds_commanded_yaw_while_translating():
    """Yaw must not drift when the vehicle is moving — it did, badly, when the
    attitude loop was too slow relative to the position loop."""
    p = Params()
    q, c = Quadrotor(p), Controller(p)
    fly(q, c, [0, 0, -5.0])
    fly(q, c, [12.0, 9.0, -5.0])
    assert abs(math.degrees(q.euler[2])) < 5.0


def test_attitude_loop_is_a_decade_above_position_loop():
    c = Controller(Params())
    w_att = math.sqrt(c.att_gain[0] / Params().inertia[0])
    assert w_att > 10 * 1.2          # position loop sits near 1.2 rad/s


# --------------------------------------------------------------- commands
def test_parses_json_wrapped_in_prose_and_fences():
    cmd = parse('Sure!\n```json\n{"verb":"takeoff","altitude":"6m"}\n```')
    assert cmd.verb == "takeoff" and cmd.altitude == pytest.approx(6.0)


def test_clamps_instead_of_failing():
    cmd = parse('{"verb":"goto","north":500,"east":0,"altitude":200,"speed":99}')
    env = Envelope()
    assert cmd.altitude == env.max_altitude
    assert math.hypot(cmd.north, cmd.east) == pytest.approx(env.geofence_radius)
    assert cmd.speed == env.max_speed
    assert len(cmd.clamped) == 3


def test_geofence_clamp_preserves_bearing():
    cmd = parse('{"verb":"goto","north":300,"east":400}')       # bearing 3-4-5
    assert cmd.north / cmd.east == pytest.approx(0.75, rel=1e-6)


@pytest.mark.parametrize("bad", [
    '{"verb":"selfdestruct"}',
    '{"verb":"goto","north":null}',
    '{"verb":"goto","north":"NaN"}',
    '{"verb":"goto","north":true}',
    "not json at all",
    '{"verb":"goto"',
])
def test_rejects_unsafe_or_malformed(bad):
    with pytest.raises(Rejected):
        parse(bad)


def test_yaw_wraps_into_range():
    assert parse('{"verb":"goto","yaw":370}').yaw == pytest.approx(math.radians(10))
    assert parse('{"verb":"goto","yaw":-190}').yaw == pytest.approx(math.radians(170))


# ---------------------------------------------------------------- mission
def test_mission_takeoff_then_land_returns_to_ground():
    m = Mission()
    m.execute(parse('{"verb":"takeoff","altitude":5}'))
    assert m.drone.altitude > 4.0
    m.execute(parse('{"verb":"land"}'))
    assert m.drone.altitude < 0.5


def test_rtl_comes_home():
    m = Mission()
    m.execute(parse('{"verb":"takeoff","altitude":6}'))
    m.execute(parse('{"verb":"goto","north":12,"east":9,"altitude":6}'))
    m.execute(parse('{"verb":"rtl"}'))
    assert np.linalg.norm(m.drone.position[:2]) < 1.0
    assert m.drone.altitude < 0.5


def test_scripted_planner_sequences_a_mission():
    p = ScriptedPlanner()
    grounded = State(0, 0, 0, 0, airborne=False, step=1)
    assert p("fly 10m north and circle it", grounded)["verb"] == "takeoff"
    flying = State(0, 0, 6, 0, airborne=True, step=2)
    assert p("fly 10m north and circle it", flying)["verb"] == "goto"
    assert p("fly 10m north and circle it", flying)["verb"] == "orbit"
    assert p("fly 10m north and circle it", flying)["verb"] == "rtl"
