"""Executes validated commands and records the flight.

The executor owns the control loop. It turns one Command into a sequence of
setpoints, runs the vehicle until the command is satisfied or times out, and
reports what happened so the planner can decide the next step from the actual
state rather than from what it hoped would happen.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .commands import Command, Envelope
from .control import Controller
from .dynamics import Params, Quadrotor

DT = 0.005              # 200 Hz inner loop
ARRIVAL_RADIUS = 0.6    # m
SETTLE_SPEED = 0.4      # m/s — must also be slow to count as arrived


@dataclass
class Outcome:
    verb: str
    ok: bool
    detail: str
    position: tuple[float, float, float]
    elapsed: float
    clamped: list[str] = field(default_factory=list)

    def summary(self) -> str:
        n, e, d = self.position
        state = "ok" if self.ok else "TIMEOUT"
        note = f"  (clamped: {', '.join(self.clamped)})" if self.clamped else ""
        return (f"{self.verb:8} {state:8} {self.detail:28} "
                f"N={n:6.1f} E={e:6.1f} alt={-d:5.1f}  {self.elapsed:5.1f}s{note}")


class Mission:
    """Runs commands against the vehicle and logs the trajectory."""

    def __init__(self, envelope: Envelope | None = None, params: Params | None = None):
        self.params = params or Params()
        self.env = envelope or Envelope()
        self.drone = Quadrotor(self.params)
        self.control = Controller(self.params)
        self.trail: list[np.ndarray] = [self.drone.position.copy()]
        self.events: list[tuple[int, str]] = []     # (trail index, label)
        self.outcomes: list[Outcome] = []
        self.time = 0.0

    # ---- low-level ------------------------------------------------------
    def _fly_to(self, target: np.ndarray, yaw: float, timeout: float) -> tuple[bool, float]:
        """Hold a setpoint until reached and settled, or until timeout."""
        start = self.time
        while self.time - start < timeout:
            self.drone.step(self.control(self.drone.state, target, yaw, DT), DT)
            self.time += DT
            self.trail.append(self.drone.position.copy())
            close = np.linalg.norm(self.drone.position - target) < ARRIVAL_RADIUS
            slow = np.linalg.norm(self.drone.velocity) < SETTLE_SPEED
            if close and slow:
                return True, self.time - start
        return False, self.time - start

    # ---- verbs ----------------------------------------------------------
    def execute(self, cmd: Command) -> Outcome:
        pos = self.drone.position
        label = cmd.verb

        if cmd.verb == "takeoff":
            target = np.array([pos[0], pos[1], -cmd.altitude])
            ok, t = self._fly_to(target, cmd.yaw, timeout=20.0)
            detail = f"climb to {cmd.altitude:.1f}m"

        elif cmd.verb == "goto":
            target = np.array(cmd.target_ned())
            # Long legs get proportionally more time, at the commanded speed.
            dist = float(np.linalg.norm(target - pos))
            ok, t = self._fly_to(target, cmd.yaw, timeout=max(14.0, 3.0 * dist / cmd.speed))
            detail = f"{dist:.1f}m leg"

        elif cmd.verb == "orbit":
            ok, t = self._orbit(cmd)
            detail = f"r={cmd.radius:.1f}m circle"

        elif cmd.verb == "hold":
            target = pos.copy()
            ok, t = self._fly_to(target, cmd.yaw, timeout=3.0)
            ok, detail = True, "station keeping"

        elif cmd.verb in ("land", "rtl"):
            if cmd.verb == "rtl":
                # Climb to a safe transit height, return, then descend.
                self._fly_to(np.array([pos[0], pos[1], -8.0]), cmd.yaw, timeout=10.0)
                self._fly_to(np.array([0.0, 0.0, -8.0]), cmd.yaw, timeout=25.0)
            here = self.drone.position
            ok, t = self._fly_to(np.array([here[0], here[1], -0.15]), cmd.yaw, timeout=18.0)
            detail = "touchdown"
            label = cmd.verb

        else:                                        # unreachable: parse() gates this
            raise ValueError(f"unhandled verb {cmd.verb}")

        self.events.append((len(self.trail) - 1, label))
        outcome = Outcome(cmd.verb, ok, detail, tuple(self.drone.position), t, cmd.clamped)
        self.outcomes.append(outcome)
        return outcome

    def _orbit(self, cmd: Command) -> tuple[bool, float]:
        """Fly a circle around a centre, nose pointed inward."""
        start = self.time
        centre = np.array([cmd.north, cmd.east])
        # Enter at the nearest point on the circle to avoid a dash across it.
        here = self.drone.position[:2]
        offset = here - centre
        phase = math.atan2(offset[1], offset[0]) if np.linalg.norm(offset) > 1e-3 else 0.0
        entry = np.array([centre[0] + cmd.radius * math.cos(phase),
                          centre[1] + cmd.radius * math.sin(phase),
                          -cmd.altitude])
        self._fly_to(entry, cmd.yaw, timeout=20.0)

        # One lap, at the commanded tangential speed.
        laptime = 2 * math.pi * cmd.radius / cmd.speed
        steps = int(laptime / DT)
        for i in range(steps):
            a = phase + 2 * math.pi * i / steps
            target = np.array([centre[0] + cmd.radius * math.cos(a),
                               centre[1] + cmd.radius * math.sin(a),
                               -cmd.altitude])
            # Face the centre so a camera would stay on the subject.
            yaw = math.atan2(centre[1] - target[1], centre[0] - target[0])
            self.drone.step(self.control(self.drone.state, target, yaw, DT), DT)
            self.time += DT
            self.trail.append(self.drone.position.copy())
        return True, self.time - start

    def path(self) -> np.ndarray:
        return np.array(self.trail)
