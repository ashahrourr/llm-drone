"""The command schema the model is allowed to emit, and the guard that checks it.

A language model cannot be trusted to emit safe flight commands, so it is never
given the controller directly. It emits one JSON object from a closed set of
verbs, and every field is validated and clamped against the flight envelope
before anything reaches the vehicle. Commands that cannot be made safe are
rejected with a reason, which is fed back to the model rather than flown.

The point is that an unsafe command is *unrepresentable*: there is no verb for
"descend below ground" and no way to express a waypoint outside the geofence.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from typing import Any

VERBS = ("takeoff", "goto", "orbit", "hold", "land", "rtl")


@dataclass
class Envelope:
    """Hard limits. Nothing outside these ever reaches the controller."""

    max_altitude: float = 30.0      # m above launch
    min_altitude: float = 0.5       # m, below this only `land` is legal
    geofence_radius: float = 60.0   # m from launch
    max_speed: float = 8.0          # m/s
    max_orbit_radius: float = 25.0  # m


@dataclass
class Command:
    verb: str
    north: float = 0.0
    east: float = 0.0
    altitude: float = 5.0
    yaw: float = 0.0
    speed: float = 4.0
    radius: float = 5.0
    reason: str = ""
    clamped: list[str] = field(default_factory=list)

    def target_ned(self) -> tuple[float, float, float]:
        return self.north, self.east, -self.altitude


class Rejected(Exception):
    """Raised when a command cannot be made safe by clamping."""


def _number(raw: Any, name: str) -> float:
    """Coerce a JSON value to a finite float, or reject it.

    Models emit "12", 12, "12m" and null more or less interchangeably, and a
    NaN that reaches the controller propagates into the state vector and is
    very hard to trace back. Catching it here is cheap.
    """
    if isinstance(raw, bool) or raw is None:
        raise Rejected(f"{name} must be a number, got {raw!r}")
    if isinstance(raw, str):
        raw = raw.strip().rstrip("m").strip()
    try:
        value = float(raw)
    except (TypeError, ValueError):
        raise Rejected(f"{name} must be a number, got {raw!r}") from None
    if not math.isfinite(value):
        raise Rejected(f"{name} must be finite, got {value}")
    return value


def parse(raw: str | dict, envelope: Envelope | None = None) -> Command:
    """Parse and validate one model-emitted command.

    Raises Rejected for anything structurally wrong; clamps anything that is
    merely out of range and records what was clamped.
    """
    env = envelope or Envelope()

    if isinstance(raw, str):
        text = raw.strip()
        # Models like to wrap JSON in prose or ``` fences. Take the outermost
        # object rather than failing on the packaging.
        start, end = text.find("{"), text.rfind("}")
        if start == -1 or end <= start:
            raise Rejected("no JSON object in model output")
        try:
            data = json.loads(text[start:end + 1])
        except json.JSONDecodeError as exc:
            raise Rejected(f"invalid JSON: {exc.msg}") from None
    elif isinstance(raw, dict):
        data = raw
    else:
        raise Rejected(f"expected str or dict, got {type(raw).__name__}")

    verb = str(data.get("verb", "")).strip().lower()
    if verb not in VERBS:
        raise Rejected(f"unknown verb {verb!r}; expected one of {', '.join(VERBS)}")

    cmd = Command(verb=verb, reason=str(data.get("reason", ""))[:200])

    if verb in ("takeoff", "goto", "orbit"):
        cmd.altitude = _number(data.get("altitude", 5.0), "altitude")
        if cmd.altitude > env.max_altitude:
            cmd.altitude = env.max_altitude
            cmd.clamped.append(f"altitude→{env.max_altitude}m ceiling")
        if cmd.altitude < env.min_altitude:
            cmd.altitude = env.min_altitude
            cmd.clamped.append(f"altitude→{env.min_altitude}m floor")

    if verb in ("goto", "orbit"):
        cmd.north = _number(data.get("north", 0.0), "north")
        cmd.east = _number(data.get("east", 0.0), "east")
        # Pull the waypoint back onto the geofence along its own bearing,
        # rather than rejecting: the intended direction is still honoured.
        dist = math.hypot(cmd.north, cmd.east)
        if dist > env.geofence_radius:
            scale = env.geofence_radius / dist
            cmd.north *= scale
            cmd.east *= scale
            cmd.clamped.append(f"waypoint→{env.geofence_radius}m geofence")

    if verb == "orbit":
        cmd.radius = _number(data.get("radius", 5.0), "radius")
        if not 1.0 <= cmd.radius <= env.max_orbit_radius:
            cmd.radius = min(max(cmd.radius, 1.0), env.max_orbit_radius)
            cmd.clamped.append(f"radius→{cmd.radius}m")

    if "yaw" in data:
        yaw_deg = _number(data.get("yaw"), "yaw")
        cmd.yaw = math.radians((yaw_deg + 180.0) % 360.0 - 180.0)

    cmd.speed = _number(data.get("speed", 4.0), "speed")
    if not 0.5 <= cmd.speed <= env.max_speed:
        cmd.speed = min(max(cmd.speed, 0.5), env.max_speed)
        cmd.clamped.append(f"speed→{cmd.speed}m/s")

    return cmd


SCHEMA_PROMPT = f"""Emit exactly one JSON object. No prose, no code fences.

{{"verb": <one of {list(VERBS)}>,
  "north": <metres, +north>,   "east": <metres, +east>,
  "altitude": <metres above launch>, "yaw": <degrees, 0=north>,
  "speed": <m/s>, "radius": <metres, orbit only>,
  "reason": "<one short sentence>"}}

Only the fields the verb needs. Limits (exceeding them clamps, it does not
fail): altitude 0.5–30 m, within 60 m of launch, speed 0.5–8 m/s,
orbit radius 1–25 m.
"""
