"""Turns a natural-language mission into validated commands, one step at a time.

The planner is deliberately thin. It is handed the vehicle's actual state after
every command and asked for the next one, so it replans against what happened
rather than against what it intended. Everything it emits goes through
commands.parse() before it can reach the controller — the planner has no
privileged path to the vehicle.

Two backends implement the same interface:

    LLMPlanner     talks to an OpenAI-compatible endpoint (including Ollama)
    ScriptedPlanner  a small rule-based stand-in, so the project runs and
                     demos with no API key and no network

Because both return the same JSON, the safety path is identical for both, and
the scripted one doubles as a control when you want to know whether a failure
came from the model or from the flight code.
"""

from __future__ import annotations

import json
import os
import re
import urllib.error
import urllib.request
from dataclasses import dataclass

from .commands import SCHEMA_PROMPT, Envelope

SYSTEM = f"""You are the flight planner for a quadrotor. You are given the
vehicle's current state and a mission in plain English. Emit the single next
command to make progress. When the mission is complete, emit "rtl".

{SCHEMA_PROMPT}
Coordinates are metres from the launch point: +north, +east, altitude up.
"""


@dataclass
class State:
    north: float
    east: float
    altitude: float
    yaw_deg: float
    airborne: bool
    step: int
    last: str = ""

    def describe(self) -> str:
        where = (f"N={self.north:.1f}m E={self.east:.1f}m alt={self.altitude:.1f}m "
                 f"heading={self.yaw_deg:.0f}°")
        status = "airborne" if self.airborne else "on the ground"
        prev = f" Last command: {self.last}." if self.last else ""
        return f"Step {self.step}. Vehicle is {status} at {where}.{prev}"


class ScriptedPlanner:
    """Rule-based planner. Covers the verbs without needing a model.

    It reads a few intents out of the text — an altitude, a waypoint, whether a
    circle was asked for — and sequences them. This is not meant to be clever;
    it exists so the repo runs end to end offline and so flight bugs can be
    isolated from model bugs.
    """

    name = "scripted"

    def __init__(self, env: Envelope | None = None):
        self.env = env or Envelope()
        self._done: set[str] = set()

    @staticmethod
    def _find_altitude(text: str, default: float) -> float:
        m = re.search(r"(\d+(?:\.\d+)?)\s*(?:m|metre|meter)?\s*(?:high|altitude|up|above)", text)
        m = m or re.search(r"(?:at|to)\s+(\d+(?:\.\d+)?)\s*(?:m|metres|meters)\b", text)
        return float(m.group(1)) if m else default

    @staticmethod
    def _find_waypoint(text: str) -> tuple[float, float] | None:
        n = re.search(r"(\d+(?:\.\d+)?)\s*(?:m|metres|meters)?\s*(north|south)", text)
        e = re.search(r"(\d+(?:\.\d+)?)\s*(?:m|metres|meters)?\s*(east|west)", text)
        if not n and not e:
            return None
        north = float(n.group(1)) * (-1 if n and n.group(2) == "south" else 1) if n else 0.0
        east = float(e.group(1)) * (-1 if e and e.group(2) == "west" else 1) if e else 0.0
        return north, east

    def __call__(self, mission: str, state: State) -> dict:
        text = mission.lower()
        alt = self._find_altitude(text, 6.0)

        if not state.airborne:
            return {"verb": "takeoff", "altitude": alt,
                    "reason": "must be airborne before anything else"}

        waypoint = self._find_waypoint(text)
        if waypoint and "goto" not in self._done:
            self._done.add("goto")
            return {"verb": "goto", "north": waypoint[0], "east": waypoint[1],
                    "altitude": alt, "speed": 5.0, "reason": "fly to the requested point"}

        wants_circle = any(w in text for w in ("orbit", "circle", "around", "survey", "inspect"))
        if wants_circle and "orbit" not in self._done:
            self._done.add("orbit")
            centre = waypoint or (state.north, state.east)
            radius = self._find_altitude(text, 6.0) if "radius" in text else 6.0
            return {"verb": "orbit", "north": centre[0], "east": centre[1],
                    "altitude": alt, "radius": radius, "speed": 4.0,
                    "reason": "circle the point of interest"}

        return {"verb": "rtl", "reason": "mission complete, returning"}


class LLMPlanner:
    """Planner backed by any OpenAI-compatible chat endpoint.

    Works against api.openai.com, a local Ollama server, or anything else
    exposing /chat/completions. Reads OPENAI_API_KEY and OPENAI_BASE_URL.
    """

    name = "llm"

    def __init__(self, model: str = "gpt-4o-mini", env: Envelope | None = None,
                 base_url: str | None = None, api_key: str | None = None,
                 timeout: float = 30.0):
        self.model = model
        self.env = env or Envelope()
        self.base_url = (base_url or os.environ.get("OPENAI_BASE_URL")
                         or "https://api.openai.com/v1").rstrip("/")
        self.api_key = api_key or os.environ.get("OPENAI_API_KEY", "")
        self.timeout = timeout
        self.history: list[dict] = []

    def available(self) -> bool:
        return bool(self.api_key) or "localhost" in self.base_url or "127.0.0.1" in self.base_url

    def __call__(self, mission: str, state: State) -> dict:
        messages = [
            {"role": "system", "content": SYSTEM},
            *self.history,
            {"role": "user", "content": f"Mission: {mission}\n{state.describe()}"},
        ]
        body = json.dumps({
            "model": self.model,
            "messages": messages,
            "temperature": 0.1,
            "response_format": {"type": "json_object"},
        }).encode()

        req = urllib.request.Request(
            f"{self.base_url}/chat/completions", data=body,
            headers={"Content-Type": "application/json",
                     "Authorization": f"Bearer {self.api_key}"})
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                payload = json.load(resp)
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            raise RuntimeError(f"planner unreachable: {exc}") from None

        text = payload["choices"][0]["message"]["content"]
        # Keep a short rolling memory so follow-ups like "now come back" work.
        self.history.append({"role": "user", "content": state.describe()})
        self.history.append({"role": "assistant", "content": text})
        self.history = self.history[-8:]
        return text
