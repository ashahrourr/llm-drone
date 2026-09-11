#!/usr/bin/env python3
"""Fly a mission given in plain English.

    python fly.py "take off to 8m, fly 15m north and 10m east, circle it, come back"
    python fly.py --llm "survey the area 20m north then return"
    python fly.py "..." --video docs/flight.mp4 --plot docs/trajectory.png

Without --llm a small rule-based planner is used, so this runs with no API key
and no network. With --llm it talks to any OpenAI-compatible endpoint
(OPENAI_API_KEY / OPENAI_BASE_URL), including a local Ollama server.
"""

from __future__ import annotations

import argparse
import math
import sys

from drone.commands import Envelope, Rejected, parse
from drone.mission import Mission
from drone.planner import LLMPlanner, ScriptedPlanner, State

MAX_STEPS = 8


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Fly a natural-language mission.")
    ap.add_argument("mission", help="what the drone should do, in plain English")
    ap.add_argument("--llm", action="store_true", help="use an LLM planner instead of the scripted one")
    ap.add_argument("--model", default="gpt-4o-mini")
    ap.add_argument("--video", metavar="PATH", help="render an animation here (.mp4 or .gif)")
    ap.add_argument("--plot", metavar="PATH", help="save a static trajectory plot here")
    ap.add_argument("--seconds", type=float, default=13.0, help="video length")
    ap.add_argument("--max-steps", type=int, default=MAX_STEPS)
    args = ap.parse_args(argv)

    env = Envelope()
    planner = LLMPlanner(args.model, env) if args.llm else ScriptedPlanner(env)

    if args.llm and not planner.available():
        print("No OPENAI_API_KEY set and no local endpoint — falling back to the "
              "scripted planner.\n", file=sys.stderr)
        planner = ScriptedPlanner(env)

    mission = Mission(env)
    print(f"mission : {args.mission}")
    print(f"planner : {planner.name}\n")

    last = ""
    for step in range(1, args.max_steps + 1):
        pos = mission.drone.position
        state = State(north=float(pos[0]), east=float(pos[1]),
                      altitude=float(-pos[2]),
                      yaw_deg=math.degrees(float(mission.drone.euler[2])),
                      airborne=float(-pos[2]) > 0.4, step=step, last=last)

        try:
            raw = planner(args.mission, state)
        except RuntimeError as exc:
            print(f"  planner failed: {exc}", file=sys.stderr)
            return 2

        # Every command crosses the same validation boundary, model or not.
        try:
            cmd = parse(raw, env)
        except Rejected as exc:
            # A rejected command is not flown. Tell the planner why and let it
            # try again; that feedback is the whole point of rejecting rather
            # than silently clamping.
            print(f"  rejected: {exc}")
            last = f"rejected ({exc})"
            continue

        outcome = mission.execute(cmd)
        print("  " + outcome.summary())
        if cmd.reason:
            print(f"           reason: {cmd.reason}")
        last = cmd.verb

        if cmd.verb in ("land", "rtl"):
            break

    print(f"\nflight time {mission.time:.1f}s over {len(mission.outcomes)} commands")

    if args.plot:
        from drone.viz import plot_trajectory
        print("wrote", plot_trajectory(mission, args.plot, args.mission[:60]))
    if args.video:
        from drone.viz import animate
        print("wrote", animate(mission, args.video, seconds=args.seconds,
                               title=args.mission[:60]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
