# llm-drone

**Fly a quadrotor by describing the mission in English — with the model kept behind a safety guard.**

![Quadrotor flying a takeoff, transit, orbit and return-to-launch](docs/flight.gif)

```bash
python fly.py "take off to 8m, fly 15m north and 10m east, circle it, then come home"
```

```
takeoff  ok   climb to 8.0m     N=  0.0 E=  0.0 alt= 8.6   4.8s
goto     ok   18.0m leg         N= 14.8 E= 10.0 alt= 8.2   5.1s
orbit    ok   r=6.0m circle     N=  8.1 E= 10.5 alt= 8.0  13.2s
rtl      ok   touchdown         N= -0.1 E=  0.0 alt=-0.0   2.5s
```

---

## The problem

Language models are good at turning "survey the north field and come back" into a
plan. They are bad at everything that makes flight safe — they hallucinate
coordinates, emit `NaN`, invent units, and confidently produce a waypoint two
kilometres away.

Hand one a tool that moves the vehicle and a bad token reaches an actuator, with
no layer whose job is to say no.

## The solution

The model emits **one JSON object from six verbs**, and it crosses a guard before
anything moves:

```jsonc
{"verb": "goto", "north": 15, "east": 10, "altitude": 8, "speed": 5}
```

- **Malformed → rejected.** Unknown verb, `null`, `NaN`, unparseable JSON. Not
  flown; the reason goes back to the planner to try again.
- **Out of range → clamped.** A 500 m waypoint is pulled onto the geofence along
  its own bearing, so the intended direction survives.
- **Unsafe is unrepresentable.** No verb descends below ground; no waypoint
  escapes the fence.

Real output, hostile commands published straight at the guard:

```
goto → N=15.0 E=10.0 alt=8.0
goto: clamped altitude→30.0m ceiling, waypoint→60.0m geofence
rejected: unknown verb 'descend_below_ground'
rejected: north must be a number, got None
rejected: no JSON object in model output
```

## Three ways to run it

**Standalone** — numpy only, no ROS:

```bash
pip install -r requirements.txt
python fly.py "take off to 6m and circle 10m north of here"
```

**As a ROS 2 graph** — planner → guard → vehicle, each its own node:

```bash
ros2 launch drone_agent mission.launch.py mission:="fly 20m north and come back"
```

The guard being a separate node is the point: the safety rule is enforced by a
process the planner cannot bypass, and every verdict lands on `/drone/guard`.

**Against real PX4 firmware** — EKF2, commander, the real position controller:

```bash
./px4/build.sh && ./px4/fly.sh
```

Verified on an Apple Silicon Mac — armed, climbed to 10.05 m, translated to
N=20.3 E=15.4, returned, landed. No Gazebo needed: PX4's built-in SIH integrates
its own rigid-body model at 250 Hz, which is also what makes it run on arm64.
Notes in [px4/README.md](px4/README.md).

## What's in it

| | |
|---|---|
| `drone/dynamics.py` | 6-DOF rigid body in NED, RK4, X-quad mixer from `r × F` |
| `drone/control.py` | cascaded PID: position → attitude → rotor speeds |
| `drone/commands.py` | the schema and the guard |
| `drone/planner.py` | LLM backend (OpenAI-compatible / Ollama) + an offline one |
| `drone/mavlink_io.py` | MAVLink vehicle and ground station |
| `ros2_ws/` | the three nodes, launch file, integration test |
| `px4/` | build and fly real PX4 |
| `tests/` | 29 tests, checked against closed-form physics |

```bash
pytest tests/ -q                   # 29 passed
bash ros2_ws/test_graph.sh         # guard verdicts over real topics
```

Running real missions needs an LLM configured for `--llm`; without a key it uses
the offline planner, so everything above runs with no network.

![Trajectory](docs/trajectory.png)

<sub>Prior art: the LLM-commands-a-drone framing follows
[pratikPhadte/LLM-controlled-drone](https://github.com/pratikPhadte/LLM-controlled-drone).
This is an independent implementation.</sub>
