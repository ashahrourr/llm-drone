# llm-drone

**Fly a quadrotor by describing the mission in English — where the model is never allowed to touch the controls.**

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
plan. They are bad at everything that makes flight safe: they hallucinate
coordinates, emit `NaN`, invent units, and confidently produce a waypoint two
kilometres away.

The usual approach hands the model a tool that moves the vehicle. That means one
bad token reaches an actuator, and there is no layer whose job is to say no.

## The solution

The model never touches the controller. It emits **one JSON object from a closed
set of six verbs**, and everything it produces crosses a validation boundary
before anything moves:

```jsonc
{"verb": "goto", "north": 15, "east": 10, "altitude": 8, "speed": 5,
 "reason": "fly to the requested point"}
```

- **Structurally invalid → rejected.** Unknown verb, `null`, `NaN`, a boolean
  where a number belongs, unparseable JSON. The command is not flown, and the
  reason goes back to the planner so it can try again.
- **Out of range → clamped, not failed.** 200 m altitude becomes the 30 m
  ceiling; a 500 m waypoint is pulled back onto the geofence *along its own
  bearing*, so the intended direction survives. Every clamp is reported.
- **Unsafe is unrepresentable.** There is no verb for "descend below ground" and
  no way to express a waypoint outside the fence.

The planner is also handed the vehicle's **actual** state after every command, so
it replans against what happened rather than what it hoped would happen.

## Two ways to run it

**As a ROS 2 graph**, which is how it would run on a real vehicle:

```
/drone/mission   (English)      →  planner_node
/drone/command   (JSON)         →  guard_node      ← validates, clamps, rejects
/drone/setpoint  (PoseStamped)  →  sim_node        ← 200 Hz dynamics + control
/drone/odometry  (Odometry)     →  back to planner and guard
```

```bash
ros2 launch drone_agent mission.launch.py \
    mission:="take off to 8m, fly 15m north and 10m east, circle it, come home"
```

The guard is its own node on purpose: the safety rule is enforced by a process
the planner cannot bypass, and every verdict is published on `/drone/guard`
where you can `ros2 topic echo` it mid-flight.

`sim_node` publishes `nav_msgs/Odometry` in NED with PX4's best-effort sensor
QoS, so **replacing it with PX4 SITL is a launch-file change, not a rewrite** —
the topics, the frame conventions and the guard all stay put.

**Or standalone**, with no ROS at all, for quick iteration and for rendering:

```bash
python fly.py "take off to 6m and circle 10m north of here"
```

Both paths import the same flight code.

| | |
|---|---|
| `drone/dynamics.py` | 6-DOF rigid body in NED, RK4 integration, X-quad mixer derived from `r × F` |
| `drone/control.py` | cascaded PID: position → acceleration → attitude → body moments → rotor speeds |
| `drone/commands.py` | the schema and the guard described above |
| `drone/mission.py` | executes verbs, detects arrival, logs the trajectory |
| `drone/planner.py` | LLM backend (OpenAI-compatible / Ollama) + an offline rule-based one |
| `drone/viz.py` | 3-D animation and trajectory plots |
| `ros2_ws/` | the three nodes, launch file, and a graph-level integration test |
| `tests/` | 24 tests, checked against closed-form physics |

### The guard, over real topics

Publishing hostile commands straight at `/drone/command`:

```
goto → N=15.0 E=10.0 alt=8.0
goto: clamped altitude→30.0m ceiling, waypoint→60.0m geofence
rejected: unknown verb 'descend_below_ground'; expected one of takeoff, goto, orbit, hold, land, rtl
rejected: north must be a number, got None
rejected: no JSON object in model output
```

A 5 km waypoint at 900 m becomes a legal one on the fence at the ceiling. The
three malformed commands never become setpoints. `ros2_ws/test_graph.sh`
asserts all of it.

## Three bugs worth keeping

**The tilt signs were inverted.** Commanded east, it flew 268 m west. Rather than
guess, the fix came from evaluating the rotation matrix: `+roll → +east`,
`+pitch → −north`. There's a test pinning it now.

**The mixer was degenerate.** Pitch and yaw were computed from the same rotor
combination, so pitch moment was *always exactly zero* and the vehicle could
only move on one axis. Re-deriving the layout from `r × F` made all four axes
independent — `test_mixer_axes_are_independent` is the test that would have
caught it.

**The control loops were fighting.** Hand-tuned attitude gains put the inner
loop at 3.35 rad/s and the outer at 1.18 rad/s. A cascade assumes the inner loop
is roughly a decade faster; at 3× they interfere, and the drone oscillated and
yawed 50° off heading. The attitude gains are now computed from the inertia
(`kp = I·ω²`, `kd = 2ζ√(kp·I)`) instead of guessed.

## Run it

No Ubuntu needed — the ROS 2 image runs natively on Apple Silicon:

```bash
docker run --rm -it -v "$PWD":/work -w /work/ros2_ws -e DRONE_REPO=/work \
    ros:jazzy-ros-base bash
# inside:
source /opt/ros/jazzy/setup.bash && colcon build --packages-select drone_agent
source install/setup.bash
ros2 launch drone_agent mission.launch.py mission:="fly 20m north and come back"

bash test_graph.sh      # graph-level integration test
```

Standalone, without ROS:

```bash
pip install numpy matplotlib
python fly.py "take off to 6m and circle 10m north of here"

# save the flight
python fly.py "..." --video docs/flight.mp4 --plot docs/trajectory.png

# use a real model instead of the offline planner
export OPENAI_API_KEY=sk-...
python fly.py --llm "survey the area 20m north, then come home"
```

`--llm` works against any OpenAI-compatible endpoint — set `OPENAI_BASE_URL` to
point at a local Ollama server. Without a key it falls back to the rule-based
planner, so the repo runs and demos offline. Both backends emit the same JSON
and cross the same guard, which makes the scripted one a useful control when you
want to know whether a failure came from the model or from the flight code.

```bash
pytest tests/ -q      # 24 passed
```

![Trajectory](docs/trajectory.png)

<sub>Prior art: the LLM-commands-a-drone framing follows
[pratikPhadte/LLM-controlled-drone](https://github.com/pratikPhadte/LLM-controlled-drone),
which does it on ROS 2 + PX4 + Gazebo. This is an independent implementation:
the dynamics, control, command schema and guard are written here, and the graph
runs without PX4 or Gazebo so it works on any machine.</sub>
