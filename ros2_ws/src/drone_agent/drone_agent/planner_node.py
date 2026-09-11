"""Planner node: turns a plain-English mission into one command at a time.

It waits for the vehicle to settle before planning the next step, so each
command is chosen from the state actually reached rather than the one intended.
It has no direct path to the vehicle — everything goes through the guard.

Subscribes: /drone/mission   (std_msgs/String, plain English)
            /drone/odometry  (nav_msgs/Odometry)
            /drone/guard     (std_msgs/String, the guard's verdict)
Publishes:  /drone/command   (std_msgs/String, JSON)
"""

from __future__ import annotations

import json
import math
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from ._repo import add_flight_package_to_path

add_flight_package_to_path()
from drone.planner import LLMPlanner, ScriptedPlanner, State   # noqa: E402

from .sim_node import SENSOR_QOS                                # noqa: E402

SETTLE_RADIUS = 0.8      # m
SETTLE_SPEED = 0.4       # m/s
SETTLE_TICKS = 12        # consecutive checks before we call it arrived


class PlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("drone_planner")
        self.declare_parameter("use_llm", False)
        self.declare_parameter("model", "gpt-4o-mini")
        self.declare_parameter("max_steps", 8)

        use_llm = self.get_parameter("use_llm").value
        self.planner = (LLMPlanner(self.get_parameter("model").value)
                        if use_llm else ScriptedPlanner())
        if use_llm and not self.planner.available():
            self.get_logger().warn("no API key reachable — using the scripted planner")
            self.planner = ScriptedPlanner()

        self.mission: str | None = None
        self.step = 0
        self.max_steps = self.get_parameter("max_steps").value
        self.last = ""
        self.finished = False
        self.awaiting = False
        self.settled = 0
        self.target = None
        self.state = (0.0, 0.0, 0.0, 0.0, 0.0)     # n, e, d, yaw, speed

        self.cmd_pub = self.create_publisher(String, "/drone/command", 10)
        self.create_subscription(String, "/drone/mission", self.on_mission, 10)
        self.create_subscription(Odometry, "/drone/odometry", self.on_odom, SENSOR_QOS)
        self.create_subscription(String, "/drone/guard", self.on_guard, 10)
        self.create_timer(0.25, self.tick)
        self.get_logger().info(f"planner up ({self.planner.name})")

    def on_mission(self, msg: String) -> None:
        self.mission = msg.data
        self.step = 0
        self.finished = False
        self.awaiting = False
        self.last = ""
        self.get_logger().info(f'mission: "{msg.data}"')

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        yaw = 2.0 * math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        self.state = (p.x, p.y, p.z, yaw, math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2))

    def on_guard(self, msg: String) -> None:
        verdict = json.loads(msg.data).get("verdict")
        if verdict == "rejected":
            # The guard refused it; ask for another rather than waiting forever.
            self.last = "rejected by guard"
            self.awaiting = False

    def arrived(self) -> bool:
        if self.target is None:
            return True
        n, e, d, _, speed = self.state
        gap = math.dist((n, e, d), self.target)
        self.settled = self.settled + 1 if (gap < SETTLE_RADIUS and speed < SETTLE_SPEED) else 0
        return self.settled >= SETTLE_TICKS

    def tick(self) -> None:
        if self.mission is None or self.finished:
            return
        if self.awaiting and not self.arrived():
            return

        self.awaiting = False
        if self.step >= self.max_steps:
            self.get_logger().warn("step budget exhausted")
            self.finished = True
            return

        self.step += 1
        n, e, d, yaw, _ = self.state
        state = State(north=n, east=e, altitude=-d, yaw_deg=math.degrees(yaw),
                      airborne=(-d) > 0.4, step=self.step, last=self.last)
        try:
            raw = self.planner(self.mission, state)
        except RuntimeError as exc:
            self.get_logger().error(f"planner failed: {exc}")
            self.finished = True
            return

        payload = raw if isinstance(raw, str) else json.dumps(raw)
        self.cmd_pub.publish(String(data=payload))

        data = raw if isinstance(raw, dict) else {}
        verb = str(data.get("verb", "")).lower()
        self.last = verb or "sent"
        self.awaiting = True
        self.settled = 0
        # Track roughly where we asked to go, to know when the leg is done.
        if verb in ("goto", "orbit"):
            self.target = (float(data.get("north", n)), float(data.get("east", e)),
                           -float(data.get("altitude", -d)))
        elif verb == "takeoff":
            self.target = (n, e, -float(data.get("altitude", 5.0)))
        elif verb in ("land", "rtl"):
            self.target = (0.0, 0.0, -0.15) if verb == "rtl" else (n, e, -0.15)
            self.finished = True
        else:
            self.target = None


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
