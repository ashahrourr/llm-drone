"""Guard node: the only path from planner to vehicle.

Every command the planner emits is validated here before it can become a
setpoint. Structurally invalid commands are rejected and the reason is
published back; out-of-range values are clamped and the clamp is reported.

Keeping this in its own node rather than inside the planner is deliberate —
the safety rule is enforced by a process the planner cannot bypass, and the
rejections are visible on a topic you can echo while it flies.

Subscribes: /drone/command   (std_msgs/String, JSON from the planner)
            /drone/odometry  (nav_msgs/Odometry)
Publishes:  /drone/setpoint  (geometry_msgs/PoseStamped)
            /drone/guard     (std_msgs/String, accepted/clamped/rejected)
"""

from __future__ import annotations

import json
import math
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from ._repo import add_flight_package_to_path

add_flight_package_to_path()
from drone.commands import Envelope, Rejected, parse   # noqa: E402

from .sim_node import SENSOR_QOS, yaw_to_quaternion     # noqa: E402


class GuardNode(Node):
    def __init__(self) -> None:
        super().__init__("drone_guard")
        self.declare_parameter("max_altitude", 30.0)
        self.declare_parameter("geofence_radius", 60.0)
        self.declare_parameter("max_speed", 8.0)
        self.env = Envelope(
            max_altitude=self.get_parameter("max_altitude").value,
            geofence_radius=self.get_parameter("geofence_radius").value,
            max_speed=self.get_parameter("max_speed").value,
        )

        self.position = (0.0, 0.0, 0.0)
        self.setpoint_pub = self.create_publisher(PoseStamped, "/drone/setpoint", 10)
        self.report_pub = self.create_publisher(String, "/drone/guard", 10)
        self.create_subscription(String, "/drone/command", self.on_command, 10)
        self.create_subscription(Odometry, "/drone/odometry", self.on_odom, SENSOR_QOS)
        self.get_logger().info(
            f"guard up — ceiling {self.env.max_altitude}m, "
            f"fence {self.env.geofence_radius}m, max {self.env.max_speed}m/s")

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.position = (p.x, p.y, p.z)

    def report(self, verdict: str, detail: str) -> None:
        self.report_pub.publish(String(data=json.dumps({"verdict": verdict, "detail": detail})))

    def on_command(self, msg: String) -> None:
        try:
            cmd = parse(msg.data, self.env)
        except Rejected as exc:
            # Not flown. The planner reads this and can try again.
            self.get_logger().warn(f"rejected: {exc}")
            self.report("rejected", str(exc))
            return

        if cmd.verb in ("land", "rtl"):
            north, east, alt = (0.0, 0.0, 0.15) if cmd.verb == "rtl" else \
                               (self.position[0], self.position[1], 0.15)
        elif cmd.verb in ("hold",):
            north, east, alt = self.position[0], self.position[1], -self.position[2]
        elif cmd.verb == "takeoff":
            north, east, alt = self.position[0], self.position[1], cmd.altitude
        else:
            north, east, alt = cmd.north, cmd.east, cmd.altitude

        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "map"
        out.pose.position.x = float(north)
        out.pose.position.y = float(east)
        out.pose.position.z = float(-alt)          # NED: down is positive
        qx, qy, qz, qw = yaw_to_quaternion(float(cmd.yaw))
        out.pose.orientation.x = qx
        out.pose.orientation.y = qy
        out.pose.orientation.z = qz
        out.pose.orientation.w = qw
        self.setpoint_pub.publish(out)

        if cmd.clamped:
            self.get_logger().warn(f"{cmd.verb}: clamped {', '.join(cmd.clamped)}")
            self.report("clamped", "; ".join(cmd.clamped))
        else:
            self.get_logger().info(
                f"{cmd.verb} → N={north:.1f} E={east:.1f} alt={alt:.1f}")
            self.report("accepted", cmd.verb)


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = GuardNode()
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
