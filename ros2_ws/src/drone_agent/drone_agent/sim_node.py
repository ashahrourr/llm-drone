"""Vehicle node: integrates the dynamics and publishes state.

This is the node PX4 replaces. It subscribes to setpoints and publishes
odometry on the same topics and with the same NED conventions PX4 uses, so
swapping in PX4 SITL + Gazebo is a launch-file change rather than a rewrite.

Publishes:  /drone/odometry   (nav_msgs/Odometry, NED)
Subscribes: /drone/setpoint   (geometry_msgs/PoseStamped, NED + yaw)
"""

from __future__ import annotations

import math
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

# The flight code is plain numpy and lives outside the ROS package, so the same
# modules back both the standalone CLI and these nodes.
from ._repo import add_flight_package_to_path

add_flight_package_to_path()
from drone.control import Controller          # noqa: E402
from drone.dynamics import Params, Quadrotor   # noqa: E402

RATE_HZ = 200.0

# PX4 publishes telemetry best-effort: it is a firehose where the newest sample
# matters and a dropped one is not worth retransmitting. Matching that here
# means a subscriber written against this node also works against PX4.
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class SimNode(Node):
    def __init__(self) -> None:
        super().__init__("drone_sim")
        params = Params()
        self.drone = Quadrotor(params)
        self.control = Controller(params)
        self.target = np.array([0.0, 0.0, 0.0])
        self.target_yaw = 0.0
        self.have_setpoint = False

        self.odom_pub = self.create_publisher(Odometry, "/drone/odometry", SENSOR_QOS)
        self.create_subscription(PoseStamped, "/drone/setpoint", self.on_setpoint, 10)
        self.create_timer(1.0 / RATE_HZ, self.tick)
        self.get_logger().info(f"vehicle up, integrating at {RATE_HZ:.0f} Hz")

    def on_setpoint(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self.target = np.array([p.x, p.y, p.z])
        q = msg.pose.orientation
        self.target_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.have_setpoint = True

    def tick(self) -> None:
        dt = 1.0 / RATE_HZ
        if self.have_setpoint:
            rotors = self.control(self.drone.state, self.target, self.target_yaw, dt)
        else:
            rotors = np.zeros(4)          # sit on the ground until commanded
        self.drone.step(rotors, dt)

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.child_frame_id = "base_link"
        pos, vel = self.drone.position, self.drone.velocity
        msg.pose.pose.position.x = float(pos[0])
        msg.pose.pose.position.y = float(pos[1])
        msg.pose.pose.position.z = float(pos[2])
        qx, qy, qz, qw = yaw_to_quaternion(float(self.drone.euler[2]))
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = float(vel[0])
        msg.twist.twist.linear.y = float(vel[1])
        msg.twist.twist.linear.z = float(vel[2])
        self.odom_pub.publish(msg)


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = SimNode()
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
