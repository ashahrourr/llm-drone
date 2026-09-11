"""MAVLink interface to the simulated vehicle.

MAVLink is the protocol real flight controllers speak — a compact binary format
carried over serial or UDP. Exposing the simulator over it means anything that
talks to a drone can talk to this one: QGroundControl, MAVSDK, `mavproxy`, or a
companion computer.

Two halves:

    MavlinkVehicle   the vehicle side. Streams HEARTBEAT and telemetry, and
                     accepts position setpoints and commands.
    MavlinkLink      the operator side. Waits for a heartbeat, then sends
                     setpoints — what a ground station does.

Frames follow the MAVLink convention, which is the same NED used throughout
this project, so nothing has to be converted at the boundary.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

from pymavlink import mavutil

# MAV_CMD values used here, spelled out rather than magic numbers.
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_NAV_LAND = 21
MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
MAV_CMD_COMPONENT_ARM_DISARM = 400

# Bitmask for SET_POSITION_TARGET_LOCAL_NED: ignore everything except x, y, z
# and yaw. MAVLink's mask is inverted — a set bit means "ignore this field".
IGNORE_VEL = 0b0000_1111_1111_1000
POSITION_AND_YAW = IGNORE_VEL & ~0b0000_1000_0000_0000     # keep yaw too

HEARTBEAT_HZ = 1.0
TELEMETRY_HZ = 20.0


@dataclass
class Setpoint:
    north: float = 0.0
    east: float = 0.0
    down: float = 0.0
    yaw: float = 0.0


class MavlinkVehicle:
    """Serves the simulated drone over MAVLink.

    Point QGroundControl at udpin:0.0.0.0:14550 (or run MavlinkLink) and this
    behaves like a vehicle: heartbeats, position telemetry, and acceptance of
    setpoints and commands.
    """

    def __init__(self, endpoint: str = "udpin:0.0.0.0:14550",
                 system_id: int = 1, component_id: int = 1):
        self.conn = mavutil.mavlink_connection(endpoint, source_system=system_id,
                                               source_component=component_id)
        self.boot = time.time()
        self.armed = False
        self.setpoint = Setpoint()
        self._last_heartbeat = 0.0
        self._last_telemetry = 0.0

    def _ms(self) -> int:
        return int((time.time() - self.boot) * 1000)

    # ---- outgoing ------------------------------------------------------
    def send_heartbeat(self) -> None:
        base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        if self.armed:
            base_mode |= mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        self.conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_QUADROTOR,
            mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
            base_mode, 0,
            mavutil.mavlink.MAV_STATE_ACTIVE if self.armed
            else mavutil.mavlink.MAV_STATE_STANDBY)

    def send_telemetry(self, position, velocity, euler) -> None:
        """LOCAL_POSITION_NED + ATTITUDE, the two a ground station plots."""
        self.conn.mav.local_position_ned_send(
            self._ms(),
            float(position[0]), float(position[1]), float(position[2]),
            float(velocity[0]), float(velocity[1]), float(velocity[2]))
        self.conn.mav.attitude_send(
            self._ms(),
            float(euler[0]), float(euler[1]), float(euler[2]),
            0.0, 0.0, 0.0)

    def pump(self, position, velocity, euler) -> None:
        """Send whatever is due. Call this every control tick."""
        now = time.time()
        if now - self._last_heartbeat >= 1.0 / HEARTBEAT_HZ:
            self.send_heartbeat()
            self._last_heartbeat = now
        if now - self._last_telemetry >= 1.0 / TELEMETRY_HZ:
            self.send_telemetry(position, velocity, euler)
            self._last_telemetry = now

    # ---- incoming ------------------------------------------------------
    def poll(self) -> list[str]:
        """Drain pending messages. Returns a log of what was acted on."""
        acted: list[str] = []
        while True:
            msg = self.conn.recv_match(blocking=False)
            if msg is None:
                return acted
            kind = msg.get_type()

            if kind == "SET_POSITION_TARGET_LOCAL_NED":
                self.setpoint = Setpoint(msg.x, msg.y, msg.z, msg.yaw)
                acted.append(f"setpoint N={msg.x:.1f} E={msg.y:.1f} D={msg.z:.1f}")

            elif kind == "COMMAND_LONG":
                acted.append(self._command(msg))

    def _command(self, msg) -> str:
        cmd = msg.command
        result = mavutil.mavlink.MAV_RESULT_ACCEPTED

        if cmd == MAV_CMD_COMPONENT_ARM_DISARM:
            self.armed = msg.param1 >= 0.5
            note = "armed" if self.armed else "disarmed"
        elif cmd == MAV_CMD_NAV_TAKEOFF:
            altitude = msg.param7 or 5.0
            self.setpoint = Setpoint(self.setpoint.north, self.setpoint.east,
                                     -abs(altitude), self.setpoint.yaw)
            self.armed = True
            note = f"takeoff to {abs(altitude):.1f}m"
        elif cmd == MAV_CMD_NAV_LAND:
            self.setpoint = Setpoint(self.setpoint.north, self.setpoint.east,
                                     -0.15, self.setpoint.yaw)
            note = "land"
        elif cmd == MAV_CMD_NAV_RETURN_TO_LAUNCH:
            self.setpoint = Setpoint(0.0, 0.0, -0.15, self.setpoint.yaw)
            note = "return to launch"
        else:
            result = mavutil.mavlink.MAV_RESULT_UNSUPPORTED
            note = f"unsupported command {cmd}"

        self.conn.mav.command_ack_send(cmd, result)
        return note


class MavlinkLink:
    """Operator side: what a ground station or companion computer does."""

    def __init__(self, endpoint: str = "udpout:127.0.0.1:14550",
                 system_id: int = 255, component_id: int = 190):
        self.conn = mavutil.mavlink_connection(endpoint, source_system=system_id,
                                               source_component=component_id)
        self.target_system = 1
        self.target_component = 1

    def wait_heartbeat(self, timeout: float = 10.0) -> bool:
        # A ground station has to hear the vehicle before addressing it, since
        # the system id to target comes from the heartbeat itself.
        self.conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        msg = self.conn.recv_match(type="HEARTBEAT", blocking=True, timeout=timeout)
        if msg is None:
            return False
        self.target_system = msg.get_srcSystem()
        self.target_component = msg.get_srcComponent()
        return True

    def command(self, cmd: int, *params: float) -> None:
        padded = (list(params) + [0.0] * 7)[:7]
        self.conn.mav.command_long_send(self.target_system, self.target_component,
                                        cmd, 0, *padded)

    def arm(self) -> None:
        self.command(MAV_CMD_COMPONENT_ARM_DISARM, 1.0)

    def takeoff(self, altitude: float) -> None:
        self.command(MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, altitude)

    def rtl(self) -> None:
        self.command(MAV_CMD_NAV_RETURN_TO_LAUNCH)

    def goto(self, north: float, east: float, altitude: float, yaw: float = 0.0) -> None:
        self.conn.mav.set_position_target_local_ned_send(
            0, self.target_system, self.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            POSITION_AND_YAW,
            float(north), float(east), float(-abs(altitude)),
            0, 0, 0, 0, 0, 0, float(yaw), 0)

    def telemetry(self, timeout: float = 2.0):
        return self.conn.recv_match(type="LOCAL_POSITION_NED", blocking=True,
                                    timeout=timeout)
