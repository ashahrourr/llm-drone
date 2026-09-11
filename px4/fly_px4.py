#!/usr/bin/env python3
"""Fly real PX4 firmware over MAVLink, the way a companion computer does.

Run through ./px4/fly.sh, which starts PX4 SITL first. The sequence is the one
PX4 actually requires, and both steps are easy to get wrong:

  1. Stream position setpoints *before* asking for OFFBOARD. PX4 refuses the
     mode without a live setpoint stream, and drops out of it if the stream
     stops, so it runs on a background thread for the whole flight.
  2. Retry arming. The first attempts come back TEMPORARILY_REJECTED while the
     EKF converges and home is set — that is normal, not a failure.
"""

from __future__ import annotations

import threading
import time

from pymavlink import mavutil

RESULT = {0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED",
          3: "UNSUPPORTED", 4: "FAILED", 5: "IN_PROGRESS"}

# Position-only setpoint: every other field is ignored (MAVLink's mask is
# inverted, so a set bit means "ignore this one").
POSITION_ONLY = 0b0000_1111_1111_1000
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6


class PX4:
    def __init__(self, endpoint: str = "udpin:0.0.0.0:14550"):
        self.conn = mavutil.mavlink_connection(endpoint)
        self.target = {"north": 0.0, "east": 0.0, "down": -10.0}
        self._stop = threading.Event()

    # ---- setup ---------------------------------------------------------
    def connect(self, timeout: float = 40.0) -> None:
        if not self.conn.wait_heartbeat(timeout=timeout):
            raise SystemExit("no PX4 heartbeat — is SITL running?")
        print(f"connected to PX4 (system {self.conn.target_system})")
        self.request_stream(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 20)

    def request_stream(self, msg_id: int, hz: float) -> None:
        self.command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                     msg_id, int(1e6 / hz), wait=False)

    def wait_for_home(self, timeout: float = 90.0) -> bool:
        print("waiting for the EKF to converge and home to be set...")
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.conn.recv_match(type=["HOME_POSITION", "GPS_GLOBAL_ORIGIN"],
                                    blocking=True, timeout=5):
                print("  home set")
                return True
        print("  no home position")
        return False

    # ---- messaging -----------------------------------------------------
    def command(self, cmd: int, *params: float, wait: bool = True) -> str:
        padded = (list(params) + [0.0] * 7)[:7]
        self.conn.mav.command_long_send(self.conn.target_system,
                                        self.conn.target_component, cmd, 0, *padded)
        if not wait:
            return ""
        ack = self.conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
        return RESULT.get(ack.result, str(ack.result)) if ack else "no-ack"

    def send_setpoint(self) -> None:
        t = self.target
        self.conn.mav.set_position_target_local_ned_send(
            0, self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, POSITION_ONLY,
            t["north"], t["east"], t["down"], 0, 0, 0, 0, 0, 0, 0, 0)

    def start_streaming(self, hz: float = 20.0) -> None:
        def loop() -> None:
            while not self._stop.is_set():
                self.send_setpoint()
                time.sleep(1.0 / hz)
        threading.Thread(target=loop, daemon=True).start()
        time.sleep(1.5)      # let a few land before the mode switch

    def stop(self) -> None:
        self._stop.set()

    # ---- flight --------------------------------------------------------
    def offboard(self) -> str:
        return self.command(mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                            1, PX4_CUSTOM_MAIN_MODE_OFFBOARD, 0)

    def arm(self, attempts: int = 20) -> bool:
        for i in range(attempts):
            result = self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1.0)
            if result == "ACCEPTED":
                print(f"ARMED after {i + 1} attempt(s)")
                return True
            time.sleep(2.0)
        print("never armed")
        return False

    def position(self, timeout: float = 2.0):
        return self.conn.recv_match(type="LOCAL_POSITION_NED",
                                    blocking=True, timeout=timeout)

    def goto(self, north: float, east: float, altitude: float,
             seconds: float = 28.0, label: str = "") -> None:
        self.target.update(north=north, east=east, down=-abs(altitude))
        if label:
            print(label)
        deadline = time.time() + seconds
        last_report = 0.0
        while time.time() < deadline:
            msg = self.position(timeout=1.0)
            if not msg:
                continue
            gap = ((msg.x - north) ** 2 + (msg.y - east) ** 2
                   + (msg.z + abs(altitude)) ** 2) ** 0.5
            now = time.time()
            if now - last_report > 2.0:      # readable in a log, not a flood
                print(f"  N={msg.x:6.1f} E={msg.y:6.1f} alt={-msg.z:5.1f}  gap={gap:5.2f} m")
                last_report = now
            if gap < 1.5:
                print(f"  arrived: N={msg.x:.1f} E={msg.y:.1f} alt={-msg.z:.1f}")
                return


def main() -> int:
    px4 = PX4()
    px4.connect()
    px4.wait_for_home()
    px4.start_streaming()

    print("OFFBOARD:", px4.offboard())
    if not px4.arm():
        px4.stop()
        return 1

    px4.goto(0.0, 0.0, 10.0, seconds=35, label="climbing to 10 m")
    px4.goto(20.0, 15.0, 10.0, seconds=40, label="translating to N=20 E=15")
    px4.goto(0.0, 0.0, 10.0, seconds=45, label="returning")

    print("land:", px4.command(mavutil.mavlink.MAV_CMD_NAV_LAND))
    last = 0.0
    for _ in range(90):
        msg = px4.position(timeout=1.0)
        if not msg:
            continue
        if time.time() - last > 2.0:
            print(f"  alt {-msg.z:5.1f} m")
            last = time.time()
        if -msg.z < 0.5:
            break
    print("done")
    px4.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
