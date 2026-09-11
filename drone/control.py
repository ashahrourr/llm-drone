"""Cascaded position/attitude controller.

Structure, outer loop to inner:

    position error → desired acceleration   (PID, ~2 Hz bandwidth)
    desired acceleration → desired tilt     (algebraic inversion)
    attitude error → body moments           (PD, ~10 Hz bandwidth)
    thrust + moments → rotor speeds         (mixer inversion)

The loops are separated by roughly a decade of bandwidth so the inner one
looks instantaneous to the outer one, which is what lets them be tuned
independently.
"""

from __future__ import annotations

import numpy as np

from .dynamics import GRAVITY, Params, rotation_matrix


class PID:
    def __init__(self, kp: float, ki: float, kd: float, i_limit: float = 4.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit = i_limit
        self.integral = 0.0
        self.prev_error: float | None = None

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_error = None

    def __call__(self, error: float, dt: float) -> float:
        self.integral = float(np.clip(self.integral + error * dt, -self.i_limit, self.i_limit))
        derivative = 0.0 if self.prev_error is None else (error - self.prev_error) / dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative


class Controller:
    """Drives the quadrotor to a position setpoint and yaw."""

    MAX_TILT = np.radians(30.0)     # keeps the small-angle inversion honest
    MAX_CLIMB = 4.0                 # m/s

    # Closed-loop bandwidths, rad/s. The attitude loop is placed a decade above
    # the position loop so the cascade's timescale-separation assumption holds;
    # tuned by hand they ended up within a factor of three of each other, and
    # the two loops fought until the vehicle oscillated.
    W_ATTITUDE = 30.0
    W_YAW = 15.0
    DAMPING = 0.85

    def __init__(self, params: Params):
        self.p = params
        # Outer loop: position → acceleration. ω ≈ 1.2 rad/s.
        self.pid_x = PID(1.4, 0.05, 2.2)
        self.pid_y = PID(1.4, 0.05, 2.2)
        self.pid_z = PID(6.0, 1.20, 4.5)

        # Inner loop: attitude → moments, as a second-order system per axis.
        # I·θ̈ = −kp·θ − kd·θ̇  ⇒  kp = I·ω²,  kd = 2ζ·√(kp·I)
        I = self.p.inertia
        w = np.array([self.W_ATTITUDE, self.W_ATTITUDE, self.W_YAW])
        self.att_gain = I * w ** 2
        self.rate_gain = 2.0 * self.DAMPING * np.sqrt(self.att_gain * I)

    def reset(self) -> None:
        for pid in (self.pid_x, self.pid_y, self.pid_z):
            pid.reset()

    def __call__(self, state: np.ndarray, target: np.ndarray,
                 target_yaw: float, dt: float) -> np.ndarray:
        pos, vel = state[0:3], state[3:6]
        roll, pitch, yaw = state[6:9]
        omega = state[9:12]

        # ---- outer loop: where do we want to accelerate? -----------------
        ax = self.pid_x(target[0] - pos[0], dt)
        ay = self.pid_y(target[1] - pos[1], dt)
        # z is Down, so a positive altitude error means we need negative az.
        az = self.pid_z(target[2] - pos[2], dt)
        az = float(np.clip(az, -self.MAX_CLIMB, self.MAX_CLIMB))

        # Total thrust must cancel gravity and supply the vertical demand.
        # Dividing by cos(roll)cos(pitch) compensates for the tilt: when the
        # vehicle leans, less of its thrust points up.
        tilt_comp = max(np.cos(roll) * np.cos(pitch), 0.5)
        thrust = self.p.mass * (GRAVITY - az) / tilt_comp
        thrust = float(np.clip(thrust, 0.0, 4.0 * self.p.thrust_coeff * self.p.max_rotor_speed ** 2))

        # ---- attitude the horizontal demand implies ----------------------
        # Rotate the world-frame demand into the heading frame first, so the
        # vehicle leans toward the target regardless of which way it faces.
        cy, sy = np.cos(yaw), np.sin(yaw)
        ax_b =  ax * cy + ay * sy
        ay_b = -ax * sy + ay * cy
        # Signs follow from the rotation matrix: +roll accelerates East,
        # +pitch accelerates South. So pitch is negated, roll is not.
        scale = self.p.mass / max(thrust, 1e-6)
        des_pitch = float(np.clip(np.arcsin(np.clip(-ax_b * scale, -1, 1)), -self.MAX_TILT, self.MAX_TILT))
        des_roll = float(np.clip(np.arcsin(np.clip(ay_b * scale, -1, 1)), -self.MAX_TILT, self.MAX_TILT))

        # ---- inner loop: attitude → moments ------------------------------
        yaw_error = (target_yaw - yaw + np.pi) % (2 * np.pi) - np.pi
        att_error = np.array([des_roll - roll, des_pitch - pitch, yaw_error])
        moments = self.att_gain * att_error - self.rate_gain * omega

        return self.to_rotor_speeds(thrust, moments)

    def to_rotor_speeds(self, thrust: float, moments: np.ndarray) -> np.ndarray:
        """Invert the mixer: (thrust, moments) → four rotor speeds.

        Exact inverse of Quadrotor.mix, which is a full-rank 4x4 map, so this
        is a closed-form solve rather than a least-squares fit. Negative
        solutions mean the demand is outside what four upward-only rotors can
        produce; clamping at zero degrades gracefully instead of failing.
        """
        L = self.p.arm / np.sqrt(2.0)
        kf = self.p.thrust_coeff
        c = self.p.torque_coeff / kf
        mr, mp, my = moments
        t4, r4, p4, y4 = thrust / 4.0, mr / (4 * L), mp / (4 * L), my / (4 * c)

        f = np.array([
            t4 - r4 + p4 + y4,     # 0 front-right
            t4 + r4 - p4 + y4,     # 1 rear-left
            t4 + r4 + p4 - y4,     # 2 front-left
            t4 - r4 - p4 - y4,     # 3 rear-right
        ])
        return np.sqrt(np.clip(f, 0.0, None) / kf)
