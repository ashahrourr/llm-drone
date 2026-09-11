"""Six-degree-of-freedom quadrotor dynamics.

State is integrated in the world frame using NED coordinates, matching PX4:
    x → North, y → East, z → Down (so altitude is negative z).

The vehicle is a rigid body with four rotors in an X configuration. Each rotor
produces thrust along the body -z axis and a reaction torque about it; the
mixer below converts the four rotor forces into a collective thrust and three
body moments.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

GRAVITY = 9.81


@dataclass
class Params:
    """Physical parameters, loosely modelled on a 250 mm racing quad."""

    mass: float = 0.9                      # kg
    arm: float = 0.125                     # rotor distance from centre, m
    # Diagonal inertia. A symmetric X-quad has Ixx ≈ Iyy, and Izz larger
    # because the rotors sit far from the yaw axis.
    inertia: np.ndarray = field(
        default_factory=lambda: np.array([4.9e-3, 4.9e-3, 8.8e-3])
    )
    thrust_coeff: float = 1.0e-5           # N per (rad/s)^2
    torque_coeff: float = 1.6e-7           # N·m per (rad/s)^2
    max_rotor_speed: float = 2200.0        # rad/s
    drag: float = 0.10                     # translational drag, N per (m/s)


def rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Body→world rotation for a Z-Y-X (yaw-pitch-roll) Euler sequence."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def euler_rates(roll: float, pitch: float, omega: np.ndarray) -> np.ndarray:
    """Body angular velocity → Euler angle rates.

    This is *not* the identity: the three Euler rates are measured about three
    different axes, so the body rates have to be resolved onto them. The matrix
    is singular at pitch = ±90°, the classic gimbal lock, which is why the
    controller keeps pitch well bounded.
    """
    cr, sr = np.cos(roll), np.sin(roll)
    cp, tp = np.cos(pitch), np.tan(pitch)
    cp = np.sign(cp) * max(abs(cp), 1e-6)      # guard the singularity
    return np.array([
        [1.0, sr * tp,  cr * tp],
        [0.0, cr,      -sr],
        [0.0, sr / cp,  cr / cp],
    ]) @ omega


class Quadrotor:
    """Rigid-body quadrotor integrated with fourth-order Runge-Kutta."""

    def __init__(self, params: Params | None = None, position=(0.0, 0.0, 0.0)):
        self.p = params or Params()
        # state = [pos(3), vel(3), euler(3), body rates(3)], all world/NED
        self.state = np.zeros(12)
        self.state[0:3] = np.asarray(position, dtype=float)
        self.rotor_speeds = np.zeros(4)

    # ---- accessors -------------------------------------------------------
    @property
    def position(self) -> np.ndarray:
        return self.state[0:3]

    @property
    def velocity(self) -> np.ndarray:
        return self.state[3:6]

    @property
    def euler(self) -> np.ndarray:
        return self.state[6:9]

    @property
    def altitude(self) -> float:
        """Height above the origin plane, in metres (positive up)."""
        return -self.state[2]

    # ---- dynamics --------------------------------------------------------
    def mix(self, rotor_speeds: np.ndarray) -> tuple[float, np.ndarray]:
        """Four rotor speeds → collective thrust and body moments.

        Rotor layout (X config, viewed from above, nose = +x = North), with
        L the half-diagonal so each rotor sits at (±L, ±L):

            0 front-right (+L, +L) CCW      2 front-left (+L, −L) CW
            3 rear-right  (−L, +L) CW       1 rear-left  (−L, −L) CCW

        Each rotor pushes along body −z with magnitude f, so its moment about
        the centre is r × F = (−y·f, x·f, 0). Summing gives roll from the
        left/right difference and pitch from the front/back difference. Yaw
        comes from reaction torque, which is why the two CCW rotors sit on a
        diagonal — it makes yaw independent of the other two axes.
        """
        w2 = np.clip(rotor_speeds, 0.0, self.p.max_rotor_speed) ** 2
        f = self.p.thrust_coeff * w2
        L = self.p.arm / np.sqrt(2.0)     # half-diagonal of an X layout
        c = self.p.torque_coeff / self.p.thrust_coeff

        thrust = f.sum()
        moments = np.array([
            L * (-f[0] + f[1] + f[2] - f[3]),   # roll:  more thrust left → roll right
            L * (f[0] - f[1] + f[2] - f[3]),    # pitch: more thrust front → nose up
            c * (f[0] + f[1] - f[2] - f[3]),    # yaw:   CCW pair vs CW pair
        ])
        return float(thrust), moments

    def derivative(self, state: np.ndarray, thrust: float, moments: np.ndarray) -> np.ndarray:
        vel = state[3:6]
        roll, pitch, yaw = state[6:9]
        omega = state[9:12]

        R = rotation_matrix(roll, pitch, yaw)
        # Thrust acts along body -z (up). Gravity is +z in NED.
        accel = (R @ np.array([0.0, 0.0, -thrust])) / self.p.mass
        accel[2] += GRAVITY
        accel -= self.p.drag * vel / self.p.mass

        # Euler's rigid-body equation: I·ω̇ = M − ω × (I·ω)
        I = self.p.inertia
        omega_dot = (moments - np.cross(omega, I * omega)) / I

        d = np.zeros(12)
        d[0:3] = vel
        d[3:6] = accel
        d[6:9] = euler_rates(roll, pitch, omega)
        d[9:12] = omega_dot
        return d

    def step(self, rotor_speeds: np.ndarray, dt: float) -> None:
        """Advance one timestep with RK4.

        Euler integration visibly loses energy at the 100–200 Hz rates used
        here, which shows up as a hover that slowly sinks. RK4 costs three
        extra evaluations and removes the artefact.
        """
        self.rotor_speeds = np.clip(rotor_speeds, 0.0, self.p.max_rotor_speed)
        thrust, moments = self.mix(self.rotor_speeds)

        s = self.state
        k1 = self.derivative(s, thrust, moments)
        k2 = self.derivative(s + 0.5 * dt * k1, thrust, moments)
        k3 = self.derivative(s + 0.5 * dt * k2, thrust, moments)
        k4 = self.derivative(s + dt * k3, thrust, moments)
        self.state = s + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

        # Wrap yaw into (-π, π] so heading errors stay small near the wrap.
        self.state[8] = (self.state[8] + np.pi) % (2 * np.pi) - np.pi

        # The ground is solid: clamp at z = 0 and kill downward motion.
        if self.state[2] > 0.0:
            self.state[2] = 0.0
            self.state[5] = min(self.state[5], 0.0)

    def hover_rotor_speed(self) -> float:
        """Rotor speed at which total thrust balances weight."""
        per_rotor = self.p.mass * GRAVITY / 4.0
        return float(np.sqrt(per_rotor / self.p.thrust_coeff))
